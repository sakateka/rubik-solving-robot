//! Shared UART event loop for hardware-backed and simulated robot daemons.

use crate::{
    pca9685::PwmOutput,
    robot_link::{ReceivedPacket, UartFrameEncoder, UartStreamDecoder},
    robot_service::{FaceScanner, RobotService, ServiceMessage},
};
use anyhow::{bail, Context, Result};
use rubik_link_protocol as link;
use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
    path::Path,
    process::Command,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    time::{Duration, Instant},
};

/// Receives telemetry from the daemon loop alongside the primary transport.
///
/// Implementations run on the daemon thread and must never block: every
/// callback is invoked from the same loop that services UART traffic.
pub trait DaemonObserver: Send {
    fn observe_status(&mut self, _status: &link::StatusSnapshot) {}
    fn observe_request(&mut self, _packet: &ReceivedPacket) {}
    fn observe_messages(&mut self, _messages: &[ServiceMessage]) {}
}

/// Bridge between the daemon loop and auxiliary transports such as the
/// simulation HTTP server.
///
/// Frames pushed through the returned sender are decoded by the daemon loop
/// exactly like UART bytes, so HTTP-injected commands produce protocol
/// responses (and events) that are also mirrored onto the real UART when one
/// is attached.
pub struct DaemonHub {
    observer: Box<dyn DaemonObserver>,
    inbound: mpsc::Receiver<Vec<u8>>,
}

impl DaemonHub {
    /// Creates a hub plus the sender auxiliary clients use to inject frames.
    pub fn new(observer: Box<dyn DaemonObserver>) -> (Self, mpsc::Sender<Vec<u8>>) {
        let (sender, receiver) = mpsc::channel();
        (
            Self {
                observer,
                inbound: receiver,
            },
            sender,
        )
    }

    fn try_recv_inbound(&mut self) -> Option<Vec<u8>> {
        self.inbound.try_recv().ok()
    }
}

pub struct UartDaemonOptions<'a> {
    pub process_name: &'a str,
    /// `None` runs without a UART transport (for example an HTTP-only sim).
    pub uart_device: Option<&'a Path>,
    pub skip_uart_config: bool,
    pub hub: Option<DaemonHub>,
}

pub fn run_uart_daemon<D, S>(
    options: UartDaemonOptions<'_>,
    mut service: RobotService<D, S>,
) -> Result<()>
where
    D: PwmOutput,
    S: FaceScanner,
{
    let UartDaemonOptions {
        process_name,
        uart_device,
        skip_uart_config,
        mut hub,
    } = options;

    let (writer, receiver) = match uart_device {
        Some(path) => {
            if !skip_uart_config {
                configure_uart(path)?;
            }
            let uart = OpenOptions::new()
                .read(true)
                .write(true)
                .open(path)
                .with_context(|| format!("failed to open UART {}", path.display()))?;
            let reader = uart
                .try_clone()
                .context("failed to clone UART for reader thread")?;
            (Some(uart), Some(spawn_uart_reader(reader)))
        }
        None => (None, None),
    };
    let mut decoder = UartStreamDecoder::default();
    let mut encoder = UartFrameEncoder::default();

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))
        .context("failed to install Ctrl-C shutdown handler")?;

    match uart_device {
        Some(path) => eprintln!(
            "{} ready uart={} pose=unknown; send RecoverToOpen first",
            process_name,
            path.display()
        ),
        None => eprintln!(
            "{process_name} ready uart=none pose=unknown; send RecoverToOpen first"
        ),
    }

    if let Some(hub) = hub.as_mut() {
        hub.observer.observe_status(service.status());
    }

    let run_result = run_event_loop(
        &running,
        receiver.as_ref(),
        &mut decoder,
        &mut encoder,
        writer.as_ref(),
        &mut service,
        hub.as_mut(),
    );
    let shutdown_result = service.shutdown();

    match (run_result, shutdown_result) {
        (Ok(()), Ok(())) => {
            eprintln!("{} stopped; outputs=all_off", process_name);
            Ok(())
        }
        (Err(error), Ok(())) => Err(error),
        (Ok(()), Err(error)) => Err(error).context("failed to disable outputs on shutdown"),
        (Err(run_error), Err(shutdown_error)) => Err(run_error).context(format!(
            "daemon failed and outputs could not be disabled: {shutdown_error:#}"
        )),
    }
}

fn run_event_loop<D, S>(
    running: &AtomicBool,
    receiver: Option<&mpsc::Receiver<std::io::Result<Vec<u8>>>>,
    decoder: &mut UartStreamDecoder,
    encoder: &mut UartFrameEncoder,
    writer: Option<&File>,
    service: &mut RobotService<D, S>,
    mut hub: Option<&mut DaemonHub>,
) -> Result<()>
where
    D: PwmOutput,
    S: FaceScanner,
{
    while running.load(Ordering::SeqCst) {
        if let Some(hub) = hub.as_mut() {
            while let Some(bytes) = hub.try_recv_inbound() {
                for byte in bytes {
                    match decoder.push(byte) {
                        Some(Ok(packet)) => {
                            hub.observer.observe_request(&packet);
                            let messages = service.handle_packet(&packet, Instant::now());
                            write_messages(writer, encoder, &messages)?;
                            hub.observer.observe_messages(&messages);
                        }
                        Some(Err(error)) => eprintln!("discarded inbound frame: {error:?}"),
                        None => {}
                    }
                }
            }
        }

        match receiver {
            Some(receiver) => match receiver.recv_timeout(Duration::from_millis(10)) {
                Ok(Ok(bytes)) => {
                    for byte in bytes {
                        match decoder.push(byte) {
                            Some(Ok(packet)) => {
                                if let Some(hub) = hub.as_mut() {
                                    hub.observer.observe_request(&packet);
                                }
                                let messages = service.handle_packet(&packet, Instant::now());
                                write_messages(writer, encoder, &messages)?;
                                if let Some(hub) = hub.as_mut() {
                                    hub.observer.observe_messages(&messages);
                                }
                            }
                            Some(Err(error)) => eprintln!("discarded UART frame: {error:?}"),
                            None => {}
                        }
                    }
                }
                Ok(Err(error)) => return Err(error).context("UART reader failed"),
                Err(mpsc::RecvTimeoutError::Timeout) => {}
                Err(mpsc::RecvTimeoutError::Disconnected) => bail!("UART reader stopped"),
            },
            None => std::thread::sleep(Duration::from_millis(10)),
        }

        let messages = service.tick(Instant::now());
        write_messages(writer, encoder, &messages)?;
        if let Some(hub) = hub.as_mut() {
            hub.observer.observe_status(service.status());
            hub.observer.observe_messages(&messages);
        }
    }
    Ok(())
}

fn configure_uart(path: &Path) -> Result<()> {
    let status = Command::new("stty")
        .args(["-F"])
        .arg(path)
        .args(["115200", "raw", "-echo", "-ixon", "-ixoff"])
        .status()
        .with_context(|| format!("failed to run stty for {}", path.display()))?;
    if !status.success() {
        bail!("stty failed for {} with {status}", path.display());
    }
    Ok(())
}

fn spawn_uart_reader(mut reader: File) -> mpsc::Receiver<std::io::Result<Vec<u8>>> {
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || loop {
        let mut buffer = [0u8; 128];
        match reader.read(&mut buffer) {
            Ok(0) => {
                let _ = sender.send(Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "UART returned EOF",
                )));
                break;
            }
            Ok(count) => {
                if sender.send(Ok(buffer[..count].to_vec())).is_err() {
                    break;
                }
            }
            Err(error) => {
                let _ = sender.send(Err(error));
                break;
            }
        }
    });
    receiver
}

fn write_messages(
    writer: Option<&File>,
    encoder: &mut UartFrameEncoder,
    messages: &[ServiceMessage],
) -> Result<()> {
    let Some(writer) = writer else {
        return Ok(());
    };
    let mut writer = writer;
    for message in messages {
        let frame = message.encode_uart(encoder)?;
        writer
            .write_all(frame)
            .context("failed to write UART frame")?;
    }
    writer.flush().context("failed to flush UART responses")?;
    Ok(())
}
