//! Shared UART event loop for hardware-backed and simulated robot daemons.

use crate::{
    pca9685::PwmOutput,
    robot_link::{UartFrameEncoder, UartStreamDecoder},
    robot_service::{RobotService, ServiceMessage},
};
use anyhow::{bail, Context, Result};
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

pub struct UartDaemonOptions<'a> {
    pub process_name: &'a str,
    pub uart_device: &'a Path,
    pub skip_uart_config: bool,
}

pub fn run_uart_daemon<D>(
    options: UartDaemonOptions<'_>,
    mut service: RobotService<D>,
) -> Result<()>
where
    D: PwmOutput,
{
    if !options.skip_uart_config {
        configure_uart(options.uart_device)?;
    }

    let uart = OpenOptions::new()
        .read(true)
        .write(true)
        .open(options.uart_device)
        .with_context(|| format!("failed to open UART {}", options.uart_device.display()))?;
    let reader = uart
        .try_clone()
        .context("failed to clone UART for reader thread")?;
    let mut writer = uart;
    let receiver = spawn_uart_reader(reader);
    let mut decoder = UartStreamDecoder::default();
    let mut encoder = UartFrameEncoder::default();

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))
        .context("failed to install Ctrl-C shutdown handler")?;

    eprintln!(
        "{} ready uart={} pose=unknown; send RecoverToOpen first",
        options.process_name,
        options.uart_device.display()
    );

    let run_result = run_event_loop(
        &running,
        &receiver,
        &mut decoder,
        &mut encoder,
        &mut writer,
        &mut service,
    );
    let shutdown_result = service.shutdown();

    match (run_result, shutdown_result) {
        (Ok(()), Ok(())) => {
            eprintln!("{} stopped; outputs=all_off", options.process_name);
            Ok(())
        }
        (Err(error), Ok(())) => Err(error),
        (Ok(()), Err(error)) => Err(error).context("failed to disable outputs on shutdown"),
        (Err(run_error), Err(shutdown_error)) => Err(run_error).context(format!(
            "daemon failed and outputs could not be disabled: {shutdown_error:#}"
        )),
    }
}

fn run_event_loop<D>(
    running: &AtomicBool,
    receiver: &mpsc::Receiver<std::io::Result<Vec<u8>>>,
    decoder: &mut UartStreamDecoder,
    encoder: &mut UartFrameEncoder,
    writer: &mut File,
    service: &mut RobotService<D>,
) -> Result<()>
where
    D: PwmOutput,
{
    while running.load(Ordering::SeqCst) {
        match receiver.recv_timeout(Duration::from_millis(10)) {
            Ok(Ok(bytes)) => {
                for byte in bytes {
                    match decoder.push(byte) {
                        Some(Ok(packet)) => {
                            let messages = service.handle_packet(&packet, Instant::now());
                            write_messages(writer, encoder, &messages)?;
                        }
                        Some(Err(error)) => eprintln!("discarded UART frame: {error:?}"),
                        None => {}
                    }
                }
            }
            Ok(Err(error)) => return Err(error).context("UART reader failed"),
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => bail!("UART reader stopped"),
        }

        let messages = service.tick(Instant::now());
        write_messages(writer, encoder, &messages)?;
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
    writer: &mut File,
    encoder: &mut UartFrameEncoder,
    messages: &[ServiceMessage],
) -> Result<()> {
    for message in messages {
        let frame = message.encode_uart(encoder)?;
        writer
            .write_all(frame)
            .context("failed to write UART frame")?;
    }
    writer.flush().context("failed to flush UART responses")?;
    Ok(())
}
