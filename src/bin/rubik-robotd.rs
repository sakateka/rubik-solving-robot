//! UART-facing, deadline-driven robot control daemon.

use anyhow::{bail, Context, Result};
use clap::Parser;
use rubik_link_protocol as link;
use rubik_scan::{
    pca9685::Pca9685,
    robot_link::UartStreamDecoder,
    robot_service::{EventMessage, ResponsePayload, RobotService, ServiceMessage},
    stand::StandCalibration,
};
use serde::Serialize;
use std::{
    fs::{File, OpenOptions},
    io::{Read, Write},
    path::{Path, PathBuf},
    process::Command,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    time::{Duration, Instant},
};

#[derive(Parser)]
#[command(about = "Run the UART robot control service")]
struct Cli {
    /// Duo UART connected to ESP32-C6
    #[arg(long, default_value = "/dev/ttyS1")]
    uart_device: PathBuf,

    /// Do not invoke stty; use an already-configured raw UART
    #[arg(long)]
    skip_uart_config: bool,

    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// Optional TOML file overriding built-in stand calibration
    #[arg(long)]
    config: Option<PathBuf>,

    /// Servo PWM frequency
    #[arg(long, default_value_t = 50.0)]
    pwm_hz: f64,

    /// Required acknowledgement that remote commands may move the stand
    #[arg(long)]
    confirm_stand_motion: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if !cli.confirm_stand_motion {
        bail!(
            "refusing to start robot daemon; pass --confirm-stand-motion after checking the stand"
        );
    }
    if !cli.skip_uart_config {
        configure_uart(&cli.uart_device)?;
    }

    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let mut output = Pca9685::open(&cli.i2c_device, cli.address)?;
    let pwm = output.initialize_safe_pwm(cli.pwm_hz)?;
    let mut service = RobotService::new(output, calibration);

    let uart = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&cli.uart_device)
        .with_context(|| format!("failed to open UART {}", cli.uart_device.display()))?;
    let reader = uart
        .try_clone()
        .context("failed to clone UART for reader thread")?;
    let mut writer = uart;
    let receiver = spawn_uart_reader(reader);
    let mut decoder = UartStreamDecoder::default();
    let mut encoder = MessageEncoder::default();

    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))
        .context("failed to install Ctrl-C shutdown handler")?;

    eprintln!(
        "rubik-robotd ready uart={} pwm_hz={:.3} pose=unknown; send RecoverToOpen first",
        cli.uart_device.display(),
        pwm.pwm_hz()
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
            eprintln!("rubik-robotd stopped; outputs=all_off");
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
    encoder: &mut MessageEncoder,
    writer: &mut File,
    service: &mut RobotService<D>,
) -> Result<()>
where
    D: rubik_scan::pca9685::PwmOutput,
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

struct MessageEncoder {
    payload: [u8; link::MAX_PAYLOAD_LEN],
    packet: [u8; link::MAX_PACKET_LEN],
    frame: [u8; link::MAX_UART_FRAME_LEN],
}

impl Default for MessageEncoder {
    fn default() -> Self {
        Self {
            payload: [0; link::MAX_PAYLOAD_LEN],
            packet: [0; link::MAX_PACKET_LEN],
            frame: [0; link::MAX_UART_FRAME_LEN],
        }
    }
}

impl MessageEncoder {
    fn encode<T: Serialize + ?Sized>(
        &mut self,
        kind: link::MessageKind,
        opcode: u16,
        request_id: u32,
        payload: &T,
    ) -> Result<&[u8]> {
        let payload = link::encode_payload(payload, &mut self.payload)
            .map_err(|error| anyhow::anyhow!("failed to encode protocol payload: {error}"))?;
        let packet = link::Packet {
            kind,
            opcode,
            request_id,
            payload,
        };
        let length = link::frame_uart(packet, &mut self.packet, &mut self.frame)
            .map_err(|error| anyhow::anyhow!("failed to frame protocol message: {error}"))?;
        Ok(&self.frame[..length])
    }
}

fn write_messages(
    writer: &mut File,
    encoder: &mut MessageEncoder,
    messages: &[ServiceMessage],
) -> Result<()> {
    for message in messages {
        let frame = match message {
            ServiceMessage::Response(response) => match &response.payload {
                ResponsePayload::Accepted(payload) => encoder.encode(
                    link::MessageKind::Response,
                    response.opcode.into(),
                    response.request_id,
                    payload,
                )?,
                ResponsePayload::Rejected(payload) => encoder.encode(
                    link::MessageKind::Response,
                    response.opcode.into(),
                    response.request_id,
                    payload,
                )?,
                ResponsePayload::Status(payload) => encoder.encode(
                    link::MessageKind::Response,
                    response.opcode.into(),
                    response.request_id,
                    payload.as_ref(),
                )?,
            },
            ServiceMessage::Event(event) => match event {
                EventMessage::RobotStateChanged(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::RobotStateChanged.into(),
                    0,
                    payload,
                )?,
                EventMessage::StandStateChanged(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::StandStateChanged.into(),
                    0,
                    payload,
                )?,
                EventMessage::CubeSessionChanged(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::CubeSessionChanged.into(),
                    0,
                    payload,
                )?,
                EventMessage::OperationCompleted(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::OperationCompleted.into(),
                    0,
                    payload,
                )?,
                EventMessage::Aborted(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::Aborted.into(),
                    0,
                    payload,
                )?,
                EventMessage::Fault(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::Fault.into(),
                    0,
                    payload,
                )?,
            },
        };
        writer
            .write_all(frame)
            .context("failed to write UART frame")?;
    }
    writer.flush().context("failed to flush UART responses")?;
    Ok(())
}

fn parse_address(value: &str) -> Result<u16, String> {
    let value = value.trim();
    let parsed = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .map_or_else(|| value.parse(), |hex| u16::from_str_radix(hex, 16))
        .map_err(|error| format!("invalid I²C address {value:?}: {error}"))?;
    if parsed > 0x7f {
        return Err(format!("I²C address must be 7-bit, got 0x{parsed:x}"));
    }
    Ok(parsed)
}
