//! Host-side command-line client for rubik-robotd.

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use rubik_link_protocol as link;
use rubik_scan::robot_client::{
    ClientCommand, ClientEvent, ClientMessage, ClientResponse, RobotClient,
};
use std::{
    collections::hash_map::DefaultHasher,
    fs::{File, OpenOptions},
    hash::{Hash, Hasher},
    io::{Read, Write},
    path::{Path, PathBuf},
    process::Command,
    sync::mpsc,
    time::{Duration, Instant, SystemTime},
};

#[derive(Parser)]
#[command(about = "Send robot control protocol commands over a serial link")]
struct Cli {
    /// Serial device. Use /dev/ttyACM0 for the C6 development bridge.
    #[arg(long, default_value = "/dev/ttyACM0")]
    serial_device: PathBuf,

    /// Do not invoke stty; use an already-configured raw serial device.
    #[arg(long)]
    skip_serial_config: bool,

    /// Maximum time to wait for a response and operation completion.
    #[arg(long, default_value = "10s", value_parser = humantime::parse_duration)]
    timeout: Duration,

    /// Required acknowledgement for commands that can move the stand.
    #[arg(long)]
    confirm_stand_motion: bool,

    #[command(subcommand)]
    command: ControlCommand,
}

#[derive(Clone, Copy, Debug, Subcommand)]
enum ControlCommand {
    /// Read the authoritative robot status without moving anything.
    Status,
    /// Move an inspected stand from unknown/aborted state to safe open.
    Recover,
    /// Capture a cube from the known open state.
    Grip,
    /// Immediately disable PWM and cancel the active operation.
    Abort,
}

impl ControlCommand {
    const fn protocol_command(self) -> ClientCommand {
        match self {
            Self::Status => ClientCommand::GetStatus,
            Self::Recover => ClientCommand::RecoverToOpen,
            Self::Grip => ClientCommand::Grip,
            Self::Abort => ClientCommand::Abort,
        }
    }

    const fn may_move_stand(self) -> bool {
        matches!(self, Self::Recover | Self::Grip)
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if cli.command.may_move_stand() && !cli.confirm_stand_motion {
        bail!(
            "{} may move the stand; pass --confirm-stand-motion after checking it",
            command_name(cli.command)
        );
    }
    if !cli.skip_serial_config {
        configure_serial(&cli.serial_device)?;
    }

    let serial = OpenOptions::new()
        .read(true)
        .write(true)
        .open(&cli.serial_device)
        .with_context(|| {
            format!(
                "failed to open serial device {}",
                cli.serial_device.display()
            )
        })?;
    let reader = serial
        .try_clone()
        .context("failed to clone serial device for reader thread")?;
    let receiver = spawn_reader(reader);
    let mut writer = serial;
    let mut client = RobotClient::with_initial_request_id(initial_request_id());
    let request = client.encode_command(cli.command.protocol_command())?;
    let request_id = request.request_id;
    let frame = request.frame.to_vec();
    send_frame(&mut writer, &frame)?;

    wait_for_result(
        &receiver,
        &mut writer,
        &mut client,
        &frame,
        request_id,
        cli.command,
        cli.timeout,
    )
}

fn initial_request_id() -> u32 {
    let mut hasher = DefaultHasher::new();
    SystemTime::now().hash(&mut hasher);
    std::process::id().hash(&mut hasher);
    (hasher.finish() as u32).max(1)
}

fn wait_for_result(
    receiver: &mpsc::Receiver<std::io::Result<Vec<u8>>>,
    writer: &mut File,
    client: &mut RobotClient,
    request_frame: &[u8],
    request_id: u32,
    command: ControlCommand,
    timeout: Duration,
) -> Result<()> {
    const RETRY_INTERVAL: Duration = Duration::from_millis(500);

    let deadline = Instant::now() + timeout;
    let mut next_retry = Instant::now() + RETRY_INTERVAL;
    let mut response_received = false;
    let mut accepted_operation = None;

    loop {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            bail!("timed out after {timeout:?} waiting for request {request_id}");
        }
        let receive_for = if response_received {
            remaining
        } else {
            remaining.min(next_retry.saturating_duration_since(Instant::now()))
        };
        let bytes = match receiver.recv_timeout(receive_for) {
            Ok(result) => result.context("serial reader failed")?,
            Err(mpsc::RecvTimeoutError::Timeout) => {
                if !response_received && Instant::now() < deadline {
                    send_frame(writer, request_frame)?;
                    next_retry = Instant::now() + RETRY_INTERVAL;
                    continue;
                }
                bail!("timed out after {timeout:?} waiting for request {request_id}")
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => bail!("serial reader stopped"),
        };

        for byte in bytes {
            let Some(result) = client.push_byte(byte) else {
                continue;
            };
            let message = match result {
                Ok(message) => message,
                Err(error) => {
                    eprintln!("discarded serial input: {error}");
                    continue;
                }
            };
            print_message(&message);

            match &message {
                ClientMessage::Response(ClientResponse::Status {
                    request_id: response_id,
                    ..
                }) if *response_id == request_id && matches!(command, ControlCommand::Status) => {
                    return Ok(())
                }
                ClientMessage::Response(ClientResponse::Rejected {
                    request_id: response_id,
                    payload,
                }) if *response_id == request_id => {
                    bail!(
                        "command rejected: {:?} (controller={:?})",
                        payload.reason,
                        payload.controller
                    )
                }
                ClientMessage::Response(ClientResponse::Accepted {
                    request_id: response_id,
                    payload,
                }) if *response_id == request_id => {
                    response_received = true;
                    accepted_operation = payload.operation_id;
                    if payload.operation_id.is_none() && !matches!(command, ControlCommand::Abort) {
                        return Ok(());
                    }
                }
                ClientMessage::Event(ClientEvent::OperationCompleted(event))
                    if accepted_operation == Some(event.operation_id) =>
                {
                    return Ok(())
                }
                ClientMessage::Event(ClientEvent::Aborted(_))
                    if matches!(command, ControlCommand::Abort) =>
                {
                    return Ok(())
                }
                ClientMessage::Event(ClientEvent::Fault(event)) => {
                    bail!(
                        "robot fault: {:?} detail={}",
                        event.fault.code,
                        event.fault.detail
                    )
                }
                _ => {}
            }
        }
    }
}

fn send_frame(writer: &mut File, frame: &[u8]) -> Result<()> {
    writer
        .write_all(frame)
        .context("failed to write command frame")?;
    writer.flush().context("failed to flush command frame")?;
    Ok(())
}

fn print_message(message: &ClientMessage) {
    match message {
        ClientMessage::Response(ClientResponse::Status { snapshot, .. }) => {
            print_snapshot(snapshot)
        }
        ClientMessage::Response(ClientResponse::Accepted {
            request_id,
            payload,
        }) => {
            println!("Command accepted");
            println!("  request    {request_id}");
            println!("  operation  {}", optional_number(payload.operation_id));
        }
        ClientMessage::Response(ClientResponse::Rejected {
            request_id,
            payload,
        }) => {
            println!("Command rejected");
            println!("  request     {request_id}");
            println!("  reason      {:?}", payload.reason);
            println!("  controller  {:?}", payload.controller);
        }
        ClientMessage::Event(event) => print_event(event),
    }
}

fn print_snapshot(snapshot: &link::StatusSnapshot) {
    println!("Robot");
    println!("  controller  {:?}", snapshot.controller);
    println!(
        "  operation   {}",
        snapshot
            .active_operation
            .map(operation)
            .unwrap_or_else(|| "—".into())
    );

    println!();
    println!("Stand");
    println!("  pose         {:?}", snapshot.stand.pose.kind);
    println!(
        "  camera face  {}",
        optional_debug(snapshot.stand.pose.camera_face)
    );
    println!(
        "  PWM outputs  {}",
        if snapshot.stand.outputs_enabled {
            "enabled"
        } else {
            "disabled"
        }
    );
    println!();
    println!("  {:<8} {:<28} gripper", "axis", "rail");
    for (name, axis) in [
        ("left", link::Axis::Left),
        ("right", link::Axis::Right),
        ("top", link::Axis::Top),
        ("bottom", link::Axis::Bottom),
    ] {
        println!(
            "  {name:<8} {:<28} {}",
            rail(&snapshot.stand.rails[axis as usize]),
            gripper(&snapshot.stand.grippers[axis as usize])
        );
    }

    println!();
    println!("Cube");
    println!(
        "  session      {}",
        snapshot
            .cube_session
            .map(|session| session.id.to_string())
            .unwrap_or_else(|| "—".into())
    );
    println!("  scan         {:?}", snapshot.scan.state);
    println!(
        "  scan detail  revision {}, current face {}, scanned {}/{} (0b{:06b})",
        optional_number(snapshot.scan.revision),
        optional_debug(snapshot.scan.current_face),
        snapshot.scan.scanned_faces.count_ones(),
        link::FACE_COUNT,
        snapshot.scan.scanned_faces
    );
    println!("  solution     {:?}", snapshot.solution.state);
    println!(
        "  solve detail id {}, moves {}/{}",
        optional_number(snapshot.solution.id),
        snapshot.solution.completed_moves,
        snapshot.solution.move_count
    );
    if let Some(fault) = snapshot.fault {
        println!();
        println!("Fault");
        println!("  code    {:?}", fault.code);
        println!("  detail  {}", fault.detail);
    }
}

fn operation(operation: link::OperationStatus) -> String {
    format!(
        "#{} {:?}, action {}/{}",
        operation.id, operation.kind, operation.current_action, operation.action_count
    )
}

fn print_event(event: &ClientEvent) {
    match event {
        ClientEvent::RobotStateChanged(event) => println!(
            "[state] controller={:?}, operation={}",
            event.controller,
            event
                .active_operation
                .map(operation)
                .unwrap_or_else(|| "—".into())
        ),
        ClientEvent::StandStateChanged(event) => println!(
            "[stand] pose={:?}, PWM={}",
            event.stand.pose.kind,
            if event.stand.outputs_enabled {
                "enabled"
            } else {
                "disabled"
            }
        ),
        ClientEvent::FaceScanned(event) => println!(
            "[scan] operation #{}, face {:?}, scanned {}/{}",
            event.operation_id,
            event.face,
            event.scanned_faces.count_ones(),
            link::FACE_COUNT
        ),
        ClientEvent::PlanChanged(event) => println!(
            "[plan] operation #{}, {} queued actions",
            event.operation_id, event.action_count
        ),
        ClientEvent::ActionStarted(event) => println!(
            "[action] operation #{}, started {}",
            event.operation_id,
            action(&event.action)
        ),
        ClientEvent::ActionCompleted(event) => println!(
            "[action] operation #{}, completed {}",
            event.operation_id,
            action(&event.action)
        ),
        ClientEvent::OperationCompleted(event) => {
            println!("[done] operation #{} {:?}", event.operation_id, event.kind)
        }
        ClientEvent::Aborted(event) => {
            println!("[abort] operation {}", optional_number(event.operation_id))
        }
        ClientEvent::CubeSessionChanged(event) => println!(
            "[cube] session {}",
            event
                .session
                .map(|session| session.id.to_string())
                .unwrap_or_else(|| "closed".into())
        ),
        ClientEvent::OperationFailed(event) => println!(
            "[failed] operation #{} {:?}",
            event.operation_id, event.kind
        ),
        ClientEvent::Fault(event) => println!(
            "[fault] operation {}, {:?}, detail {}",
            optional_number(event.operation_id),
            event.fault.code,
            event.fault.detail
        ),
    }
}

fn rail(status: &link::RailStatus) -> String {
    axis_state(status.motion, status.current, status.target)
}

fn gripper(status: &link::GripperStatus) -> String {
    axis_state(status.motion, status.current, status.target)
}

fn axis_state<T: std::fmt::Debug>(
    motion: link::AxisMotion,
    current: Option<T>,
    target: Option<T>,
) -> String {
    match motion {
        link::AxisMotion::Unknown => "unknown".into(),
        link::AxisMotion::Stable => format!("stable: {}", optional_debug(current)),
        link::AxisMotion::Moving => format!(
            "moving: {} → {}",
            optional_debug(current),
            optional_debug(target)
        ),
    }
}

fn action(action: &link::MechanicalAction) -> String {
    let subject = if let Some(axis) = action.axis {
        format!(" axis={axis:?}")
    } else if let Some(face) = action.face {
        format!(" face={face:?}")
    } else {
        String::new()
    };
    format!("#{}, {:?}{subject}", action.id, action.kind)
}

fn optional_debug<T: std::fmt::Debug>(value: Option<T>) -> String {
    value
        .map(|value| format!("{value:?}"))
        .unwrap_or_else(|| "—".into())
}

fn optional_number(value: Option<u32>) -> String {
    value
        .map(|value| value.to_string())
        .unwrap_or_else(|| "—".into())
}

const fn command_name(command: ControlCommand) -> &'static str {
    match command {
        ControlCommand::Status => "status",
        ControlCommand::Recover => "recover",
        ControlCommand::Grip => "grip",
        ControlCommand::Abort => "abort",
    }
}

fn configure_serial(path: &Path) -> Result<()> {
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

fn spawn_reader(mut reader: File) -> mpsc::Receiver<std::io::Result<Vec<u8>>> {
    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || loop {
        let mut buffer = [0u8; 128];
        match reader.read(&mut buffer) {
            Ok(0) => {
                let _ = sender.send(Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "serial device returned EOF",
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
