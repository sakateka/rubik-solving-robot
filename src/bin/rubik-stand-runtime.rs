//! Long-running persistent-PWM diagnostic shell for the Rubik stand.

use anyhow::{bail, Context, Result};
use clap::Parser;
use rubik_scan::{
    pca9685::Pca9685,
    stand::StandCalibration,
    stand_runtime::{CommandedStandState, ScanFace, StandRuntime},
};
use std::{
    io::{self, BufRead, Write},
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc,
    },
    time::Duration,
};

#[derive(Parser)]
#[command(about = "Run persistent-PWM diagnostics for the Rubik stand")]
struct Cli {
    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// Optional TOML file overriding built-in stand calibration
    #[arg(long)]
    config: Option<PathBuf>,

    /// Servo PWM frequency used when this runtime starts
    #[arg(long, default_value_t = 50.0)]
    pwm_hz: f64,

    /// Required acknowledgement that entered commands may move the stand
    #[arg(long)]
    confirm_stand_motion: bool,
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

fn main() -> Result<()> {
    let cli = Cli::parse();
    if !cli.confirm_stand_motion {
        bail!("refusing to start motion runtime; pass --confirm-stand-motion after checking the stand");
    }

    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    let status = controller.initialize_safe_pwm(cli.pwm_hz)?;
    let mut runtime = StandRuntime::new(controller, calibration);
    runtime.reset()?;

    println!(
        "runtime ready pwm_hz={:.3} state={}; commands: state, safe-open, grip, scan-pose <face>, scan-next <face>, off, reset, quit",
        status.pwm_hz(),
        state_name(runtime.state())
    );
    command_loop(&mut runtime)
}

fn command_loop(runtime: &mut StandRuntime<Pca9685>) -> Result<()> {
    let mut stdout = io::stdout();
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || signal_running.store(false, Ordering::SeqCst))
        .context("failed to install Ctrl-C shutdown handler")?;

    let (sender, receiver) = mpsc::channel();
    std::thread::spawn(move || {
        for line in io::stdin().lock().lines() {
            if sender.send(line).is_err() {
                break;
            }
        }
    });

    write!(stdout, "> ").context("failed to write runtime prompt")?;
    stdout.flush().context("failed to flush runtime prompt")?;

    while running.load(Ordering::SeqCst) {
        let line = match receiver.recv_timeout(Duration::from_millis(100)) {
            Ok(line) => line.context("failed to read runtime command from stdin")?,
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };
        let command = line.trim();
        match command {
            "" => {}
            "help" => println!(
                "commands: state, safe-open, grip, scan-pose <face>, scan-next <face>, off, reset, quit; faces: front, left, right, up, down, back"
            ),
            "state" => println!("state={}", state_name(runtime.state())),
            "safe-open" => {
                let result = runtime.safe_open();
                report_motion(result, runtime.state());
            }
            "grip" => {
                let result = runtime.grip();
                report_motion(result, runtime.state());
            }
            _ if command.starts_with("scan-pose") => match parse_scan_pose(command) {
                Ok(face) => {
                    let result = runtime.scan_pose(face);
                    report_motion(result, runtime.state());
                }
                Err(error) => eprintln!("{error}"),
            },
            _ if command.starts_with("scan-next") => match parse_scan_pose(command) {
                Ok(face) => {
                    let result = runtime.scan_next(face);
                    report_motion(result, runtime.state());
                }
                Err(error) => eprintln!("{error}"),
            },
            "off" => {
                let result = runtime.off();
                report_motion(result, runtime.state());
            }
            "reset" => {
                let result = runtime.reset();
                report_motion(result, runtime.state());
            }
            "quit" | "exit" => {
                runtime.off()?;
                println!("outputs=all_off; runtime stopped");
                return Ok(());
            }
            _ => eprintln!("unknown command {command:?}; type help"),
        }
        write!(stdout, "> ").context("failed to write runtime prompt")?;
        stdout.flush().context("failed to flush runtime prompt")?;
    }

    runtime.off()?;
    if running.load(Ordering::SeqCst) {
        println!("stdin closed; outputs=all_off; runtime stopped");
    } else {
        println!("Ctrl-C received; outputs=all_off; runtime stopped");
    }
    Ok(())
}

fn report_motion(result: Result<()>, state: CommandedStandState) {
    match result {
        Ok(()) => println!("state={}", state_name(state)),
        Err(error) => eprintln!("command failed: {error:#}"),
    }
}

fn state_name(state: CommandedStandState) -> &'static str {
    match state {
        CommandedStandState::Unknown => "unknown",
        CommandedStandState::OutputsOff => "outputs-off",
        CommandedStandState::SafeOpen => "safe-open",
        CommandedStandState::Gripped => "gripped",
        CommandedStandState::ScanHold(face) => face.name(),
        CommandedStandState::Faulted => "faulted",
    }
}

fn parse_scan_pose(command: &str) -> Result<ScanFace, String> {
    let mut parts = command.split_whitespace();
    let _ = parts.next();
    let Some(face) = parts.next() else {
        return Err("usage: scan-pose|scan-next <front|left|right|up|down|back>".to_owned());
    };
    if parts.next().is_some() {
        return Err("usage: scan-pose|scan-next <front|left|right|up|down|back>".to_owned());
    }
    match face {
        "front" | "f" => Ok(ScanFace::Front),
        "left" | "l" => Ok(ScanFace::Left),
        "right" | "r" => Ok(ScanFace::Right),
        "up" | "u" => Ok(ScanFace::Up),
        "down" | "d" => Ok(ScanFace::Down),
        "back" | "b" => Ok(ScanFace::Back),
        _ => Err(format!(
            "unknown scan face {face:?}; expected front, left, right, up, down, or back"
        )),
    }
}
