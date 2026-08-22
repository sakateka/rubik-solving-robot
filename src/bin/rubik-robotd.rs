//! UART-facing, deadline-driven robot control daemon.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    operator_button::{ButtonInput, OperatorButton, SysfsActiveLowButton, DUO256M_GP21_GPIO},
    pca9685::Pca9685,
    robot_daemon::{run_uart_daemon, UartDaemonOptions},
    robot_service::RobotService,
    stand::StandCalibration,
    vision_scanner::VisionScanner,
};
use std::path::PathBuf;
use std::time::{Duration, Instant};

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

    /// Path to the vendor GC2083 sensor configuration
    #[arg(long, default_value = "/mnt/data/sensor_cfg.ini")]
    sensor_config: PathBuf,

    /// VPSS frames discarded once while automatic exposure settles
    #[arg(long, default_value_t = 10)]
    warmup_frames: u32,

    /// VPSS frames discarded before every scanned face
    #[arg(long, default_value_t = 0)]
    scan_discard_frames: u32,

    /// Production CV181X TPU model
    #[arg(long, default_value = "/mnt/storage/cube_yolov8n_320_bf16.cvimodel")]
    cvimodel: PathBuf,

    /// Root directory for per-scan training artifacts
    #[arg(long, default_value = "/mnt/storage/rubik-scan-records")]
    record_dir: PathBuf,

    /// Detection confidence threshold
    #[arg(long, default_value_t = 0.5)]
    conf: f32,

    /// Class-agnostic NMS IoU threshold
    #[arg(long, default_value_t = 0.5)]
    iou: f32,

    /// Required acknowledgement that remote commands may move the stand
    #[arg(long)]
    confirm_stand_motion: bool,

    /// Linux GPIO number for the active-low operator button (Duo256M GP21)
    #[arg(long, default_value_t = DUO256M_GP21_GPIO)]
    button_gpio: u32,

    /// Disable the physical operator button (diagnostics only)
    #[arg(long)]
    no_button: bool,

    /// Stable active-low time required for one button press
    #[arg(
        long,
        default_value_t = 50,
        value_parser = clap::value_parser!(u64).range(10..=1000)
    )]
    button_debounce_ms: u64,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if !cli.confirm_stand_motion {
        bail!(
            "refusing to start robot daemon; pass --confirm-stand-motion after checking the stand"
        );
    }
    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let operator_button = if cli.no_button {
        eprintln!("operator button disabled");
        None
    } else {
        let input = if cli.button_gpio == DUO256M_GP21_GPIO {
            SysfsActiveLowButton::open_duo256m_gp21()?
        } else {
            SysfsActiveLowButton::open(cli.button_gpio)?
        };
        eprintln!(
            "operator button gpio={} active_low=true debounce_ms={}",
            input.gpio(),
            cli.button_debounce_ms
        );
        let input: Box<dyn ButtonInput> = Box::new(input);
        Some(OperatorButton::new(
            input,
            Duration::from_millis(cli.button_debounce_ms),
            Instant::now(),
        ))
    };
    let mut output = Pca9685::open(&cli.i2c_device, cli.address)?;
    let pwm = output.initialize_safe_pwm(cli.pwm_hz)?;
    let scanner = VisionScanner::open(
        &cli.sensor_config,
        cli.warmup_frames,
        cli.scan_discard_frames,
        &cli.cvimodel,
        cli.conf,
        cli.iou,
        cli.record_dir,
    )?;
    eprintln!("hardware backend pwm_hz={:.3}", pwm.pwm_hz());
    run_uart_daemon(
        UartDaemonOptions {
            process_name: "rubik-robotd",
            uart_device: Some(&cli.uart_device),
            skip_uart_config: cli.skip_uart_config,
            hub: None,
            operator_button,
        },
        RobotService::with_scanner(output, calibration, scanner),
    )
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
