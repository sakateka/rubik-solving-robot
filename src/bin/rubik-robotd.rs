//! UART-facing, deadline-driven robot control daemon.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    pca9685::Pca9685,
    robot_daemon::{run_uart_daemon, UartDaemonOptions},
    robot_service::RobotService,
    stand::StandCalibration,
};
use std::path::PathBuf;

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
    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let mut output = Pca9685::open(&cli.i2c_device, cli.address)?;
    let pwm = output.initialize_safe_pwm(cli.pwm_hz)?;
    eprintln!("hardware backend pwm_hz={:.3}", pwm.pwm_hz());
    run_uart_daemon(
        UartDaemonOptions {
            process_name: "rubik-robotd",
            uart_device: &cli.uart_device,
            skip_uart_config: cli.skip_uart_config,
        },
        RobotService::new(output, calibration),
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
