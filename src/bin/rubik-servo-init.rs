//! Set PCA9685's servo-rate PWM clock without enabling any output channel.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::pca9685::Pca9685;
use std::path::PathBuf;

#[derive(Parser)]
#[command(about = "Initialize PCA9685 at a PWM frequency with all outputs forced off")]
struct Cli {
    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// PWM frequency for later servo control; no channel is enabled here
    #[arg(long, default_value_t = 50.0)]
    pwm_hz: f64,

    /// Required acknowledgement: all PCA9685 outputs will be explicitly disabled
    #[arg(long)]
    confirm_safe_output_state: bool,
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
    if !cli.confirm_safe_output_state {
        bail!(
            "refusing to write PCA9685; pass --confirm-safe-output-state after verifying it is safe to disable every output"
        );
    }

    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    let status = controller.initialize_safe_pwm(cli.pwm_hz)?;

    println!(
        "PCA9685 initialized device={} address=0x{:02x}",
        cli.i2c_device.display(),
        cli.address
    );
    println!(
        "PRESCALE=0x{:02x} estimated_pwm_hz={:.3}",
        status.prescale,
        status.pwm_hz()
    );
    println!("outputs=all_off; no servo pulse was emitted");
    Ok(())
}
