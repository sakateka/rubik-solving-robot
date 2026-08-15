//! Read-only PCA9685 verification for the Rubik stand.

use anyhow::Result;
use clap::Parser;
use rubik_scan::pca9685::Pca9685;
use std::path::PathBuf;

#[derive(Parser)]
#[command(about = "Read PCA9685 status registers without moving any servo")]
struct Cli {
    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,
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
    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    let status = controller.status()?;

    println!(
        "PCA9685 device={} address=0x{:02x}",
        cli.i2c_device.display(),
        cli.address
    );
    println!(
        "MODE1=0x{:02x} sleep={} auto_increment={} all_call={}",
        status.mode1,
        status.sleeping(),
        status.auto_increment(),
        status.all_call_enabled()
    );
    println!(
        "MODE2=0x{:02x} output_driver={}",
        status.mode2,
        if status.push_pull_output() {
            "push-pull"
        } else {
            "open-drain"
        }
    );
    println!(
        "PRESCALE=0x{:02x} estimated_pwm_hz={:.2}",
        status.prescale,
        status.pwm_hz()
    );
    println!("writes=0 (read-only probe)");
    Ok(())
}
