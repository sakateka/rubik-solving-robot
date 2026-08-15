//! Bounded, single-axis servo pulse tool for mechanical calibration.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    pca9685::{Pca9685, NOMINAL_MAX_PULSE_US, NOMINAL_MIN_PULSE_US},
    stand::StandAxis,
};
use std::{path::PathBuf, time::Duration};

#[derive(Parser)]
#[command(about = "Move exactly one Rubik stand axis for a bounded calibration interval")]
struct Cli {
    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// Axis: right-gripper, bottom-gripper, top-gripper, left-gripper, top-rail, left-rail, bottom-rail, right-rail
    #[arg(long)]
    axis: StandAxis,

    /// Servo pulse width in microseconds
    #[arg(long)]
    pulse_us: u16,

    /// Required for exploratory pulses outside DS3218's documented 500..=2500 us range
    #[arg(long)]
    allow_out_of_spec_pulse: bool,

    /// Time to hold the pulse; limited to 100..=5000 ms
    #[arg(long, default_value_t = 500, value_parser = parse_hold_ms)]
    hold_ms: u64,

    /// Required acknowledgement that the selected axis may move
    #[arg(long)]
    confirm_axis_motion: bool,
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

fn parse_hold_ms(value: &str) -> Result<u64, String> {
    let value: u64 = value
        .parse()
        .map_err(|error| format!("invalid hold duration {value:?}: {error}"))?;
    if !(100..=5_000).contains(&value) {
        return Err(format!("hold duration must be 100..=5000 ms, got {value}"));
    }
    Ok(value)
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if !cli.confirm_axis_motion {
        bail!("refusing to move an axis; pass --confirm-axis-motion after checking the stand");
    }
    if !(NOMINAL_MIN_PULSE_US..=NOMINAL_MAX_PULSE_US).contains(&cli.pulse_us)
        && !cli.allow_out_of_spec_pulse
    {
        bail!(
            "pulse {} us is outside the documented {}..={} us range; pass --allow-out-of-spec-pulse for a short exploratory test",
            cli.pulse_us,
            NOMINAL_MIN_PULSE_US,
            NOMINAL_MAX_PULSE_US,
        );
    }

    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    println!(
        "calibration axis={} channel={} pulse_us={} hold_ms={}",
        cli.axis.name(),
        cli.axis.channel(),
        cli.pulse_us,
        cli.hold_ms,
    );
    controller.pulse_channel_for(
        cli.axis.channel(),
        cli.pulse_us,
        Duration::from_millis(cli.hold_ms),
    )?;
    println!("outputs=all_off");
    Ok(())
}
