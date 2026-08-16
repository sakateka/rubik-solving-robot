//! Bounded, single-axis servo pulse tool for mechanical calibration.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    pca9685::{Pca9685, NOMINAL_MAX_PULSE_US, NOMINAL_MIN_PULSE_US},
    stand::StandAxis,
};
use std::{
    path::PathBuf,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    time::{Duration, Instant},
};

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

    /// Time to hold the pulse, for example 500ms, 5s, 1m, or 2h
    #[arg(long, default_value = "500ms", value_parser = humantime::parse_duration)]
    hold: Duration,

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

    let interrupted = Arc::new(AtomicBool::new(false));
    let signal_interrupted = Arc::clone(&interrupted);
    ctrlc::set_handler(move || signal_interrupted.store(true, Ordering::SeqCst))?;

    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    println!(
        "calibration axis={} channel={} pulse_us={} hold={:?}",
        cli.axis.name(),
        cli.axis.channel(),
        cli.pulse_us,
        cli.hold,
    );
    let pulse_result = controller.begin_pulse_channels(&[(cli.axis.channel(), cli.pulse_us)]);
    let was_interrupted = if pulse_result.is_ok() {
        wait_for_hold(cli.hold, &interrupted)
    } else {
        interrupted.load(Ordering::SeqCst)
    };
    let off_result = controller.all_off();

    pulse_result?;
    off_result?;
    if was_interrupted {
        bail!("Ctrl-C received; outputs=all_off");
    }
    println!("outputs=all_off");
    Ok(())
}

/// Sleeps in short intervals so Ctrl-C can stop PWM promptly during a hold.
fn wait_for_hold(duration: Duration, interrupted: &AtomicBool) -> bool {
    let mut remaining = duration;
    while !remaining.is_zero() {
        if interrupted.load(Ordering::SeqCst) {
            return true;
        }
        let interval = remaining.min(Duration::from_millis(10));
        let start = Instant::now();
        std::thread::sleep(interval);
        remaining = remaining.saturating_sub(start.elapsed());
    }
    false
}

#[cfg(test)]
mod tests {
    use super::Cli;
    use clap::Parser;
    use std::time::Duration;

    #[test]
    fn cli_uses_humantime_duration_syntax_without_an_upper_bound() {
        let cli = Cli::try_parse_from([
            "rubik-servo-calibrate",
            "--axis",
            "right-gripper",
            "--hold",
            "1h 30m",
            "--confirm-axis-motion",
            "--pulse-us",
            "1500",
        ])
        .unwrap();

        assert_eq!(cli.hold, Duration::from_secs(5_400));
    }
}
