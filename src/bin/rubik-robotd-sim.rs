//! UART robot daemon with an in-memory PWM backend.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    pca9685::PwmOutput,
    robot_daemon::{run_uart_daemon, UartDaemonOptions},
    robot_service::RobotService,
    stand::StandCalibration,
};
use std::path::PathBuf;

const PWM_CHANNEL_COUNT: usize = 16;

#[derive(Parser)]
#[command(about = "Run the robot control service without I2C or servo output")]
struct Cli {
    /// Duo UART connected to ESP32-C6
    #[arg(long, default_value = "/dev/ttyS1")]
    uart_device: PathBuf,

    /// Do not invoke stty; use an already-configured raw UART
    #[arg(long)]
    skip_uart_config: bool,

    /// Optional TOML file overriding built-in stand calibration and timings
    #[arg(long)]
    config: Option<PathBuf>,

    /// Print simulated PWM writes to stderr
    #[arg(long)]
    trace_pwm: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let output = SimulatedPwmOutput::new(cli.trace_pwm);

    eprintln!("SIMULATION: no I2C device will be opened and no PWM will be emitted");
    run_uart_daemon(
        UartDaemonOptions {
            process_name: "rubik-robotd-sim",
            uart_device: &cli.uart_device,
            skip_uart_config: cli.skip_uart_config,
        },
        RobotService::new(output, calibration),
    )
}

struct SimulatedPwmOutput {
    channels: [Option<u16>; PWM_CHANNEL_COUNT],
    trace: bool,
}

impl SimulatedPwmOutput {
    const fn new(trace: bool) -> Self {
        Self {
            channels: [None; PWM_CHANNEL_COUNT],
            trace,
        }
    }

    fn validate_channel(channel: u8) -> Result<usize> {
        let index = usize::from(channel);
        if index >= PWM_CHANNEL_COUNT {
            bail!("simulated PWM channel must be 0..15, got {channel}");
        }
        Ok(index)
    }
}

impl PwmOutput for SimulatedPwmOutput {
    fn set_channels(&mut self, channels: &[(u8, u16)]) -> Result<()> {
        for &(channel, pulse_us) in channels {
            self.channels[Self::validate_channel(channel)?] = Some(pulse_us);
        }
        if self.trace {
            eprintln!("[sim:pwm] set {channels:?}");
        }
        Ok(())
    }

    fn disable_channels(&mut self, channels: &[u8]) -> Result<()> {
        for &channel in channels {
            self.channels[Self::validate_channel(channel)?] = None;
        }
        if self.trace {
            eprintln!("[sim:pwm] disable {channels:?}");
        }
        Ok(())
    }

    fn all_off(&mut self) -> Result<()> {
        self.channels.fill(None);
        if self.trace {
            eprintln!("[sim:pwm] all off");
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn simulated_output_tracks_set_disable_and_all_off() {
        let mut output = SimulatedPwmOutput::new(false);

        output.set_channels(&[(0, 1_500), (7, 2_500)]).unwrap();
        assert_eq!(output.channels[0], Some(1_500));
        assert_eq!(output.channels[7], Some(2_500));

        output.disable_channels(&[0]).unwrap();
        assert_eq!(output.channels[0], None);
        assert_eq!(output.channels[7], Some(2_500));

        output.all_off().unwrap();
        assert!(output.channels.iter().all(Option::is_none));
    }
}
