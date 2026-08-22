//! Named, calibrated stand actions for the Rubik cube mechanism.

use anyhow::{bail, Result};
use clap::{Parser, Subcommand};
use rubik_scan::{
    pca9685::Pca9685,
    stand::{GripConfiguration, GripperOrientation, RailPosition, StandAxis, StandCalibration},
};
use std::path::PathBuf;

#[derive(Parser)]
#[command(about = "Execute bounded calibrated actions of the Rubik stand")]
struct Cli {
    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// Optional TOML file overriding the built-in measured calibration
    #[arg(long)]
    config: Option<PathBuf>,

    /// Required acknowledgement that the stand may move
    #[arg(long)]
    confirm_stand_motion: bool,

    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Retract all four rails to far/open simultaneously
    Open,
    /// Open, set every gripper perpendicular, then close all four rails
    Grip,
    /// Move one gripper to a measured named orientation
    Pose {
        #[arg(long)]
        axis: StandAxis,
        #[arg(long)]
        orientation: GripperOrientation,
    },
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
        bail!("refusing to move the stand; pass --confirm-stand-motion after checking it");
    }

    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };
    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    match cli.command {
        Command::Open => {
            open_to_safe_pose(&mut controller, &calibration)?;
            println!("stand=open grippers=frame-perpendicular outputs=all_off");
        }
        Command::Grip => {
            open_to_safe_pose(&mut controller, &calibration)?;
            controller.pulse_channels_for(
                &rail_channels(&calibration, RailPosition::NearGrip),
                calibration.rail_duration(RailPosition::NearGrip),
            )?;
            println!("stand=grip via=open->perpendicular->grip outputs=all_off");
        }
        Command::Pose { axis, orientation } => {
            if !axis.is_gripper() {
                bail!("{} is a rail; pose accepts only gripper axes", axis.name());
            }
            let pulse_us = calibration
                .gripper_pulse(axis, orientation)
                .ok_or_else(|| {
                    anyhow::anyhow!(
                        "{} has no calibrated {} pose",
                        axis.name(),
                        orientation.name()
                    )
                })?;
            controller.pulse_channel_for(
                axis.channel(),
                pulse_us,
                calibration.gripper_pose_duration(),
            )?;
            println!(
                "stand=pose axis={} orientation={} pulse_us={} outputs=all_off",
                axis.name(),
                orientation.name(),
                pulse_us
            );
        }
    }
    Ok(())
}

fn rail_channels(calibration: &StandCalibration, position: RailPosition) -> Vec<(u8, u16)> {
    StandAxis::RAILS
        .into_iter()
        .map(|axis| (axis.channel(), calibration.rail_pulse(axis, position)))
        .collect()
}

fn open_to_safe_pose(controller: &mut Pca9685, calibration: &StandCalibration) -> Result<()> {
    controller.pulse_channels_for(
        &rail_channels(calibration, RailPosition::FarOpen),
        calibration.rail_duration(RailPosition::FarOpen),
    )?;

    let configuration = GripConfiguration::all_frame_perpendicular();
    configuration.validate()?;
    let mut channels = Vec::with_capacity(4);
    for (axis, orientation) in [
        (StandAxis::LeftGripper, configuration.left),
        (StandAxis::TopGripper, configuration.top),
        (StandAxis::RightGripper, configuration.right),
        (StandAxis::BottomGripper, configuration.bottom),
    ] {
        let pulse_us = calibration
            .gripper_pulse(axis, orientation)
            .ok_or_else(|| {
                anyhow::anyhow!(
                    "{} has no calibrated {} safe pose",
                    axis.name(),
                    orientation.name()
                )
            })?;
        channels.push((axis.channel(), pulse_us));
    }
    controller.pulse_channels_for(&channels, calibration.gripper_pose_duration())
}
