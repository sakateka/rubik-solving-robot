//! Fixed hardware layout of the eight-servo Rubik stand.
//!
//! Channel assignment is physical wiring. Measured servo poses are loaded from
//! a TOML calibration file, so mechanical adjustment never requires a rebuild.

use anyhow::{Context, Result};
use serde::Deserialize;
use std::{path::Path, time::Duration};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StandAxis {
    RightGripper,
    BottomGripper,
    TopGripper,
    LeftGripper,
    TopRail,
    LeftRail,
    BottomRail,
    RightRail,
}

/// Chosen project operating range for DSSERVO DS3218 gripper axes.
///
/// This is intentionally narrower than the calibration tool's exploratory
/// range and wider than the servo's nominal 500..=2500 us specification.
pub const GRIPPER_WORKING_MIN_PULSE_US: u16 = 400;
pub const GRIPPER_WORKING_MAX_PULSE_US: u16 = 2700;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GripperOrientation {
    FrameParallel,
    FramePerpendicular,
    FrameParallelReversed,
}

impl std::str::FromStr for GripperOrientation {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "frame-parallel" | "frame_parallel" => Ok(Self::FrameParallel),
            "frame-perpendicular" | "frame_perpendicular" => Ok(Self::FramePerpendicular),
            "frame-parallel-reversed" | "frame_parallel_reversed" => {
                Ok(Self::FrameParallelReversed)
            }
            _ => Err(format!("unknown gripper orientation {value:?}")),
        }
    }
}

impl GripperOrientation {
    pub const fn name(self) -> &'static str {
        match self {
            Self::FrameParallel => "frame-parallel",
            Self::FramePerpendicular => "frame-perpendicular",
            Self::FrameParallelReversed => "frame-parallel-reversed",
        }
    }

    pub const fn is_frame_parallel(self) -> bool {
        matches!(self, Self::FrameParallel | Self::FrameParallelReversed)
    }
}

/// Declared physical orientation of every gripper before the rails close.
///
/// The controller has no external position feedback, so this is a planned
/// state, not a sensor reading. It is nevertheless mandatory: closing rails
/// with adjacent parallel grippers would cause a mechanical collision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GripConfiguration {
    pub left: GripperOrientation,
    pub top: GripperOrientation,
    pub right: GripperOrientation,
    pub bottom: GripperOrientation,
}

impl GripConfiguration {
    /// Collision-free orientation used after the rails are fully open.
    pub const fn all_frame_perpendicular() -> Self {
        Self {
            left: GripperOrientation::FramePerpendicular,
            top: GripperOrientation::FramePerpendicular,
            right: GripperOrientation::FramePerpendicular,
            bottom: GripperOrientation::FramePerpendicular,
        }
    }

    pub fn validate(self) -> Result<()> {
        for (first_name, first, second_name, second) in [
            ("left", self.left, "top", self.top),
            ("top", self.top, "right", self.right),
            ("right", self.right, "bottom", self.bottom),
            ("bottom", self.bottom, "left", self.left),
        ] {
            if first.is_frame_parallel() && second.is_frame_parallel() {
                anyhow::bail!(
                    "cannot grip: adjacent {first_name} and {second_name} grippers are both frame-parallel"
                );
            }
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StandCalibration {
    pub rails: RailCalibration,
    pub grippers: GrippersCalibration,
    pub timing: TimingCalibration,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RailCalibration {
    pub far_open_us: u16,
    pub near_grip_us: u16,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GrippersCalibration {
    pub right: GripperCalibration,
    pub bottom: GripperCalibration,
    pub top: GripperCalibration,
    pub left: GripperCalibration,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct GripperCalibration {
    pub frame_parallel_us: u16,
    pub frame_perpendicular_us: u16,
    pub frame_parallel_reversed_us: u16,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimingCalibration {
    /// Time for every rail to reach the far/open position before grippers move.
    pub rails_open_ms: u64,
    /// Time for every rail to reach the near/grip position.
    pub rails_grip_ms: u64,
    /// Time for a gripper to reach one calibrated orientation.
    pub gripper_pose_ms: u64,
}

impl Default for StandCalibration {
    fn default() -> Self {
        Self {
            rails: RailCalibration {
                far_open_us: 2500,
                near_grip_us: 1200,
            },
            grippers: GrippersCalibration {
                left: GripperCalibration {
                    frame_parallel_us: 400,
                    frame_perpendicular_us: 1450,
                    frame_parallel_reversed_us: 2450,
                },
                right: GripperCalibration {
                    frame_parallel_us: 450,
                    frame_perpendicular_us: 1500,
                    frame_parallel_reversed_us: 2500,
                },
                top: GripperCalibration {
                    frame_parallel_us: 400,
                    frame_perpendicular_us: 1450,
                    frame_parallel_reversed_us: 2450,
                },
                bottom: GripperCalibration {
                    frame_parallel_us: 400,
                    frame_perpendicular_us: 1450,
                    frame_parallel_reversed_us: 2500,
                },
            },
            timing: TimingCalibration {
                rails_open_ms: 1_200,
                rails_grip_ms: 1_200,
                gripper_pose_ms: 1_000,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RailPosition {
    FarOpen,
    NearGrip,
}

impl StandCalibration {
    pub fn load(path: &Path) -> Result<Self> {
        let contents = std::fs::read_to_string(path)
            .with_context(|| format!("failed to read stand calibration {}", path.display()))?;
        let calibration: Self = toml::from_str(&contents)
            .with_context(|| format!("invalid stand calibration {}", path.display()))?;
        calibration.validate()?;
        Ok(calibration)
    }

    pub fn rail_pulse(&self, position: RailPosition) -> u16 {
        match position {
            RailPosition::FarOpen => self.rails.far_open_us,
            RailPosition::NearGrip => self.rails.near_grip_us,
        }
    }

    pub fn rail_duration(&self, position: RailPosition) -> Duration {
        let milliseconds = match position {
            RailPosition::FarOpen => self.timing.rails_open_ms,
            RailPosition::NearGrip => self.timing.rails_grip_ms,
        };
        Duration::from_millis(milliseconds)
    }

    pub fn gripper_pose_duration(&self) -> Duration {
        Duration::from_millis(self.timing.gripper_pose_ms)
    }

    pub fn gripper_pulse(&self, axis: StandAxis, orientation: GripperOrientation) -> Option<u16> {
        let gripper = match axis {
            StandAxis::RightGripper => &self.grippers.right,
            StandAxis::BottomGripper => &self.grippers.bottom,
            StandAxis::TopGripper => &self.grippers.top,
            StandAxis::LeftGripper => &self.grippers.left,
            _ => return None,
        };
        Some(match orientation {
            GripperOrientation::FrameParallel => gripper.frame_parallel_us,
            GripperOrientation::FramePerpendicular => gripper.frame_perpendicular_us,
            GripperOrientation::FrameParallelReversed => gripper.frame_parallel_reversed_us,
        })
    }

    fn validate(&self) -> Result<()> {
        validate_pulse("rails.far_open_us", self.rails.far_open_us, 300, 2800)?;
        validate_pulse("rails.near_grip_us", self.rails.near_grip_us, 300, 2800)?;
        validate_duration("timing.rails_open_ms", self.timing.rails_open_ms)?;
        validate_duration("timing.rails_grip_ms", self.timing.rails_grip_ms)?;
        validate_duration("timing.gripper_pose_ms", self.timing.gripper_pose_ms)?;
        for (name, gripper) in [
            ("grippers.right", &self.grippers.right),
            ("grippers.bottom", &self.grippers.bottom),
            ("grippers.top", &self.grippers.top),
            ("grippers.left", &self.grippers.left),
        ] {
            for (pose, pulse) in [
                ("frame_parallel_us", gripper.frame_parallel_us),
                ("frame_perpendicular_us", gripper.frame_perpendicular_us),
                (
                    "frame_parallel_reversed_us",
                    gripper.frame_parallel_reversed_us,
                ),
            ] {
                validate_pulse(
                    &format!("{name}.{pose}"),
                    pulse,
                    GRIPPER_WORKING_MIN_PULSE_US,
                    GRIPPER_WORKING_MAX_PULSE_US,
                )?;
            }
        }
        Ok(())
    }
}

fn validate_duration(name: &str, milliseconds: u64) -> Result<()> {
    if !(100..=5_000).contains(&milliseconds) {
        anyhow::bail!("{name} must be 100..=5000 ms, got {milliseconds}");
    }
    Ok(())
}

fn validate_pulse(name: &str, pulse: u16, min: u16, max: u16) -> Result<()> {
    if !(min..=max).contains(&pulse) {
        anyhow::bail!("{name} must be {min}..={max} us, got {pulse}");
    }
    Ok(())
}

impl std::str::FromStr for StandAxis {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "right-gripper" => Ok(Self::RightGripper),
            "bottom-gripper" => Ok(Self::BottomGripper),
            "top-gripper" => Ok(Self::TopGripper),
            "left-gripper" => Ok(Self::LeftGripper),
            "top-rail" => Ok(Self::TopRail),
            "left-rail" => Ok(Self::LeftRail),
            "bottom-rail" => Ok(Self::BottomRail),
            "right-rail" => Ok(Self::RightRail),
            _ => Err(format!("unknown stand axis {value:?}")),
        }
    }
}

impl StandAxis {
    pub const GRIPPERS: [Self; 4] = [
        Self::RightGripper,
        Self::BottomGripper,
        Self::TopGripper,
        Self::LeftGripper,
    ];

    pub const RAILS: [Self; 4] = [
        Self::TopRail,
        Self::LeftRail,
        Self::BottomRail,
        Self::RightRail,
    ];

    pub const ALL: [Self; 8] = [
        Self::RightGripper,
        Self::BottomGripper,
        Self::TopGripper,
        Self::LeftGripper,
        Self::TopRail,
        Self::LeftRail,
        Self::BottomRail,
        Self::RightRail,
    ];

    pub const fn channel(self) -> u8 {
        match self {
            Self::RightGripper => 0,
            Self::BottomGripper => 1,
            Self::TopGripper => 2,
            Self::LeftGripper => 3,
            Self::TopRail => 4,
            Self::LeftRail => 5,
            Self::BottomRail => 6,
            Self::RightRail => 7,
        }
    }

    pub const fn name(self) -> &'static str {
        match self {
            Self::RightGripper => "right-gripper",
            Self::BottomGripper => "bottom-gripper",
            Self::TopGripper => "top-gripper",
            Self::LeftGripper => "left-gripper",
            Self::TopRail => "top-rail",
            Self::LeftRail => "left-rail",
            Self::BottomRail => "bottom-rail",
            Self::RightRail => "right-rail",
        }
    }

    pub const fn is_gripper(self) -> bool {
        self.channel() < 4
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn physical_wiring_is_contiguous_and_unique() {
        let channels: Vec<_> = StandAxis::ALL.iter().map(|axis| axis.channel()).collect();
        assert_eq!(channels, (0..8).collect::<Vec<_>>());
        assert!(StandAxis::RightGripper.is_gripper());
        assert!(!StandAxis::TopRail.is_gripper());
    }

    #[test]
    fn parses_stable_axis_names() {
        assert_eq!("left-rail".parse(), Ok(StandAxis::LeftRail));
        assert!("rail-left".parse::<StandAxis>().is_err());
    }

    #[test]
    fn loads_and_validates_calibration() {
        let calibration: StandCalibration =
            toml::from_str(include_str!("../config/stand.toml")).unwrap();
        calibration.validate().unwrap();
        assert_eq!(
            calibration.gripper_pulse(StandAxis::LeftGripper, GripperOrientation::FrameParallel),
            Some(400)
        );
        assert_eq!(
            calibration.gripper_pulse(
                StandAxis::BottomGripper,
                GripperOrientation::FrameParallelReversed
            ),
            Some(2500)
        );
    }

    #[test]
    fn default_calibration_matches_the_current_stand() {
        let calibration = StandCalibration::default();
        calibration.validate().unwrap();
        assert_eq!(calibration.rail_pulse(RailPosition::FarOpen), 2500);
        assert_eq!(
            calibration.gripper_pulse(StandAxis::BottomGripper, GripperOrientation::FrameParallel),
            Some(400)
        );
        assert_eq!(
            calibration.rail_duration(RailPosition::FarOpen),
            Duration::from_millis(1_200)
        );
    }

    #[test]
    fn grip_configuration_rejects_adjacent_parallel_grippers() {
        let safe = GripConfiguration {
            left: GripperOrientation::FrameParallel,
            top: GripperOrientation::FramePerpendicular,
            right: GripperOrientation::FrameParallel,
            bottom: GripperOrientation::FramePerpendicular,
        };
        safe.validate().unwrap();

        let unsafe_configuration = GripConfiguration {
            top: GripperOrientation::FrameParallel,
            ..safe
        };
        assert!(unsafe_configuration.validate().is_err());
    }

    #[test]
    fn safe_open_configuration_uses_perpendicular_pose_for_every_gripper() {
        let calibration = StandCalibration::default();
        let safe = GripConfiguration::all_frame_perpendicular();
        safe.validate().unwrap();

        assert_eq!(safe.left, GripperOrientation::FramePerpendicular);
        assert_eq!(safe.top, GripperOrientation::FramePerpendicular);
        assert_eq!(safe.right, GripperOrientation::FramePerpendicular);
        assert_eq!(safe.bottom, GripperOrientation::FramePerpendicular);
        assert_eq!(
            calibration.gripper_pulse(StandAxis::LeftGripper, safe.left),
            Some(1450)
        );
        assert_eq!(
            calibration.gripper_pulse(StandAxis::TopGripper, safe.top),
            Some(1450)
        );
        assert_eq!(
            calibration.gripper_pulse(StandAxis::RightGripper, safe.right),
            Some(1500)
        );
        assert_eq!(
            calibration.gripper_pulse(StandAxis::BottomGripper, safe.bottom),
            Some(1450)
        );
    }
}
