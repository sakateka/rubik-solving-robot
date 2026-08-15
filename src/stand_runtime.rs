//! Stateful, persistent-PWM control for the Rubik stand.
//!
//! This module tracks the last successfully commanded state; it cannot observe
//! physical servo position. A failed I²C operation therefore faults the runtime
//! and requires an explicit reset before another motion is attempted.

use crate::{
    pca9685::PwmOutput,
    stand::{GripConfiguration, RailPosition, StandAxis, StandCalibration},
};
use anyhow::{Context, Result};
use std::time::Duration;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandedStandState {
    /// No safe assumption can be made about PCA output or physical positions.
    Unknown,
    /// PWM was explicitly disabled on all channels.
    OutputsOff,
    /// Rails are open and every gripper is perpendicular to its frame side.
    SafeOpen,
    /// Rails are closed around the cube with the safe gripper configuration.
    Gripped,
    /// An output operation failed; no motion is allowed until `reset` succeeds.
    Faulted,
}

pub trait Delay {
    fn sleep(&mut self, duration: Duration);
}

#[derive(Debug, Default)]
pub struct ThreadDelay;

impl Delay for ThreadDelay {
    fn sleep(&mut self, duration: Duration) {
        std::thread::sleep(duration);
    }
}

/// Owns one PWM output and the commanded state of the mechanical stand.
pub struct StandRuntime<D, T = ThreadDelay> {
    output: D,
    calibration: StandCalibration,
    delay: T,
    state: CommandedStandState,
}

impl<D> StandRuntime<D, ThreadDelay>
where
    D: PwmOutput,
{
    pub fn new(output: D, calibration: StandCalibration) -> Self {
        Self::with_delay(output, calibration, ThreadDelay)
    }
}

impl<D, T> StandRuntime<D, T>
where
    D: PwmOutput,
    T: Delay,
{
    pub fn with_delay(output: D, calibration: StandCalibration, delay: T) -> Self {
        Self {
            output,
            calibration,
            delay,
            state: CommandedStandState::Unknown,
        }
    }

    pub const fn state(&self) -> CommandedStandState {
        self.state
    }

    /// Establishes the only trusted initial state without moving a servo.
    pub fn reset(&mut self) -> Result<()> {
        match self.output.all_off() {
            Ok(()) => {
                self.state = CommandedStandState::OutputsOff;
                Ok(())
            }
            Err(error) => self.fault("failed to disable PCA9685 outputs during reset", error),
        }
    }

    /// Stops PWM output and discards the current pose assumption.
    pub fn off(&mut self) -> Result<()> {
        self.reset()
    }

    /// Opens rails, waits for their configured travel time, then moves all
    /// grippers to the collision-free perpendicular configuration.
    pub fn safe_open(&mut self) -> Result<()> {
        self.require_operational("safe-open")?;

        self.set_channels(&rail_channels(&self.calibration, RailPosition::FarOpen))?;
        self.delay
            .sleep(self.calibration.rail_duration(RailPosition::FarOpen));

        let configuration = GripConfiguration::all_frame_perpendicular();
        configuration.validate()?;
        self.set_channels(&gripper_channels(&self.calibration, configuration)?)?;
        self.delay.sleep(self.calibration.gripper_pose_duration());

        self.state = CommandedStandState::SafeOpen;
        Ok(())
    }

    /// Always starts from `safe_open`, then closes rails while retaining PWM on
    /// all four grippers.
    pub fn grip(&mut self) -> Result<()> {
        self.safe_open()?;
        self.set_channels(&rail_channels(&self.calibration, RailPosition::NearGrip))?;
        self.delay
            .sleep(self.calibration.rail_duration(RailPosition::NearGrip));
        self.state = CommandedStandState::Gripped;
        Ok(())
    }

    pub fn into_inner(self) -> D {
        self.output
    }

    fn require_operational(&self, operation: &str) -> Result<()> {
        match self.state {
            CommandedStandState::Unknown => anyhow::bail!(
                "cannot {operation}: runtime state is unknown; call reset before commanding motion"
            ),
            CommandedStandState::Faulted => anyhow::bail!(
                "cannot {operation}: runtime is faulted; call reset and then safe-open"
            ),
            CommandedStandState::OutputsOff
            | CommandedStandState::SafeOpen
            | CommandedStandState::Gripped => Ok(()),
        }
    }

    fn set_channels(&mut self, channels: &[(u8, u16)]) -> Result<()> {
        match self.output.set_channels(channels) {
            Ok(()) => Ok(()),
            Err(error) => self.fault("failed to update PCA9685 PWM outputs", error),
        }
    }

    fn fault<R>(&mut self, message: &str, error: anyhow::Error) -> Result<R> {
        self.state = CommandedStandState::Faulted;
        match self.output.all_off() {
            Ok(()) => Err(error.context(format!("{message}; runtime is now faulted"))),
            Err(off_error) => Err(error.context(format!(
                "{message}; runtime is faulted and best-effort all_off also failed: {off_error:#}"
            ))),
        }
    }
}

fn rail_channels(calibration: &StandCalibration, position: RailPosition) -> Vec<(u8, u16)> {
    StandAxis::RAILS
        .into_iter()
        .map(|axis| (axis.channel(), calibration.rail_pulse(position)))
        .collect()
}

fn gripper_channels(
    calibration: &StandCalibration,
    configuration: GripConfiguration,
) -> Result<Vec<(u8, u16)>> {
    [
        (StandAxis::RightGripper, configuration.right),
        (StandAxis::BottomGripper, configuration.bottom),
        (StandAxis::TopGripper, configuration.top),
        (StandAxis::LeftGripper, configuration.left),
    ]
    .into_iter()
    .map(|(axis, orientation)| {
        calibration
            .gripper_pulse(axis, orientation)
            .map(|pulse_us| (axis.channel(), pulse_us))
            .with_context(|| {
                format!(
                    "{} has no calibrated {} pose",
                    axis.name(),
                    orientation.name()
                )
            })
    })
    .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Clone, PartialEq, Eq)]
    enum OutputEvent {
        Set(Vec<(u8, u16)>),
        AllOff,
    }

    #[derive(Default)]
    struct MockOutput {
        events: Vec<OutputEvent>,
        fail_set_call: Option<usize>,
        set_calls: usize,
    }

    impl PwmOutput for MockOutput {
        fn set_channels(&mut self, channels: &[(u8, u16)]) -> Result<()> {
            self.set_calls += 1;
            if self.fail_set_call == Some(self.set_calls) {
                anyhow::bail!("injected I2C write failure");
            }
            self.events.push(OutputEvent::Set(channels.to_vec()));
            Ok(())
        }

        fn all_off(&mut self) -> Result<()> {
            self.events.push(OutputEvent::AllOff);
            Ok(())
        }
    }

    #[derive(Default)]
    struct MockDelay(Vec<Duration>);

    impl Delay for MockDelay {
        fn sleep(&mut self, duration: Duration) {
            self.0.push(duration);
        }
    }

    fn initialized_runtime(output: MockOutput) -> StandRuntime<MockOutput, MockDelay> {
        let mut runtime =
            StandRuntime::with_delay(output, StandCalibration::default(), MockDelay::default());
        runtime.reset().unwrap();
        runtime
    }

    #[test]
    fn safe_open_requires_explicit_reset() {
        let mut runtime = StandRuntime::with_delay(
            MockOutput::default(),
            StandCalibration::default(),
            MockDelay::default(),
        );

        assert!(runtime.safe_open().is_err());
        assert_eq!(runtime.state(), CommandedStandState::Unknown);
        assert!(runtime.into_inner().events.is_empty());
    }

    #[test]
    fn grip_keeps_gripper_pwm_active_while_rails_close() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();

        assert_eq!(runtime.state(), CommandedStandState::Gripped);
        assert_eq!(
            runtime.delay.0,
            [
                Duration::from_secs(2),
                Duration::from_secs(1),
                Duration::from_secs(2),
            ]
        );
        assert_eq!(
            runtime.output.events,
            vec![
                OutputEvent::AllOff,
                OutputEvent::Set(vec![(4, 2500), (5, 2500), (6, 2500), (7, 2500)]),
                OutputEvent::Set(vec![(0, 1500), (1, 1450), (2, 1450), (3, 1450)]),
                OutputEvent::Set(vec![(4, 1200), (5, 1200), (6, 1200), (7, 1200)]),
            ]
        );
    }

    #[test]
    fn output_failure_faults_runtime_and_stops_outputs() {
        let mut runtime = initialized_runtime(MockOutput {
            fail_set_call: Some(2),
            ..MockOutput::default()
        });

        assert!(runtime.safe_open().is_err());
        assert_eq!(runtime.state(), CommandedStandState::Faulted);
        assert_eq!(runtime.output.events.last(), Some(&OutputEvent::AllOff));
        assert!(runtime.grip().is_err());
        assert_eq!(runtime.output.set_calls, 2);
    }
}
