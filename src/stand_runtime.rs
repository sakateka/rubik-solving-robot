//! Stateful, persistent-PWM control for the Rubik stand.
//!
//! This module tracks the last successfully commanded state; it cannot observe
//! physical servo position. A failed I²C operation therefore faults the runtime
//! and requires an explicit reset before another motion is attempted.

use crate::{
    pca9685::PwmOutput,
    stand::{GripConfiguration, GripperOrientation, RailPosition, StandAxis, StandCalibration},
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
    /// A named cube face is open for the camera and held by one opposite pair.
    ScanHold(ScanFace),
    /// An output operation failed; no motion is allowed until `reset` succeeds.
    Faulted,
}

/// A face reached from the initial `F` grip in a canonical scan orientation.
///
/// The names are cube coordinates, not detected sticker colours. Every
/// supported pose exposes the named face without a software grid transform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScanFace {
    Front,
    Left,
    Right,
    Up,
    Down,
    Back,
}

impl ScanFace {
    pub const fn name(self) -> &'static str {
        match self {
            Self::Front => "front",
            Self::Left => "left",
            Self::Right => "right",
            Self::Up => "up",
            Self::Down => "down",
            Self::Back => "back",
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum RailPair {
    LeftRight,
    TopBottom,
}

#[derive(Clone, Copy)]
enum Side {
    Left,
    Right,
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
    holding_pair: Option<RailPair>,
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
            holding_pair: None,
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
                self.holding_pair = None;
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

        self.open_all_rails()?;

        let configuration = GripConfiguration::all_frame_perpendicular();
        configuration.validate()?;
        self.set_channels(&gripper_channels(&self.calibration, configuration)?)?;
        self.delay.sleep(self.calibration.gripper_pose_duration());

        self.state = CommandedStandState::SafeOpen;
        self.holding_pair = None;
        Ok(())
    }

    /// Always starts from `safe_open`, then closes rails while retaining PWM on
    /// all four grippers.
    pub fn grip(&mut self) -> Result<()> {
        if let CommandedStandState::ScanHold(face) = self.state {
            anyhow::bail!(
                "cannot grip from scan-hold {}: it would open the only rails holding the cube",
                face.name()
            );
        }
        self.safe_open()?;
        self.move_all_rails(RailPosition::NearGrip)?;
        self.state = CommandedStandState::Gripped;
        self.holding_pair = None;
        Ok(())
    }

    /// Moves from the initial `Gripped(F)` pose to a camera-open scan pose.
    ///
    /// This is deliberately a bounded validation primitive, not a general
    /// motion planner. Each rail-opening phase completes before any gripper
    /// moves on that pair, and the final state keeps PWM asserted for the
    /// holding pair while the caller inspects the cube.
    pub fn scan_pose(&mut self, face: ScanFace) -> Result<()> {
        if self.state != CommandedStandState::Gripped {
            anyhow::bail!(
                "cannot enter scan pose {}: expected gripped initial pose, current state is {}",
                face.name(),
                state_name(self.state)
            );
        }

        match face {
            ScanFace::Front => self.regrip_left_right_for_scan()?,
            ScanFace::Left => self.side_scan_from_grip(Side::Left)?,
            ScanFace::Right => self.side_scan_from_grip(Side::Right)?,
            ScanFace::Up => self.horizontal_turn_to_up()?,
            ScanFace::Down => self.horizontal_turn_to_down()?,
            ScanFace::Back => self.vertical_turn_to_back()?,
        };
        let holding_pair = match face {
            ScanFace::Front => RailPair::LeftRight,
            ScanFace::Left | ScanFace::Right | ScanFace::Up | ScanFace::Down | ScanFace::Back => {
                RailPair::TopBottom
            }
        };
        self.state = CommandedStandState::ScanHold(face);
        self.holding_pair = Some(holding_pair);
        Ok(())
    }

    /// Performs one verified direct transition between camera-open scan poses.
    ///
    /// Unlike [`Self::scan_pose`], this never invokes `grip` or `safe_open`.
    /// The private holding-pair tag distinguishes mechanically different
    /// camera poses with the same face name. In particular, `Front` may be
    /// held by left/right in a diagnostic pose or by top/bottom in the
    /// canonical scan flow; only the latter can continue to `Back`.
    pub fn scan_next(&mut self, next: ScanFace) -> Result<()> {
        match (self.state, self.holding_pair, next) {
            (
                CommandedStandState::ScanHold(ScanFace::Front),
                Some(RailPair::LeftRight),
                ScanFace::Up,
            ) => {
                self.pose_grippers(&[
                    (
                        StandAxis::LeftGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                    (
                        StandAxis::RightGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                ])?;
                self.regrip_top_bottom_for_scan()?;
                self.set_scan_hold(ScanFace::Up, RailPair::TopBottom);
                Ok(())
            }
            (
                CommandedStandState::ScanHold(ScanFace::Left),
                Some(RailPair::TopBottom),
                ScanFace::Right,
            ) => {
                self.pose_grippers(&[
                    (
                        StandAxis::TopGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                    (
                        StandAxis::BottomGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                ])?;
                self.pose_grippers(&[
                    (
                        StandAxis::TopGripper,
                        GripperOrientation::FrameParallelReversed,
                    ),
                    (StandAxis::BottomGripper, GripperOrientation::FrameParallel),
                ])?;
                self.set_scan_hold(ScanFace::Right, RailPair::TopBottom);
                Ok(())
            }
            (
                CommandedStandState::ScanHold(ScanFace::Right),
                Some(RailPair::TopBottom),
                ScanFace::Down,
            ) => {
                // Return to FrontReference(top/bottom), then hand the cube to
                // left/right before top/bottom rails open.
                self.pose_grippers(&[
                    (
                        StandAxis::TopGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                    (
                        StandAxis::BottomGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                ])?;
                self.close_pair(RailPair::LeftRight)?;
                self.open_pair(RailPair::TopBottom)?;
                self.pose_grippers(&[
                    (StandAxis::LeftGripper, GripperOrientation::FrameParallel),
                    (
                        StandAxis::RightGripper,
                        GripperOrientation::FrameParallelReversed,
                    ),
                ])?;
                self.set_scan_hold(ScanFace::Down, RailPair::LeftRight);
                Ok(())
            }
            (
                CommandedStandState::ScanHold(ScanFace::Down),
                Some(RailPair::LeftRight),
                ScanFace::Up,
            ) => {
                self.pose_grippers(&[
                    (
                        StandAxis::LeftGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                    (
                        StandAxis::RightGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                ])?;
                self.pose_grippers(&[
                    (
                        StandAxis::LeftGripper,
                        GripperOrientation::FrameParallelReversed,
                    ),
                    (StandAxis::RightGripper, GripperOrientation::FrameParallel),
                ])?;
                self.set_scan_hold(ScanFace::Up, RailPair::LeftRight);
                Ok(())
            }
            (
                CommandedStandState::ScanHold(ScanFace::Up),
                Some(RailPair::LeftRight),
                ScanFace::Front,
            ) => {
                // Return to FrontReference(left/right), then hand the cube to
                // top/bottom before left/right rails open.
                self.pose_grippers(&[
                    (
                        StandAxis::LeftGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                    (
                        StandAxis::RightGripper,
                        GripperOrientation::FramePerpendicular,
                    ),
                ])?;
                self.pose_grippers(&[
                    (StandAxis::TopGripper, GripperOrientation::FrameParallel),
                    (
                        StandAxis::BottomGripper,
                        GripperOrientation::FrameParallelReversed,
                    ),
                ])?;
                self.close_pair(RailPair::TopBottom)?;
                self.open_pair(RailPair::LeftRight)?;
                self.set_scan_hold(ScanFace::Front, RailPair::TopBottom);
                Ok(())
            }
            (
                CommandedStandState::ScanHold(ScanFace::Front),
                Some(RailPair::TopBottom),
                ScanFace::Back,
            ) => {
                self.pose_grippers(&[
                    (
                        StandAxis::TopGripper,
                        GripperOrientation::FrameParallelReversed,
                    ),
                    (StandAxis::BottomGripper, GripperOrientation::FrameParallel),
                ])?;
                self.set_scan_hold(ScanFace::Back, RailPair::TopBottom);
                Ok(())
            }
            (CommandedStandState::ScanHold(current), holding_pair, _) => anyhow::bail!(
                "no verified direct scan transition from {} ({}) to {}",
                current.name(),
                holding_pair.map(rail_pair_name).unwrap_or("unknown holder"),
                next.name()
            ),
            (state, _, _) => anyhow::bail!(
                "cannot transition to scan pose {}: current state is {}",
                next.name(),
                state_name(state)
            ),
        }
    }

    /// Returns a completed canonical scan to the initial front-facing open pose.
    ///
    /// This is available only after the `... -> F -> B` scan tail, where
    /// top/bottom still holds the cube. It returns `B -> F`, regrips both
    /// pairs in perpendicular orientations, then opens the stand without a
    /// subsequent gripper turn.
    pub fn finish_scan(&mut self) -> Result<()> {
        match (self.state, self.holding_pair) {
            (CommandedStandState::ScanHold(ScanFace::Back), Some(RailPair::TopBottom)) => {}
            (CommandedStandState::ScanHold(face), holding_pair) => anyhow::bail!(
                "cannot finish scan from {} ({})",
                face.name(),
                holding_pair.map(rail_pair_name).unwrap_or("unknown holder")
            ),
            (state, _) => {
                anyhow::bail!("cannot finish scan: current state is {}", state_name(state))
            }
        }

        self.pose_grippers(&[
            (StandAxis::TopGripper, GripperOrientation::FrameParallel),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FrameParallelReversed,
            ),
        ])?;
        self.close_pair(RailPair::LeftRight)?;
        self.open_pair(RailPair::TopBottom)?;
        self.pose_grippers(&[
            (
                StandAxis::TopGripper,
                GripperOrientation::FramePerpendicular,
            ),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FramePerpendicular,
            ),
        ])?;
        self.close_pair(RailPair::TopBottom)?;
        self.open_all_rails()?;
        self.state = CommandedStandState::SafeOpen;
        self.holding_pair = None;
        Ok(())
    }

    pub fn into_inner(self) -> D {
        self.output
    }

    fn set_scan_hold(&mut self, face: ScanFace, holding_pair: RailPair) {
        self.state = CommandedStandState::ScanHold(face);
        self.holding_pair = Some(holding_pair);
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
            | CommandedStandState::Gripped
            | CommandedStandState::ScanHold(_) => Ok(()),
        }
    }

    fn set_channels(&mut self, channels: &[(u8, u16)]) -> Result<()> {
        match self.output.set_channels(channels) {
            Ok(()) => Ok(()),
            Err(error) => self.fault("failed to update PCA9685 PWM outputs", error),
        }
    }

    fn disable_channels(&mut self, channels: &[u8]) -> Result<()> {
        match self.output.disable_channels(channels) {
            Ok(()) => Ok(()),
            Err(error) => self.fault("failed to disable completed PCA9685 rail motion", error),
        }
    }

    fn move_all_rails(&mut self, position: RailPosition) -> Result<()> {
        self.set_channels(&rail_channels(&self.calibration, position))?;
        self.delay.sleep(self.calibration.rail_duration(position));
        self.disable_channels(&rail_channel_ids_all())
    }

    fn open_all_rails(&mut self) -> Result<()> {
        self.move_all_rails(RailPosition::FarOpen)
    }

    fn open_pair(&mut self, pair: RailPair) -> Result<()> {
        self.set_channels(&rail_pair_channels(
            &self.calibration,
            pair,
            RailPosition::FarOpen,
        ))?;
        self.delay
            .sleep(self.calibration.rail_duration(RailPosition::FarOpen));
        self.disable_channels(&rail_channel_ids(pair))
    }

    fn close_pair(&mut self, pair: RailPair) -> Result<()> {
        self.set_channels(&rail_pair_channels(
            &self.calibration,
            pair,
            RailPosition::NearGrip,
        ))?;
        self.delay
            .sleep(self.calibration.rail_duration(RailPosition::NearGrip));
        self.disable_channels(&rail_channel_ids(pair))
    }

    fn pose_grippers(&mut self, poses: &[(StandAxis, GripperOrientation)]) -> Result<()> {
        self.set_channels(&gripper_channels_for_axes(&self.calibration, poses)?)?;
        self.delay.sleep(self.calibration.gripper_pose_duration());
        Ok(())
    }

    /// Canonical `F -> R -> B` turn around the top/bottom axis.
    fn vertical_turn_to_back(&mut self) -> Result<()> {
        self.open_pair(RailPair::TopBottom)?;
        self.pose_grippers(&[
            (StandAxis::TopGripper, GripperOrientation::FrameParallel),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FrameParallelReversed,
            ),
        ])?;
        self.close_pair(RailPair::TopBottom)?;
        self.open_pair(RailPair::LeftRight)?;
        self.pose_grippers(&[
            (
                StandAxis::TopGripper,
                GripperOrientation::FramePerpendicular,
            ),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FramePerpendicular,
            ),
        ])?;

        // Continue the same turn from R to B. Parallel top/bottom grippers
        // leave B open, so no additional regrip is needed.
        self.pose_grippers(&[
            (
                StandAxis::TopGripper,
                GripperOrientation::FrameParallelReversed,
            ),
            (StandAxis::BottomGripper, GripperOrientation::FrameParallel),
        ])
    }

    /// Opens left/right rails and scans a side while top/bottom keeps holding.
    fn side_scan_from_grip(&mut self, side: Side) -> Result<()> {
        self.open_pair(RailPair::LeftRight)?;
        let (top, bottom) = match side {
            Side::Right => (
                GripperOrientation::FrameParallelReversed,
                GripperOrientation::FrameParallel,
            ),
            Side::Left => (
                GripperOrientation::FrameParallel,
                GripperOrientation::FrameParallelReversed,
            ),
        };
        self.pose_grippers(&[
            (StandAxis::TopGripper, top),
            (StandAxis::BottomGripper, bottom),
        ])
    }

    /// Opens the camera view while retaining the cube with left/right.
    fn regrip_left_right_for_scan(&mut self) -> Result<()> {
        self.open_pair(RailPair::LeftRight)?;
        self.enter_left_right_scan_hold()
    }

    /// Completes a left/right regrip after those rails have fully opened.
    fn enter_left_right_scan_hold(&mut self) -> Result<()> {
        self.pose_grippers(&[
            (StandAxis::LeftGripper, GripperOrientation::FrameParallel),
            (
                StandAxis::RightGripper,
                GripperOrientation::FrameParallelReversed,
            ),
        ])?;
        self.close_pair(RailPair::LeftRight)?;
        self.open_pair(RailPair::TopBottom)
    }

    /// `F -> U`, then regrip on top/bottom to expose the face.
    fn horizontal_turn_to_up(&mut self) -> Result<()> {
        self.open_pair(RailPair::LeftRight)?;
        self.pose_grippers(&[
            (StandAxis::LeftGripper, GripperOrientation::FrameParallel),
            (
                StandAxis::RightGripper,
                GripperOrientation::FrameParallelReversed,
            ),
        ])?;
        self.close_pair(RailPair::LeftRight)?;
        self.open_pair(RailPair::TopBottom)?;
        self.pose_grippers(&[
            (
                StandAxis::LeftGripper,
                GripperOrientation::FramePerpendicular,
            ),
            (
                StandAxis::RightGripper,
                GripperOrientation::FramePerpendicular,
            ),
        ])?;

        self.regrip_top_bottom_for_scan()
    }

    /// `F -> D`, then regrip on top/bottom to expose the face.
    fn horizontal_turn_to_down(&mut self) -> Result<()> {
        self.open_pair(RailPair::LeftRight)?;
        self.pose_grippers(&[
            (
                StandAxis::LeftGripper,
                GripperOrientation::FrameParallelReversed,
            ),
            (StandAxis::RightGripper, GripperOrientation::FrameParallel),
        ])?;
        self.close_pair(RailPair::LeftRight)?;
        self.open_pair(RailPair::TopBottom)?;
        self.pose_grippers(&[
            (
                StandAxis::LeftGripper,
                GripperOrientation::FramePerpendicular,
            ),
            (
                StandAxis::RightGripper,
                GripperOrientation::FramePerpendicular,
            ),
        ])?;

        self.regrip_top_bottom_for_scan()
    }

    /// Opens the camera view while retaining the cube with top/bottom.
    fn regrip_top_bottom_for_scan(&mut self) -> Result<()> {
        // The top/bottom rails are already open in the F -> U/D paths.
        self.pose_grippers(&[
            (StandAxis::TopGripper, GripperOrientation::FrameParallel),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FrameParallelReversed,
            ),
        ])?;
        self.close_pair(RailPair::TopBottom)?;
        self.open_pair(RailPair::LeftRight)
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

fn rail_pair_channels(
    calibration: &StandCalibration,
    pair: RailPair,
    position: RailPosition,
) -> Vec<(u8, u16)> {
    let axes = match pair {
        RailPair::LeftRight => [StandAxis::LeftRail, StandAxis::RightRail],
        RailPair::TopBottom => [StandAxis::TopRail, StandAxis::BottomRail],
    };
    axes.into_iter()
        .map(|axis| (axis.channel(), calibration.rail_pulse(position)))
        .collect()
}

fn rail_channel_ids_all() -> Vec<u8> {
    StandAxis::RAILS
        .into_iter()
        .map(StandAxis::channel)
        .collect()
}

fn rail_channel_ids(pair: RailPair) -> Vec<u8> {
    let axes = match pair {
        RailPair::LeftRight => [StandAxis::LeftRail, StandAxis::RightRail],
        RailPair::TopBottom => [StandAxis::TopRail, StandAxis::BottomRail],
    };
    axes.into_iter().map(StandAxis::channel).collect()
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

fn gripper_channels_for_axes(
    calibration: &StandCalibration,
    poses: &[(StandAxis, GripperOrientation)],
) -> Result<Vec<(u8, u16)>> {
    poses
        .iter()
        .copied()
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

fn state_name(state: CommandedStandState) -> &'static str {
    match state {
        CommandedStandState::Unknown => "unknown",
        CommandedStandState::OutputsOff => "outputs-off",
        CommandedStandState::SafeOpen => "safe-open",
        CommandedStandState::Gripped => "gripped",
        CommandedStandState::ScanHold(face) => face.name(),
        CommandedStandState::Faulted => "faulted",
    }
}

const fn rail_pair_name(pair: RailPair) -> &'static str {
    match pair {
        RailPair::LeftRight => "left/right holder",
        RailPair::TopBottom => "top/bottom holder",
    }
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
        disabled_channels: Vec<Vec<u8>>,
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

        fn disable_channels(&mut self, channels: &[u8]) -> Result<()> {
            self.disabled_channels.push(channels.to_vec());
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
                Duration::from_millis(1_200),
                Duration::from_secs(1),
                Duration::from_millis(1_200),
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
        assert_eq!(
            runtime.output.disabled_channels,
            [vec![4, 5, 6, 7], vec![4, 5, 6, 7]]
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

    #[test]
    fn left_scan_pose_opens_left_right_and_uses_top_bottom_endpoints() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Left).unwrap();

        assert_eq!(
            runtime.state(),
            CommandedStandState::ScanHold(ScanFace::Left)
        );
        assert_eq!(
            runtime.output.events,
            vec![
                OutputEvent::AllOff,
                OutputEvent::Set(vec![(4, 2500), (5, 2500), (6, 2500), (7, 2500)]),
                OutputEvent::Set(vec![(0, 1500), (1, 1450), (2, 1450), (3, 1450)]),
                OutputEvent::Set(vec![(4, 1200), (5, 1200), (6, 1200), (7, 1200)]),
                // Top/bottom retains the cube while left/right opens for L.
                OutputEvent::Set(vec![(5, 2500), (7, 2500)]),
                OutputEvent::Set(vec![(2, 400), (1, 2500)]),
            ]
        );
        assert_eq!(runtime.delay.0.len(), 5);
    }

    #[test]
    fn scan_pose_requires_the_initial_gripped_pose() {
        let mut runtime = initialized_runtime(MockOutput::default());

        assert!(runtime.scan_pose(ScanFace::Right).is_err());
        assert_eq!(runtime.state(), CommandedStandState::OutputsOff);
        assert_eq!(runtime.output.events, vec![OutputEvent::AllOff]);
    }

    #[test]
    fn grip_is_rejected_from_scan_hold_without_commanding_any_motion() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Front).unwrap();
        let events_before = runtime.output.events.clone();

        assert!(runtime.grip().is_err());
        assert_eq!(
            runtime.state(),
            CommandedStandState::ScanHold(ScanFace::Front)
        );
        assert_eq!(runtime.output.events, events_before);
    }

    #[test]
    fn scan_next_from_front_to_up_never_releases_the_cube() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Front).unwrap();
        let events_before = runtime.output.events.len();

        runtime.scan_next(ScanFace::Up).unwrap();

        assert_eq!(runtime.state(), CommandedStandState::ScanHold(ScanFace::Up));
        assert_eq!(
            &runtime.output.events[events_before..],
            [
                OutputEvent::Set(vec![(3, 1450), (0, 1500)]),
                OutputEvent::Set(vec![(2, 400), (1, 2500)]),
                OutputEvent::Set(vec![(4, 1200), (6, 1200)]),
                OutputEvent::Set(vec![(5, 2500), (7, 2500)]),
            ]
        );
    }

    #[test]
    fn scan_next_from_right_to_down_regrips_before_opening_top_bottom() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Left).unwrap();
        runtime.scan_next(ScanFace::Right).unwrap();
        let events_before = runtime.output.events.len();

        runtime.scan_next(ScanFace::Down).unwrap();

        assert_eq!(
            runtime.state(),
            CommandedStandState::ScanHold(ScanFace::Down)
        );
        assert_eq!(
            &runtime.output.events[events_before..],
            [
                OutputEvent::Set(vec![(2, 1450), (1, 1450)]),
                OutputEvent::Set(vec![(5, 1200), (7, 1200)]),
                OutputEvent::Set(vec![(4, 2500), (6, 2500)]),
                OutputEvent::Set(vec![(3, 400), (0, 2500)]),
            ]
        );
    }

    #[test]
    fn scan_next_from_down_to_up_returns_through_front_reference() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Left).unwrap();
        runtime.scan_next(ScanFace::Right).unwrap();
        runtime.scan_next(ScanFace::Down).unwrap();
        let events_before = runtime.output.events.len();

        runtime.scan_next(ScanFace::Up).unwrap();

        assert_eq!(runtime.state(), CommandedStandState::ScanHold(ScanFace::Up));
        assert_eq!(
            &runtime.output.events[events_before..],
            [
                OutputEvent::Set(vec![(3, 1450), (0, 1500)]),
                OutputEvent::Set(vec![(3, 2450), (0, 450)]),
            ]
        );
    }

    #[test]
    fn scan_next_from_up_to_front_regrips_before_opening_left_right() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Left).unwrap();
        runtime.scan_next(ScanFace::Right).unwrap();
        runtime.scan_next(ScanFace::Down).unwrap();
        runtime.scan_next(ScanFace::Up).unwrap();
        let events_before = runtime.output.events.len();

        runtime.scan_next(ScanFace::Front).unwrap();

        assert_eq!(
            runtime.state(),
            CommandedStandState::ScanHold(ScanFace::Front)
        );
        assert_eq!(
            &runtime.output.events[events_before..],
            [
                OutputEvent::Set(vec![(3, 1450), (0, 1500)]),
                OutputEvent::Set(vec![(2, 400), (1, 2500)]),
                OutputEvent::Set(vec![(4, 1200), (6, 1200)]),
                OutputEvent::Set(vec![(5, 2500), (7, 2500)]),
            ]
        );
    }

    #[test]
    fn scan_next_from_front_to_back_keeps_top_bottom_holding() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Left).unwrap();
        runtime.scan_next(ScanFace::Right).unwrap();
        runtime.scan_next(ScanFace::Down).unwrap();
        runtime.scan_next(ScanFace::Up).unwrap();
        runtime.scan_next(ScanFace::Front).unwrap();
        let events_before = runtime.output.events.len();

        runtime.scan_next(ScanFace::Back).unwrap();

        assert_eq!(
            runtime.state(),
            CommandedStandState::ScanHold(ScanFace::Back)
        );
        assert_eq!(
            &runtime.output.events[events_before..],
            [OutputEvent::Set(vec![(2, 2450), (1, 400)])]
        );
    }

    #[test]
    fn scan_next_refuses_back_from_diagnostic_front_pose() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Front).unwrap();
        let events_before = runtime.output.events.clone();

        assert!(runtime.scan_next(ScanFace::Back).is_err());
        assert_eq!(
            runtime.state(),
            CommandedStandState::ScanHold(ScanFace::Front)
        );
        assert_eq!(runtime.output.events, events_before);
    }

    #[test]
    fn finish_scan_returns_back_to_front_before_opening_the_stand() {
        let mut runtime = initialized_runtime(MockOutput::default());
        runtime.grip().unwrap();
        runtime.scan_pose(ScanFace::Left).unwrap();
        runtime.scan_next(ScanFace::Right).unwrap();
        runtime.scan_next(ScanFace::Down).unwrap();
        runtime.scan_next(ScanFace::Up).unwrap();
        runtime.scan_next(ScanFace::Front).unwrap();
        runtime.scan_next(ScanFace::Back).unwrap();
        let events_before = runtime.output.events.len();

        runtime.finish_scan().unwrap();

        assert_eq!(runtime.state(), CommandedStandState::SafeOpen);
        assert_eq!(
            &runtime.output.events[events_before..],
            [
                OutputEvent::Set(vec![(2, 400), (1, 2500)]),
                OutputEvent::Set(vec![(5, 1200), (7, 1200)]),
                OutputEvent::Set(vec![(4, 2500), (6, 2500)]),
                OutputEvent::Set(vec![(2, 1450), (1, 1450)]),
                OutputEvent::Set(vec![(4, 1200), (6, 1200)]),
                OutputEvent::Set(vec![(4, 2500), (5, 2500), (6, 2500), (7, 2500)]),
            ]
        );
    }
}
