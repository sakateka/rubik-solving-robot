//! Debounced active-low operator button and its high-level robot workflow.

use crate::robot_service::{LocalCommand, LocalCommandOutcome};
use anyhow::{bail, Context, Result};
use rubik_link_protocol as link;
use std::{
    fs::{File, OpenOptions},
    io::{Read, Seek, SeekFrom, Write},
    path::Path,
    process::Command,
    time::{Duration, Instant},
};

const GPIO_EXPORT_TIMEOUT: Duration = Duration::from_millis(500);
const GPIO_EXPORT_RETRY: Duration = Duration::from_millis(10);
pub const DUO256M_GP21_GPIO: u32 = 506;
const DUO256M_GP21_PINMUX_REGISTER: u32 = 0x0300_104c;
const DUO256M_GP21_PAD_REGISTER: u32 = 0x0300_1918;
const DUO256M_GP21_GPIO_FUNCTION: u32 = 3;
const PINMUX_FUNCTION_MASK: u32 = 0x7;
const PAD_PULL_MASK: u32 = 0xc;
const PAD_PULL_UP: u32 = 0x4;

/// A digital input whose asserted state means that the operator button is held.
pub trait ButtonInput {
    fn pressed(&mut self) -> Result<bool>;
}

impl<T> ButtonInput for Box<T>
where
    T: ButtonInput + ?Sized,
{
    fn pressed(&mut self) -> Result<bool> {
        (**self).pressed()
    }
}

/// Linux sysfs GPIO input wired active-low: GPIO input + button + GND.
pub struct SysfsActiveLowButton {
    value: File,
    gpio: u32,
}

impl SysfsActiveLowButton {
    /// Opens the production Duo256M GP21 button after enabling and verifying
    /// the SG2002 pad's internal weak pull-up.
    pub fn open_duo256m_gp21() -> Result<Self> {
        configure_duo256m_gp21_pull_up()?;
        Self::open(DUO256M_GP21_GPIO)
    }

    pub fn open(gpio: u32) -> Result<Self> {
        Self::open_at(Path::new("/sys/class/gpio"), gpio)
    }

    fn open_at(root: &Path, gpio: u32) -> Result<Self> {
        let gpio_dir = root.join(format!("gpio{gpio}"));
        if !gpio_dir.exists() {
            write_control(&root.join("export"), &gpio.to_string())
                .with_context(|| format!("failed to export GPIO {gpio}"))?;
            let deadline = Instant::now() + GPIO_EXPORT_TIMEOUT;
            while !gpio_dir.exists() && Instant::now() < deadline {
                std::thread::sleep(GPIO_EXPORT_RETRY);
            }
        }
        if !gpio_dir.exists() {
            bail!("GPIO {gpio} did not appear under {}", root.display());
        }

        write_control(&gpio_dir.join("direction"), "in")
            .with_context(|| format!("failed to configure GPIO {gpio} as input"))?;
        let value = OpenOptions::new()
            .read(true)
            .open(gpio_dir.join("value"))
            .with_context(|| format!("failed to open GPIO {gpio} value"))?;
        Ok(Self { value, gpio })
    }

    pub const fn gpio(&self) -> u32 {
        self.gpio
    }
}

fn configure_duo256m_gp21_pull_up() -> Result<()> {
    let function = devmem_read(DUO256M_GP21_PINMUX_REGISTER)?;
    if function & PINMUX_FUNCTION_MASK != DUO256M_GP21_GPIO_FUNCTION {
        bail!(
            "Duo256M GP21 pinmux is 0x{:x}, expected GPIO function 0x{:x}",
            function & PINMUX_FUNCTION_MASK,
            DUO256M_GP21_GPIO_FUNCTION
        );
    }

    let pad = devmem_read(DUO256M_GP21_PAD_REGISTER)?;
    let configured = with_pull_up(pad);
    if configured != pad {
        devmem_write(DUO256M_GP21_PAD_REGISTER, configured)?;
    }
    let verified = devmem_read(DUO256M_GP21_PAD_REGISTER)?;
    if verified & PAD_PULL_MASK != PAD_PULL_UP {
        bail!("failed to enable Duo256M GP21 pull-up: pad register read back as 0x{verified:08x}");
    }
    Ok(())
}

const fn with_pull_up(value: u32) -> u32 {
    (value & !PAD_PULL_MASK) | PAD_PULL_UP
}

fn devmem_read(address: u32) -> Result<u32> {
    let address = format!("0x{address:08x}");
    let output = Command::new("devmem")
        .args([address.as_str(), "32"])
        .output()
        .with_context(|| format!("failed to run devmem for register {address}"))?;
    if !output.status.success() {
        bail!(
            "devmem read {address} failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    let value = String::from_utf8(output.stdout)
        .with_context(|| format!("devmem read {address} returned non-UTF-8 output"))?;
    let value = value.trim();
    let hex = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .unwrap_or(value);
    u32::from_str_radix(hex, 16)
        .with_context(|| format!("devmem read {address} returned invalid value {value:?}"))
}

fn devmem_write(address: u32, value: u32) -> Result<()> {
    let address = format!("0x{address:08x}");
    let value = format!("0x{value:08x}");
    let output = Command::new("devmem")
        .args([address.as_str(), "32", value.as_str()])
        .output()
        .with_context(|| format!("failed to run devmem for register {address}"))?;
    if !output.status.success() {
        bail!(
            "devmem write {address}={value} failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
    }
    Ok(())
}

impl ButtonInput for SysfsActiveLowButton {
    fn pressed(&mut self) -> Result<bool> {
        self.value
            .seek(SeekFrom::Start(0))
            .with_context(|| format!("failed to seek GPIO {} value", self.gpio))?;
        let mut byte = [0u8; 1];
        self.value
            .read_exact(&mut byte)
            .with_context(|| format!("failed to read GPIO {} value", self.gpio))?;
        match byte[0] {
            b'0' => Ok(true),
            b'1' => Ok(false),
            value => bail!("GPIO {} returned invalid value byte {value}", self.gpio),
        }
    }
}

fn write_control(path: &Path, value: &str) -> Result<()> {
    let mut file = OpenOptions::new()
        .write(true)
        .open(path)
        .with_context(|| format!("failed to open {}", path.display()))?;
    file.write_all(value.as_bytes())
        .with_context(|| format!("failed to write {}", path.display()))
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FollowUp {
    None,
    RecoverAfterAbort,
    AutoAfterGrip { operation_id: u32 },
}

/// Converts debounced presses and operation completion into local commands.
pub struct OperatorButton<I> {
    input: I,
    debounce: Duration,
    raw_pressed: Option<bool>,
    raw_changed_at: Instant,
    stable_pressed: bool,
    follow_up: FollowUp,
}

impl<I> OperatorButton<I>
where
    I: ButtonInput,
{
    pub fn new(input: I, debounce: Duration, now: Instant) -> Self {
        Self {
            input,
            debounce,
            raw_pressed: None,
            raw_changed_at: now,
            stable_pressed: false,
            follow_up: FollowUp::None,
        }
    }

    /// Returns at most one command per daemon iteration.
    pub fn poll(
        &mut self,
        status: &link::StatusSnapshot,
        now: Instant,
    ) -> Result<Option<LocalCommand>> {
        let pressed = self.input.pressed()?;
        let press_event = self.update_debounce(pressed, now);
        if press_event {
            self.follow_up = FollowUp::None;
            return Ok(Some(command_for_press(status)));
        }

        let command = match self.follow_up {
            FollowUp::None => None,
            FollowUp::RecoverAfterAbort => {
                if status.controller == link::ControllerState::Aborted
                    && status.active_operation.is_none()
                {
                    self.follow_up = FollowUp::None;
                    Some(LocalCommand::RecoverToOpen)
                } else if status.controller == link::ControllerState::Faulted
                    || status.active_operation.is_some()
                    || status.controller == link::ControllerState::Busy
                {
                    // Never start motion automatically when Abort itself failed
                    // or another source already started a replacement operation.
                    self.follow_up = FollowUp::None;
                    None
                } else {
                    None
                }
            }
            FollowUp::AutoAfterGrip { operation_id } => {
                if let Some(active) = status.active_operation {
                    if active.id != operation_id {
                        self.follow_up = FollowUp::None;
                    }
                    None
                } else if status.controller == link::ControllerState::Ready
                    && status.stand.pose.kind == link::StandPoseKind::CanonicalGrip
                {
                    self.follow_up = FollowUp::None;
                    status
                        .cube_session
                        .map(|session| LocalCommand::ScanSolveExecute {
                            session_id: session.id,
                        })
                } else if status.controller != link::ControllerState::Busy {
                    self.follow_up = FollowUp::None;
                    None
                } else {
                    None
                }
            }
        };
        Ok(command)
    }

    pub fn command_finished(&mut self, command: LocalCommand, outcome: LocalCommandOutcome) {
        self.follow_up = match (command, outcome) {
            (LocalCommand::Abort, LocalCommandOutcome::Accepted { .. }) => {
                FollowUp::RecoverAfterAbort
            }
            (
                LocalCommand::Grip,
                LocalCommandOutcome::Accepted {
                    operation_id: Some(operation_id),
                },
            ) => FollowUp::AutoAfterGrip { operation_id },
            _ => FollowUp::None,
        };
    }

    fn update_debounce(&mut self, pressed: bool, now: Instant) -> bool {
        let Some(raw_pressed) = self.raw_pressed else {
            // A button held while the daemon starts must be released and pressed
            // again; startup alone must never move the stand.
            self.raw_pressed = Some(pressed);
            self.stable_pressed = pressed;
            self.raw_changed_at = now;
            return false;
        };

        if pressed != raw_pressed {
            self.raw_pressed = Some(pressed);
            self.raw_changed_at = now;
            return false;
        }
        if pressed != self.stable_pressed
            && now.saturating_duration_since(self.raw_changed_at) >= self.debounce
        {
            self.stable_pressed = pressed;
            return pressed;
        }
        false
    }
}

fn command_for_press(status: &link::StatusSnapshot) -> LocalCommand {
    if status.active_operation.is_some() || status.controller == link::ControllerState::Busy {
        LocalCommand::Abort
    } else if status.controller == link::ControllerState::Ready
        && status.stand.pose.kind == link::StandPoseKind::Open
    {
        LocalCommand::Grip
    } else {
        LocalCommand::RecoverToOpen
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot_service::unknown_status;
    use std::collections::VecDeque;

    struct FakeButton {
        values: VecDeque<bool>,
        last: bool,
    }

    impl FakeButton {
        fn new(initial: bool) -> Self {
            Self {
                values: VecDeque::new(),
                last: initial,
            }
        }

        fn set(&mut self, pressed: bool) {
            self.values.push_back(pressed);
        }
    }

    impl ButtonInput for FakeButton {
        fn pressed(&mut self) -> Result<bool> {
            if let Some(value) = self.values.pop_front() {
                self.last = value;
            }
            Ok(self.last)
        }
    }

    fn ready_status(pose: link::StandPoseKind) -> link::StatusSnapshot {
        let mut status = unknown_status();
        status.controller = link::ControllerState::Ready;
        status.stand.pose.kind = pose;
        status
    }

    fn debounced_press(
        button: &mut OperatorButton<FakeButton>,
        status: &link::StatusSnapshot,
        base: Instant,
    ) -> Option<LocalCommand> {
        button.input.set(false);
        assert_eq!(button.poll(status, base).unwrap(), None);
        button.input.set(true);
        assert_eq!(
            button
                .poll(status, base + Duration::from_millis(1))
                .unwrap(),
            None
        );
        button
            .poll(status, base + Duration::from_millis(51))
            .unwrap()
    }

    #[test]
    fn held_at_start_does_not_trigger_motion() {
        let base = Instant::now();
        let input = FakeButton::new(true);
        let mut button = OperatorButton::new(input, Duration::from_millis(50), base);
        let status = ready_status(link::StandPoseKind::Open);

        assert_eq!(button.poll(&status, base).unwrap(), None);
        assert_eq!(
            button.poll(&status, base + Duration::from_secs(1)).unwrap(),
            None
        );
    }

    #[test]
    fn pull_up_update_preserves_unrelated_pad_bits() {
        assert_eq!(with_pull_up(0xffff_ffff), 0xffff_fff7);
        assert_eq!(with_pull_up(0x1234_abc0), 0x1234_abc4);
        assert_eq!(with_pull_up(0x1234_abc4), 0x1234_abc4);
    }

    #[test]
    fn open_grips_then_starts_automatic_workflow() {
        let base = Instant::now();
        let input = FakeButton::new(false);
        let mut button = OperatorButton::new(input, Duration::from_millis(50), base);
        let open = ready_status(link::StandPoseKind::Open);

        assert_eq!(
            debounced_press(&mut button, &open, base),
            Some(LocalCommand::Grip)
        );
        button.command_finished(
            LocalCommand::Grip,
            LocalCommandOutcome::Accepted {
                operation_id: Some(7),
            },
        );

        let mut busy = open;
        busy.controller = link::ControllerState::Busy;
        busy.active_operation = Some(link::OperationStatus {
            id: 7,
            kind: link::OperationKind::Grip,
            current_action: 0,
            action_count: 2,
        });
        assert_eq!(
            button
                .poll(&busy, base + Duration::from_millis(60))
                .unwrap(),
            None
        );

        let mut held = ready_status(link::StandPoseKind::CanonicalGrip);
        held.cube_session = Some(link::CubeSessionStatus { id: 42 });
        assert_eq!(
            button.poll(&held, base + Duration::from_secs(3)).unwrap(),
            Some(LocalCommand::ScanSolveExecute { session_id: 42 })
        );
    }

    #[test]
    fn known_closed_or_unknown_pose_recovers() {
        for pose in [
            link::StandPoseKind::Unknown,
            link::StandPoseKind::CanonicalGrip,
            link::StandPoseKind::ScanPose,
            link::StandPoseKind::MovePose,
        ] {
            let base = Instant::now();
            let input = FakeButton::new(false);
            let mut button = OperatorButton::new(input, Duration::from_millis(50), base);
            let status = ready_status(pose);
            assert_eq!(
                debounced_press(&mut button, &status, base),
                Some(LocalCommand::RecoverToOpen)
            );
        }
    }

    #[test]
    fn active_operation_aborts_then_recovers() {
        let base = Instant::now();
        let input = FakeButton::new(false);
        let mut button = OperatorButton::new(input, Duration::from_millis(50), base);
        let mut busy = ready_status(link::StandPoseKind::Transitional);
        busy.controller = link::ControllerState::Busy;
        busy.active_operation = Some(link::OperationStatus {
            id: 9,
            kind: link::OperationKind::Execute,
            current_action: 3,
            action_count: 10,
        });

        assert_eq!(
            debounced_press(&mut button, &busy, base),
            Some(LocalCommand::Abort)
        );
        button.command_finished(
            LocalCommand::Abort,
            LocalCommandOutcome::Accepted { operation_id: None },
        );
        let mut aborted = unknown_status();
        aborted.controller = link::ControllerState::Aborted;
        assert_eq!(
            button
                .poll(&aborted, base + Duration::from_millis(60))
                .unwrap(),
            Some(LocalCommand::RecoverToOpen)
        );
    }

    #[test]
    fn failed_abort_never_starts_motion_automatically() {
        let base = Instant::now();
        let input = FakeButton::new(false);
        let mut button = OperatorButton::new(input, Duration::from_millis(50), base);
        button.command_finished(
            LocalCommand::Abort,
            LocalCommandOutcome::Accepted { operation_id: None },
        );
        let mut faulted = unknown_status();
        faulted.controller = link::ControllerState::Faulted;

        assert_eq!(button.poll(&faulted, base).unwrap(), None);
        assert_eq!(
            button
                .poll(&faulted, base + Duration::from_secs(1))
                .unwrap(),
            None
        );
    }

    #[test]
    fn concurrent_remote_operation_cancels_abort_follow_up() {
        let base = Instant::now();
        let input = FakeButton::new(false);
        let mut button = OperatorButton::new(input, Duration::from_millis(50), base);
        button.command_finished(
            LocalCommand::Abort,
            LocalCommandOutcome::Accepted { operation_id: None },
        );
        let mut busy = unknown_status();
        busy.controller = link::ControllerState::Busy;
        busy.active_operation = Some(link::OperationStatus {
            id: 12,
            kind: link::OperationKind::RecoverToOpen,
            current_action: 0,
            action_count: 3,
        });

        assert_eq!(button.poll(&busy, base).unwrap(), None);
        let mut open = ready_status(link::StandPoseKind::Open);
        open.active_operation = None;
        assert_eq!(
            button.poll(&open, base + Duration::from_secs(4)).unwrap(),
            None
        );
    }
}
