//! Deadline-driven robot control service for UART/BLE commands.
//!
//! The service owns the PWM device and never blocks the event loop with motion
//! delays. Callers interleave [`RobotService::tick`] with transport processing,
//! which keeps `Abort` and `GetStatus` responsive while servo movement
//! deadlines are pending.

use crate::{
    cube::{
        parse_solution, solve_facelets, CubeMove, CubeState, Face, LogicalFace, MoveTurn,
        QuarterTurns, ScanPose,
    },
    move_planner::{
        append_open_steps, optimized_execute_steps, optimized_held_steps, MovePlanStep, RailPair,
        RailTarget,
    },
    pca9685::PwmOutput,
    robot_link::{FrameEncodeError, ReceivedPacket, UartFrameEncoder},
    stand::{GripperOrientation, RailPosition, StandAxis, StandCalibration},
};
use rubik_link_protocol as link;
use std::{
    collections::VecDeque,
    sync::mpsc,
    time::{Duration, Instant},
};

const REQUEST_CACHE_CAPACITY: usize = 16;
const SOLVER_MAX_MOVES: u8 = 21;

pub trait FaceScanner {
    fn available(&self) -> bool {
        true
    }

    fn begin_scan(&mut self, _revision: u32) -> anyhow::Result<()> {
        Ok(())
    }

    fn capture(&mut self, face: link::CubeFace) -> anyhow::Result<link::RecognizedFace>;

    fn finish_scan(&mut self, _status: &link::ScanStatus) -> anyhow::Result<()> {
        Ok(())
    }

    fn abort(&mut self) {}
}

#[derive(Default)]
pub struct UnavailableScanner;

impl FaceScanner for UnavailableScanner {
    fn available(&self) -> bool {
        false
    }

    fn capture(&mut self, _face: link::CubeFace) -> anyhow::Result<link::RecognizedFace> {
        anyhow::bail!("scanner backend is unavailable")
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ResponseMessage {
    pub request_id: u32,
    pub opcode: link::ResponseOpcode,
    pub payload: ResponsePayload,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ResponsePayload {
    Accepted(link::CommandAccepted),
    Rejected(link::CommandRejected),
    Status(Box<link::StatusSnapshot>),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EventMessage {
    RobotStateChanged(link::RobotStateChanged),
    StandStateChanged(link::StandStateChanged),
    CubeSessionChanged(link::CubeSessionChanged),
    FaceScanned(link::FaceScanned),
    OperationFailed(link::OperationFailed),
    OperationCompleted(link::OperationCompleted),
    Aborted(link::Aborted),
    Fault(link::FaultEvent),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ServiceMessage {
    Response(ResponseMessage),
    Event(EventMessage),
}

/// A command generated locally on the Duo rather than by a packet transport.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LocalCommand {
    RecoverToOpen,
    Grip,
    ScanSolveExecute { session_id: u32 },
    Abort,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LocalCommandOutcome {
    Accepted { operation_id: Option<u32> },
    Rejected(link::RejectionReason),
}

impl ServiceMessage {
    pub fn encode_uart<'a>(
        &self,
        encoder: &'a mut UartFrameEncoder,
    ) -> Result<&'a [u8], FrameEncodeError> {
        match self {
            Self::Response(response) => match &response.payload {
                ResponsePayload::Accepted(payload) => encoder.encode(
                    link::MessageKind::Response,
                    response.opcode.into(),
                    response.request_id,
                    payload,
                ),
                ResponsePayload::Rejected(payload) => encoder.encode(
                    link::MessageKind::Response,
                    response.opcode.into(),
                    response.request_id,
                    payload,
                ),
                ResponsePayload::Status(payload) => encoder.encode(
                    link::MessageKind::Response,
                    response.opcode.into(),
                    response.request_id,
                    payload.as_ref(),
                ),
            },
            Self::Event(event) => match event {
                EventMessage::RobotStateChanged(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::RobotStateChanged.into(),
                    0,
                    payload,
                ),
                EventMessage::StandStateChanged(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::StandStateChanged.into(),
                    0,
                    payload,
                ),
                EventMessage::CubeSessionChanged(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::CubeSessionChanged.into(),
                    0,
                    payload,
                ),
                EventMessage::FaceScanned(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::FaceScanned.into(),
                    0,
                    payload,
                ),
                EventMessage::OperationFailed(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::OperationFailed.into(),
                    0,
                    payload,
                ),
                EventMessage::OperationCompleted(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::OperationCompleted.into(),
                    0,
                    payload,
                ),
                EventMessage::Aborted(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::Aborted.into(),
                    0,
                    payload,
                ),
                EventMessage::Fault(payload) => encoder.encode(
                    link::MessageKind::Event,
                    link::EventOpcode::Fault.into(),
                    0,
                    payload,
                ),
            },
        }
    }
}

#[derive(Clone)]
struct CachedResponse {
    request_opcode: u16,
    response: ResponseMessage,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum MotionPhase {
    RecoverStart,
    RecoverFinishLeftRight,
    RecoverFinishTopBottom,
    RecoverFinishGrippers,
    GripStart,
    GripFinishGrippers,
    GripFinishRails,
}

#[derive(Clone, Copy, Debug)]
struct ActiveMotion {
    operation_id: u32,
    kind: link::OperationKind,
    phase: MotionPhase,
    deadline: Option<Instant>,
}

#[derive(Clone, Debug)]
enum MotionStep {
    SetRails(RailTarget, RailPosition),
    SetGrippers(Vec<(StandAxis, GripperOrientation)>),
    Capture(link::CubeFace),
    MoveCompleted,
    AllOff,
}

#[derive(Clone, Debug)]
enum PendingMotionStep {
    Rails(RailTarget, RailPosition),
    Grippers(Vec<(StandAxis, GripperOrientation)>),
}

struct ActiveScan {
    operation_id: u32,
    kind: link::OperationKind,
    steps: VecDeque<MotionStep>,
    pending: Option<PendingMotionStep>,
    deadline: Option<Instant>,
    cube: CubeState,
    failure: Option<link::OperationFailureKind>,
}

struct ActiveSolve {
    operation_id: u32,
    kind: link::OperationKind,
    scan_revision: u32,
    receiver: mpsc::Receiver<Result<Vec<CubeMove>, String>>,
}

struct ActiveExecute {
    operation_id: u32,
    kind: link::OperationKind,
    steps: VecDeque<MotionStep>,
    pending: Option<PendingMotionStep>,
    deadline: Option<Instant>,
}

pub struct RobotService<D, S = UnavailableScanner> {
    output: D,
    calibration: StandCalibration,
    status: link::StatusSnapshot,
    active_motion: Option<ActiveMotion>,
    active_scan: Option<ActiveScan>,
    active_solve: Option<ActiveSolve>,
    active_execute: Option<ActiveExecute>,
    scanner: S,
    next_operation_id: u32,
    next_session_id: u32,
    next_scan_revision: u32,
    next_solution_id: u32,
    request_cache: VecDeque<CachedResponse>,
    events: VecDeque<EventMessage>,
}

impl<D> RobotService<D, UnavailableScanner>
where
    D: PwmOutput,
{
    pub fn new(output: D, calibration: StandCalibration) -> Self {
        Self::with_scanner(output, calibration, UnavailableScanner)
    }
}

impl<D, S> RobotService<D, S>
where
    D: PwmOutput,
    S: FaceScanner,
{
    pub fn with_scanner(output: D, calibration: StandCalibration, scanner: S) -> Self {
        Self {
            output,
            calibration,
            status: unknown_status(),
            active_motion: None,
            active_scan: None,
            active_solve: None,
            active_execute: None,
            scanner,
            next_operation_id: 1,
            next_session_id: 1,
            next_scan_revision: 1,
            next_solution_id: 1,
            request_cache: VecDeque::with_capacity(REQUEST_CACHE_CAPACITY),
            events: VecDeque::new(),
        }
    }

    pub const fn status(&self) -> &link::StatusSnapshot {
        &self.status
    }

    pub fn handle_packet(&mut self, packet: &ReceivedPacket, now: Instant) -> Vec<ServiceMessage> {
        let response = if let Some(cached) = self
            .request_cache
            .iter()
            .find(|entry| entry.response.request_id == packet.request_id)
        {
            if cached.request_opcode == packet.opcode {
                cached.response.clone()
            } else {
                self.rejected(packet.request_id, link::RejectionReason::InvalidPayload)
            }
        } else {
            let response = self.dispatch_new_request(packet, now);
            self.cache_response(packet.opcode, &response);
            response
        };

        let mut messages = vec![ServiceMessage::Response(response)];
        messages.extend(self.drain_events().map(ServiceMessage::Event));
        messages
    }

    /// Runs a command from a local hardware control through the same admission
    /// and operation primitives as protocol requests. Only events are returned:
    /// there is no remote requester to receive a protocol response.
    pub fn handle_local_command(
        &mut self,
        command: LocalCommand,
        now: Instant,
    ) -> (LocalCommandOutcome, Vec<ServiceMessage>) {
        let outcome = match command {
            LocalCommand::RecoverToOpen => self
                .start_operation(link::OperationKind::RecoverToOpen, now)
                .map_or_else(LocalCommandOutcome::Rejected, |operation_id| {
                    LocalCommandOutcome::Accepted {
                        operation_id: Some(operation_id),
                    }
                }),
            LocalCommand::Grip => self
                .start_operation(link::OperationKind::Grip, now)
                .map_or_else(LocalCommandOutcome::Rejected, |operation_id| {
                    LocalCommandOutcome::Accepted {
                        operation_id: Some(operation_id),
                    }
                }),
            LocalCommand::ScanSolveExecute { session_id } => {
                let response =
                    self.start_scan_operation(0, session_id, link::OperationKind::ScanSolveExecute);
                match response.payload {
                    ResponsePayload::Accepted(accepted) => LocalCommandOutcome::Accepted {
                        operation_id: accepted.operation_id,
                    },
                    ResponsePayload::Rejected(rejected) => {
                        LocalCommandOutcome::Rejected(rejected.reason)
                    }
                    ResponsePayload::Status(_) => unreachable!("local operation returned status"),
                }
            }
            LocalCommand::Abort => {
                let operation_id = self.status.active_operation.map(|operation| operation.id);
                self.abort(operation_id);
                LocalCommandOutcome::Accepted { operation_id: None }
            }
        };
        let messages = self.drain_events().map(ServiceMessage::Event).collect();
        (outcome, messages)
    }

    pub fn tick(&mut self, now: Instant) -> Vec<ServiceMessage> {
        if let Some(active) = self.active_motion {
            let deadline_elapsed = match active.deadline {
                Some(deadline) => now >= deadline,
                None => true,
            };
            if deadline_elapsed {
                self.advance_motion(now);
            }
        }
        self.advance_scan(now);
        self.advance_solve();
        self.advance_execute(now);
        self.drain_events().map(ServiceMessage::Event).collect()
    }

    pub fn shutdown(&mut self) -> anyhow::Result<()> {
        self.scanner.abort();
        self.output.all_off()?;
        self.status.stand.outputs_enabled = false;
        Ok(())
    }

    pub fn into_inner(self) -> D {
        self.output
    }

    fn dispatch_new_request(&mut self, packet: &ReceivedPacket, now: Instant) -> ResponseMessage {
        if packet.kind != link::MessageKind::Request || packet.request_id == 0 {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        }

        let Ok(opcode) = link::RequestOpcode::try_from(packet.opcode) else {
            return self.rejected(packet.request_id, link::RejectionReason::UnsupportedCommand);
        };

        match opcode {
            link::RequestOpcode::GetStatus => {
                if !packet.payload().is_empty() {
                    return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
                }
                ResponseMessage {
                    request_id: packet.request_id,
                    opcode: link::ResponseOpcode::StatusSnapshot,
                    payload: ResponsePayload::Status(Box::new(self.status)),
                }
            }
            link::RequestOpcode::RecoverToOpen => {
                self.start_empty_payload_operation(packet, now, link::OperationKind::RecoverToOpen)
            }
            link::RequestOpcode::Grip => {
                self.start_empty_payload_operation(packet, now, link::OperationKind::Grip)
            }
            link::RequestOpcode::StartScan => self.start_scan_request(packet, now),
            link::RequestOpcode::Solve => self.start_solve_request(packet),
            link::RequestOpcode::Execute => self.start_execute_request(packet),
            link::RequestOpcode::ExecuteMoves => self.start_execute_moves_request(packet),
            link::RequestOpcode::ScanSolveExecute => self.start_scan_solve_execute_request(packet),
            link::RequestOpcode::Open => self.start_open_request(packet),
            link::RequestOpcode::Abort => {
                if !packet.payload().is_empty() {
                    return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
                }
                let operation_id = self.status.active_operation.map(|operation| operation.id);
                self.abort(operation_id);
                self.accepted(packet.request_id, None)
            }
        }
    }

    fn start_scan_request(&mut self, packet: &ReceivedPacket, _now: Instant) -> ResponseMessage {
        let Ok(command) = link::decode_payload::<link::StartScanCommand>(packet.payload()) else {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        };
        self.start_scan_operation(
            packet.request_id,
            command.session_id,
            link::OperationKind::Scan,
        )
    }

    fn start_scan_solve_execute_request(&mut self, packet: &ReceivedPacket) -> ResponseMessage {
        let Ok(command) = link::decode_payload::<link::ScanSolveExecuteCommand>(packet.payload())
        else {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        };
        self.start_scan_operation(
            packet.request_id,
            command.session_id,
            link::OperationKind::ScanSolveExecute,
        )
    }

    fn start_scan_operation(
        &mut self,
        request_id: u32,
        session_id: u32,
        kind: link::OperationKind,
    ) -> ResponseMessage {
        if !self.scanner.available() {
            return self.rejected(request_id, link::RejectionReason::UnsupportedCommand);
        }
        if self.operation_active() {
            return self.rejected(request_id, link::RejectionReason::OperationAlreadyActive);
        }
        if self.status.controller != link::ControllerState::Ready {
            return self.rejected(request_id, link::RejectionReason::InvalidControllerState);
        }
        let Some(session) = self.status.cube_session else {
            return self.rejected(request_id, link::RejectionReason::SessionUnavailable);
        };
        if session.id != session_id {
            return self.rejected(request_id, link::RejectionReason::SessionMismatch);
        }
        if self.status.stand.pose.kind != link::StandPoseKind::CanonicalGrip {
            return self.rejected(request_id, link::RejectionReason::StandPoseMismatch);
        }

        let revision = self.next_scan_revision;
        if self.scanner.begin_scan(revision).is_err() {
            return self.rejected(request_id, link::RejectionReason::InvalidControllerState);
        }
        self.next_scan_revision = self.next_scan_revision.wrapping_add(1).max(1);
        let operation_id = self.next_operation_id;
        self.next_operation_id = self.next_operation_id.wrapping_add(1).max(1);
        let steps = scan_steps();
        let action_count = u16::try_from(steps.len()).expect("scan plan fits u16");
        let operation = link::OperationStatus {
            id: operation_id,
            kind,
            current_action: 0,
            action_count,
        };
        self.status.controller = link::ControllerState::Busy;
        self.status.active_operation = Some(operation);
        self.status.scan = empty_scan();
        self.status.scan.state = link::ScanStateKind::InProgress;
        self.status.scan.revision = Some(revision);
        self.status.solution = empty_solution();
        self.active_scan = Some(ActiveScan {
            operation_id,
            kind,
            steps,
            pending: None,
            deadline: None,
            cube: CubeState::default(),
            failure: None,
        });
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: self.status.active_operation,
            }));
        self.accepted(request_id, Some(operation_id))
    }

    fn start_solve_request(&mut self, packet: &ReceivedPacket) -> ResponseMessage {
        let Ok(command) = link::decode_payload::<link::SolveCommand>(packet.payload()) else {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        };
        if self.operation_active() {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::OperationAlreadyActive,
            );
        }
        if self.status.controller != link::ControllerState::Ready {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::InvalidControllerState,
            );
        }
        let Some(session) = self.status.cube_session else {
            return self.rejected(packet.request_id, link::RejectionReason::SessionUnavailable);
        };
        if session.id != command.session_id {
            return self.rejected(packet.request_id, link::RejectionReason::SessionMismatch);
        }
        if self.status.scan.state != link::ScanStateKind::Valid {
            return self.rejected(packet.request_id, link::RejectionReason::ScanUnavailable);
        }
        if self.status.scan.revision != Some(command.scan_revision) {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::ScanRevisionMismatch,
            );
        }
        if self.status.stand.pose.kind != link::StandPoseKind::CanonicalGrip {
            return self.rejected(packet.request_id, link::RejectionReason::StandPoseMismatch);
        }

        let facelets = match cube_from_scan(&self.status.scan)
            .and_then(|cube| cube.facelet_string())
        {
            Ok(facelets) => facelets,
            Err(error) => {
                let _ = error;
                return self.rejected(packet.request_id, link::RejectionReason::ScanUnavailable);
            }
        };
        let operation_id = self.next_operation_id;
        self.next_operation_id = self.next_operation_id.wrapping_add(1).max(1);
        let operation = link::OperationStatus {
            id: operation_id,
            kind: link::OperationKind::Solve,
            current_action: 0,
            action_count: 1,
        };
        self.status.controller = link::ControllerState::Busy;
        self.status.active_operation = Some(operation);
        self.begin_solver(
            operation_id,
            command.scan_revision,
            link::OperationKind::Solve,
            facelets,
        );
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: self.status.active_operation,
            }));
        self.accepted(packet.request_id, Some(operation_id))
    }

    fn begin_solver(
        &mut self,
        operation_id: u32,
        scan_revision: u32,
        kind: link::OperationKind,
        facelets: String,
    ) {
        let (sender, receiver) = mpsc::channel();
        std::thread::spawn(move || {
            let result = solve_facelets(&facelets, SOLVER_MAX_MOVES)
                .and_then(|solution| parse_solution(&solution))
                .map_err(|error| format!("{error:#}"));
            let _ = sender.send(result);
        });
        self.status.solution = empty_solution();
        self.status.solution.state = link::SolutionStateKind::Solving;
        self.status.solution.source_scan_revision = Some(scan_revision);
        self.active_solve = Some(ActiveSolve {
            operation_id,
            kind,
            scan_revision,
            receiver,
        });
    }

    fn start_execute_request(&mut self, packet: &ReceivedPacket) -> ResponseMessage {
        let Ok(command) = link::decode_payload::<link::ExecuteCommand>(packet.payload()) else {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        };
        if self.operation_active() {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::OperationAlreadyActive,
            );
        }
        if self.status.controller != link::ControllerState::Ready {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::InvalidControllerState,
            );
        }
        let Some(session) = self.status.cube_session else {
            return self.rejected(packet.request_id, link::RejectionReason::SessionUnavailable);
        };
        if session.id != command.session_id {
            return self.rejected(packet.request_id, link::RejectionReason::SessionMismatch);
        }
        if self.status.scan.state != link::ScanStateKind::Valid {
            return self.rejected(packet.request_id, link::RejectionReason::ScanUnavailable);
        }
        if self.status.scan.revision != Some(command.scan_revision) {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::ScanRevisionMismatch,
            );
        }
        if self.status.solution.state != link::SolutionStateKind::Ready {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::SolutionUnavailable,
            );
        }
        if self.status.solution.id != Some(command.solution_id)
            || self.status.solution.source_scan_revision != Some(command.scan_revision)
        {
            return self.rejected(packet.request_id, link::RejectionReason::SolutionMismatch);
        }
        if self.status.stand.pose.kind != link::StandPoseKind::CanonicalGrip {
            return self.rejected(packet.request_id, link::RejectionReason::StandPoseMismatch);
        }

        let moves = self.status.solution.moves[..usize::from(self.status.solution.move_count)]
            .iter()
            .copied()
            .map(internal_move)
            .collect::<Vec<_>>();
        let steps = execute_steps(&moves);
        let action_count = u16::try_from(steps.len()).expect("execute plan fits u16");
        let operation_id = self.next_operation_id;
        self.next_operation_id = self.next_operation_id.wrapping_add(1).max(1);
        let operation = link::OperationStatus {
            id: operation_id,
            kind: link::OperationKind::Execute,
            current_action: 0,
            action_count,
        };
        self.status.controller = link::ControllerState::Busy;
        self.status.active_operation = Some(operation);
        self.status.solution.state = link::SolutionStateKind::Executing;
        self.status.solution.completed_moves = 0;
        self.active_execute = Some(ActiveExecute {
            operation_id,
            kind: link::OperationKind::Execute,
            steps,
            pending: None,
            deadline: None,
        });
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: self.status.active_operation,
            }));
        self.accepted(packet.request_id, Some(operation_id))
    }

    fn start_execute_moves_request(&mut self, packet: &ReceivedPacket) -> ResponseMessage {
        let Ok(command) = link::decode_payload::<link::ExecuteMovesCommand>(packet.payload())
        else {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        };
        if command.validate().is_err() || command.move_count == 0 {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        }
        if self.operation_active() {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::OperationAlreadyActive,
            );
        }
        if self.status.controller != link::ControllerState::Ready {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::InvalidControllerState,
            );
        }
        let Some(session) = self.status.cube_session else {
            return self.rejected(packet.request_id, link::RejectionReason::SessionUnavailable);
        };
        if session.id != command.session_id {
            return self.rejected(packet.request_id, link::RejectionReason::SessionMismatch);
        }
        if self.status.stand.pose.kind != link::StandPoseKind::CanonicalGrip {
            return self.rejected(packet.request_id, link::RejectionReason::StandPoseMismatch);
        }

        let moves = command.moves[..usize::from(command.move_count)]
            .iter()
            .copied()
            .map(internal_move)
            .collect::<Vec<_>>();
        let steps = held_execute_steps(&moves);
        let action_count = u16::try_from(steps.len()).expect("manual execute plan fits u16");
        let operation_id = self.next_operation_id;
        self.next_operation_id = self.next_operation_id.wrapping_add(1).max(1);
        let operation = link::OperationStatus {
            id: operation_id,
            kind: link::OperationKind::ExecuteMoves,
            current_action: 0,
            action_count,
        };
        self.status.controller = link::ControllerState::Busy;
        self.status.active_operation = Some(operation);
        self.active_execute = Some(ActiveExecute {
            operation_id,
            kind: link::OperationKind::ExecuteMoves,
            steps,
            pending: None,
            deadline: None,
        });
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: self.status.active_operation,
            }));
        self.accepted(packet.request_id, Some(operation_id))
    }

    fn start_open_request(&mut self, packet: &ReceivedPacket) -> ResponseMessage {
        let Ok(command) = link::decode_payload::<link::OpenCommand>(packet.payload()) else {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        };
        if self.operation_active() {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::OperationAlreadyActive,
            );
        }
        if self.status.controller != link::ControllerState::Ready {
            return self.rejected(
                packet.request_id,
                link::RejectionReason::InvalidControllerState,
            );
        }
        let Some(session) = self.status.cube_session else {
            return self.rejected(packet.request_id, link::RejectionReason::SessionUnavailable);
        };
        if session.id != command.session_id {
            return self.rejected(packet.request_id, link::RejectionReason::SessionMismatch);
        }
        if self.status.stand.pose.kind != link::StandPoseKind::CanonicalGrip {
            return self.rejected(packet.request_id, link::RejectionReason::StandPoseMismatch);
        }

        let steps = open_steps();
        let operation_id = self.next_operation_id;
        self.next_operation_id = self.next_operation_id.wrapping_add(1).max(1);
        let operation = link::OperationStatus {
            id: operation_id,
            kind: link::OperationKind::Open,
            current_action: 0,
            action_count: steps.len() as u16,
        };
        self.status.controller = link::ControllerState::Busy;
        self.status.active_operation = Some(operation);
        self.active_execute = Some(ActiveExecute {
            operation_id,
            kind: link::OperationKind::Open,
            steps,
            pending: None,
            deadline: None,
        });
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: self.status.active_operation,
            }));
        self.accepted(packet.request_id, Some(operation_id))
    }

    fn operation_active(&self) -> bool {
        self.active_motion.is_some()
            || self.active_scan.is_some()
            || self.active_solve.is_some()
            || self.active_execute.is_some()
    }

    fn start_empty_payload_operation(
        &mut self,
        packet: &ReceivedPacket,
        now: Instant,
        kind: link::OperationKind,
    ) -> ResponseMessage {
        if !packet.payload().is_empty() {
            return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
        }

        match self.start_operation(kind, now) {
            Ok(operation_id) => self.accepted(packet.request_id, Some(operation_id)),
            Err(reason) => self.rejected(packet.request_id, reason),
        }
    }

    fn start_operation(
        &mut self,
        kind: link::OperationKind,
        _now: Instant,
    ) -> Result<u32, link::RejectionReason> {
        if self.operation_active() {
            return Err(link::RejectionReason::OperationAlreadyActive);
        }

        let phase = match kind {
            link::OperationKind::RecoverToOpen => MotionPhase::RecoverStart,
            link::OperationKind::Grip => {
                if self.status.controller != link::ControllerState::Ready {
                    return Err(link::RejectionReason::InvalidControllerState);
                }
                if self.status.stand.pose.kind != link::StandPoseKind::Open {
                    return Err(
                        if self.status.stand.pose.kind == link::StandPoseKind::Unknown {
                            link::RejectionReason::StandPositionUnknown
                        } else {
                            link::RejectionReason::StandPoseMismatch
                        },
                    );
                }
                if self.status.cube_session.is_some() {
                    return Err(link::RejectionReason::SessionUnavailable);
                }
                MotionPhase::GripStart
            }
            _ => return Err(link::RejectionReason::UnsupportedCommand),
        };

        let operation_id = self.next_operation_id;
        self.next_operation_id = self.next_operation_id.wrapping_add(1).max(1);
        let action_count = match kind {
            link::OperationKind::RecoverToOpen => 3,
            link::OperationKind::Grip => 2,
            _ => 0,
        };
        let operation = link::OperationStatus {
            id: operation_id,
            kind,
            current_action: 0,
            action_count,
        };
        self.status.controller = link::ControllerState::Busy;
        self.status.active_operation = Some(operation);
        self.status.fault = None;
        self.active_motion = Some(ActiveMotion {
            operation_id,
            kind,
            phase,
            deadline: None,
        });
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: self.status.active_operation,
            }));
        Ok(operation_id)
    }

    fn advance_motion(&mut self, now: Instant) {
        let Some(active) = self.active_motion else {
            return;
        };

        let result = match active.phase {
            MotionPhase::RecoverStart => self.recover_start(now),
            MotionPhase::RecoverFinishLeftRight => self.recover_finish_left_right(now),
            MotionPhase::RecoverFinishTopBottom => self.recover_finish_top_bottom(now),
            MotionPhase::RecoverFinishGrippers => self.recover_finish_grippers(),
            MotionPhase::GripStart => self.grip_start(now),
            MotionPhase::GripFinishGrippers => self.grip_finish_grippers(now),
            MotionPhase::GripFinishRails => self.grip_finish_rails(),
        };

        if let Err(error) = result {
            self.enter_fault(active.operation_id, error);
        }
    }

    fn advance_scan(&mut self, now: Instant) {
        let Some(mut scan) = self.active_scan.take() else {
            return;
        };
        if scan.deadline.is_some_and(|deadline| now < deadline) {
            self.active_scan = Some(scan);
            return;
        }

        if let Some(pending) = scan.pending.take() {
            let result = match pending {
                PendingMotionStep::Rails(target, position) => {
                    self.finish_motion_rails(target, position)
                }
                PendingMotionStep::Grippers(poses) => {
                    self.finish_scan_grippers(&poses);
                    Ok(())
                }
            };
            if let Err(error) = result {
                self.enter_fault(scan.operation_id, error);
                return;
            }
            scan.deadline = None;
            self.advance_action_counter();
            self.emit_stand();
            // Preserve the stable endpoint as an observable protocol state.
            // Starting the next motion in this same tick collapses
            // `Moving -> Stable -> Moving` into `Moving -> Moving`, which
            // makes consumers miss the completed physical step entirely.
            self.active_scan = Some(scan);
            return;
        }

        let Some(step) = scan.steps.pop_front() else {
            self.finish_scan_operation(scan);
            return;
        };

        let result = match step {
            MotionStep::SetRails(target, position) => {
                self.status.stand.pose = transitional_pose();
                self.command_motion_rails(target.clone(), position)
                    .map(|()| {
                        scan.pending = Some(PendingMotionStep::Rails(target, position));
                        scan.deadline = Some(now + self.calibration.rail_duration(position));
                    })
            }
            MotionStep::SetGrippers(poses) => {
                self.status.stand.pose = transitional_pose();
                let duration = self.gripper_motion_duration(&poses);
                self.command_scan_grippers(&poses).map(|()| {
                    scan.pending = Some(PendingMotionStep::Grippers(poses));
                    scan.deadline = Some(now + duration);
                })
            }
            MotionStep::Capture(face) => {
                self.status.stand.pose = link::StandPose {
                    kind: link::StandPoseKind::ScanPose,
                    camera_face: Some(face),
                };
                self.status.scan.current_face = Some(face);
                if scan.failure.is_none() {
                    match self.scanner.capture(face) {
                        Ok(recognized) => {
                            if let Err(error) =
                                record_recognized_face(&mut scan.cube, face, recognized)
                            {
                                let _ = error;
                                scan.failure = Some(link::OperationFailureKind::Recognition);
                                self.status.scan.validation_error =
                                    Some(link::ScanValidationError::InvalidFacelet);
                            } else {
                                self.status.scan.camera_face = Some(recognized);
                                self.status.scan.faces[face as usize] = Some(recognized);
                                self.status.scan.scanned_faces |= 1 << face as u8;
                                add_color_counts(&mut self.status.scan.color_counts, recognized);
                                self.events.push_back(EventMessage::FaceScanned(
                                    link::FaceScanned {
                                        operation_id: scan.operation_id,
                                        face,
                                        recognized,
                                        scanned_faces: self.status.scan.scanned_faces,
                                    },
                                ));
                            }
                        }
                        Err(error) => {
                            let _ = error;
                            scan.failure = Some(link::OperationFailureKind::Recognition);
                            self.status.scan.validation_error =
                                Some(link::ScanValidationError::InferenceFailure);
                        }
                    }
                }
                self.advance_action_counter();
                self.emit_stand();
                Ok(())
            }
            MotionStep::MoveCompleted | MotionStep::AllOff => {
                Err(anyhow::anyhow!("invalid non-scan action in scan plan"))
            }
        };

        if let Err(error) = result {
            self.enter_fault(scan.operation_id, error);
        } else {
            self.active_scan = Some(scan);
        }
    }

    fn command_motion_rails(
        &mut self,
        target: RailTarget,
        position: RailPosition,
    ) -> anyhow::Result<()> {
        match target {
            RailTarget::Pair(pair) => {
                let (physical, logical) = rail_pair_axes(pair);
                self.command_rail_pair(physical, logical[0], logical[1], position)
            }
            RailTarget::Single(axis) => {
                self.output.set_channels(&[(
                    axis.channel(),
                    self.calibration.rail_pulse(axis, position),
                )])?;
                let rail = &mut self.status.stand.rails[stand_axis_index(axis)];
                rail.motion = link::AxisMotion::Moving;
                rail.target = Some(link_rail_position(position));
                self.status.stand.outputs_enabled = true;
                Ok(())
            }
        }
    }

    fn finish_motion_rails(
        &mut self,
        target: RailTarget,
        position: RailPosition,
    ) -> anyhow::Result<()> {
        match target {
            RailTarget::Pair(pair) => {
                let (physical, logical) = rail_pair_axes(pair);
                self.output
                    .disable_channels(&physical.map(StandAxis::channel))?;
                self.finish_rails(logical, link_rail_position(position));
            }
            RailTarget::Single(axis) => {
                self.output.disable_channels(&[axis.channel()])?;
                let rail = &mut self.status.stand.rails[stand_axis_index(axis)];
                rail.motion = link::AxisMotion::Stable;
                rail.current = Some(link_rail_position(position));
                rail.target = None;
            }
        }
        Ok(())
    }

    fn gripper_motion_duration(&self, poses: &[(StandAxis, GripperOrientation)]) -> Duration {
        let orientation_index = |orientation: link::GripperOrientation| match orientation {
            link::GripperOrientation::FrameParallel => 0i32,
            link::GripperOrientation::FramePerpendicular => 1,
            link::GripperOrientation::FrameParallelReversed => 2,
        };
        let quarter_turns = poses
            .iter()
            .filter_map(|&(axis, target)| {
                let current = self.status.stand.grippers[stand_axis_index(axis)].current?;
                Some(
                    (orientation_index(link_gripper_orientation(target))
                        - orientation_index(current))
                    .unsigned_abs(),
                )
            })
            .max()
            .unwrap_or(1)
            .max(1);
        self.calibration.gripper_pose_duration() * quarter_turns
    }

    fn command_scan_grippers(
        &mut self,
        poses: &[(StandAxis, GripperOrientation)],
    ) -> anyhow::Result<()> {
        let channels = poses
            .iter()
            .copied()
            .map(|(axis, orientation)| {
                self.calibration
                    .gripper_pulse(axis, orientation)
                    .map(|pulse| (axis.channel(), pulse))
                    .ok_or_else(|| anyhow::anyhow!("missing {} calibration", axis.name()))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        self.output.set_channels(&channels)?;
        for &(axis, orientation) in poses {
            let gripper = &mut self.status.stand.grippers[stand_axis_index(axis)];
            gripper.motion = link::AxisMotion::Moving;
            gripper.target = Some(link_gripper_orientation(orientation));
        }
        self.status.stand.outputs_enabled = true;
        Ok(())
    }

    fn finish_scan_grippers(&mut self, poses: &[(StandAxis, GripperOrientation)]) {
        for &(axis, orientation) in poses {
            let gripper = &mut self.status.stand.grippers[stand_axis_index(axis)];
            gripper.motion = link::AxisMotion::Stable;
            gripper.current = Some(link_gripper_orientation(orientation));
            gripper.target = None;
        }
    }

    fn advance_action_counter(&mut self) {
        if let Some(operation) = self.status.active_operation.as_mut() {
            operation.current_action = operation.current_action.saturating_add(1);
        }
    }

    fn finish_scan_operation(&mut self, scan: ActiveScan) {
        let facelets = if scan.failure.is_none() {
            scan.cube.facelet_string().ok()
        } else {
            None
        };
        let failure = scan.failure.or_else(|| {
            facelets
                .is_none()
                .then_some(link::OperationFailureKind::InvalidFacelet)
        });
        self.status.scan.current_face = None;
        self.status.scan.state = if failure.is_some() {
            link::ScanStateKind::Invalid
        } else {
            link::ScanStateKind::Valid
        };
        if matches!(failure, Some(link::OperationFailureKind::InvalidFacelet)) {
            self.status.scan.validation_error = Some(link::ScanValidationError::InvalidFacelet);
        }
        self.status.stand.pose = link::StandPose {
            kind: link::StandPoseKind::CanonicalGrip,
            camera_face: Some(link::CubeFace::Front),
        };
        let _ = self.scanner.finish_scan(&self.status.scan);
        self.emit_stand();
        match failure {
            Some(kind) => {
                self.status.controller = link::ControllerState::Ready;
                self.status.active_operation = None;
                self.events
                    .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                        controller: self.status.controller,
                        active_operation: None,
                    }));
                self.events
                    .push_back(EventMessage::OperationFailed(link::OperationFailed {
                        operation_id: scan.operation_id,
                        kind,
                    }))
            }
            None if scan.kind == link::OperationKind::ScanSolveExecute => {
                let revision = self
                    .status
                    .scan
                    .revision
                    .expect("completed scan has a revision");
                if let Some(operation) = self.status.active_operation.as_mut() {
                    operation.current_action = operation.action_count;
                    operation.action_count = operation.action_count.saturating_add(1);
                }
                self.begin_solver(
                    scan.operation_id,
                    revision,
                    scan.kind,
                    facelets.expect("valid scan has facelets"),
                );
                self.events
                    .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                        controller: self.status.controller,
                        active_operation: self.status.active_operation,
                    }));
            }
            None => {
                self.status.controller = link::ControllerState::Ready;
                self.status.active_operation = None;
                self.events
                    .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                        controller: self.status.controller,
                        active_operation: None,
                    }));
                self.events
                    .push_back(EventMessage::OperationCompleted(link::OperationCompleted {
                        operation_id: scan.operation_id,
                        kind: scan.kind,
                    }))
            }
        }
    }

    fn advance_solve(&mut self) {
        let Some(active) = self.active_solve.as_ref() else {
            return;
        };
        let result = match active.receiver.try_recv() {
            Ok(result) => result,
            Err(mpsc::TryRecvError::Empty) => return,
            Err(mpsc::TryRecvError::Disconnected) => {
                Err("solver worker disconnected without a result".to_owned())
            }
        };
        let active = self.active_solve.take().expect("active solve exists");

        match result {
            Ok(moves) => {
                if moves.len() > link::MAX_SOLUTION_MOVES {
                    self.finish_solve_failure(active.operation_id);
                    return;
                }
                let execute_plan = (active.kind == link::OperationKind::ScanSolveExecute)
                    .then(|| execute_steps(&moves));
                let solution_id = self.next_solution_id;
                self.next_solution_id = self.next_solution_id.wrapping_add(1).max(1);
                let mut solution = empty_solution();
                solution.state = link::SolutionStateKind::Ready;
                solution.id = Some(solution_id);
                solution.source_scan_revision = Some(active.scan_revision);
                solution.move_count = moves.len() as u8;
                for (destination, cube_move) in solution.moves.iter_mut().zip(moves.iter().copied())
                {
                    *destination = protocol_move(cube_move);
                }
                self.status.solution = solution;
                if let Some(steps) = execute_plan {
                    self.status.solution.state = link::SolutionStateKind::Executing;
                    self.status.solution.completed_moves = 0;
                    if let Some(operation) = self.status.active_operation.as_mut() {
                        operation.current_action = operation.current_action.saturating_add(1);
                        operation.action_count =
                            operation.current_action.saturating_add(steps.len() as u16);
                    }
                    self.active_execute = Some(ActiveExecute {
                        operation_id: active.operation_id,
                        kind: active.kind,
                        steps,
                        pending: None,
                        deadline: None,
                    });
                    self.events.push_back(EventMessage::RobotStateChanged(
                        link::RobotStateChanged {
                            controller: self.status.controller,
                            active_operation: self.status.active_operation,
                        },
                    ));
                } else {
                    self.status.controller = link::ControllerState::Ready;
                    self.status.active_operation = None;
                    self.events.push_back(EventMessage::RobotStateChanged(
                        link::RobotStateChanged {
                            controller: self.status.controller,
                            active_operation: None,
                        },
                    ));
                    self.events.push_back(EventMessage::OperationCompleted(
                        link::OperationCompleted {
                            operation_id: active.operation_id,
                            kind: active.kind,
                        },
                    ));
                }
            }
            Err(error) => {
                let _ = error;
                self.finish_solve_failure(active.operation_id);
            }
        }
    }

    fn finish_solve_failure(&mut self, operation_id: u32) {
        self.status.solution = empty_solution();
        self.status.controller = link::ControllerState::Ready;
        self.status.active_operation = None;
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: None,
            }));
        self.events
            .push_back(EventMessage::OperationFailed(link::OperationFailed {
                operation_id,
                kind: link::OperationFailureKind::SolverNoSolution,
            }));
    }

    fn advance_execute(&mut self, now: Instant) {
        let Some(mut execute) = self.active_execute.take() else {
            return;
        };
        if execute.deadline.is_some_and(|deadline| now < deadline) {
            self.active_execute = Some(execute);
            return;
        }

        if let Some(pending) = execute.pending.take() {
            let result = match pending {
                PendingMotionStep::Rails(target, position) => {
                    self.finish_motion_rails(target, position)
                }
                PendingMotionStep::Grippers(poses) => {
                    self.finish_scan_grippers(&poses);
                    Ok(())
                }
            };
            if let Err(error) = result {
                self.enter_fault(execute.operation_id, error);
                return;
            }
            execute.deadline = None;
            self.advance_action_counter();
            self.emit_stand();
            // Do not collapse a completed actuator step and the next command
            // into one status update. Visualisation and hardware monitoring
            // both need to observe the stable endpoint.
            self.active_execute = Some(execute);
            return;
        }

        let Some(step) = execute.steps.pop_front() else {
            self.finish_execute_operation(execute.operation_id, execute.kind);
            return;
        };
        let move_pose = if execute.kind == link::OperationKind::Open {
            transitional_pose()
        } else {
            link::StandPose {
                kind: link::StandPoseKind::MovePose,
                camera_face: Some(link::CubeFace::Front),
            }
        };
        let result = match step {
            MotionStep::SetRails(target, position) => {
                self.status.stand.pose = move_pose;
                self.command_motion_rails(target.clone(), position)
                    .map(|()| {
                        execute.pending = Some(PendingMotionStep::Rails(target, position));
                        execute.deadline = Some(now + self.calibration.rail_duration(position));
                    })
            }
            MotionStep::SetGrippers(poses) => {
                self.status.stand.pose = move_pose;
                let duration = self.gripper_motion_duration(&poses);
                self.command_scan_grippers(&poses).map(|()| {
                    execute.pending = Some(PendingMotionStep::Grippers(poses));
                    execute.deadline = Some(now + duration);
                })
            }
            MotionStep::MoveCompleted => {
                if execute.kind == link::OperationKind::ExecuteMoves {
                    self.clear_scan_and_solution();
                } else {
                    self.status.solution.completed_moves =
                        self.status.solution.completed_moves.saturating_add(1);
                }
                self.status.stand.pose = link::StandPose {
                    kind: link::StandPoseKind::MovePose,
                    camera_face: None,
                };
                self.advance_action_counter();
                self.emit_stand();
                Ok(())
            }
            MotionStep::AllOff => self.output.all_off().map(|()| {
                self.status.stand.outputs_enabled = false;
                self.advance_action_counter();
            }),
            MotionStep::Capture(_) => Err(anyhow::anyhow!("capture action in execute plan")),
        };

        if let Err(error) = result {
            self.enter_fault(execute.operation_id, error);
        } else {
            self.active_execute = Some(execute);
        }
    }

    fn finish_execute_operation(&mut self, operation_id: u32, kind: link::OperationKind) {
        self.status.controller = link::ControllerState::Ready;
        self.status.active_operation = None;
        if kind == link::OperationKind::ExecuteMoves {
            self.status.stand.pose = link::StandPose {
                kind: link::StandPoseKind::CanonicalGrip,
                camera_face: Some(link::CubeFace::Front),
            };
            self.events
                .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                    controller: self.status.controller,
                    active_operation: None,
                }));
            self.emit_stand();
            self.events
                .push_back(EventMessage::OperationCompleted(link::OperationCompleted {
                    operation_id,
                    kind,
                }));
            return;
        }
        self.status.stand.pose = link::StandPose {
            kind: link::StandPoseKind::Open,
            camera_face: None,
        };
        self.status.stand.outputs_enabled = false;
        self.status.cube_session = None;
        self.clear_scan_and_solution();
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: None,
            }));
        self.emit_stand();
        self.events
            .push_back(EventMessage::CubeSessionChanged(link::CubeSessionChanged {
                session: None,
            }));
        self.events
            .push_back(EventMessage::OperationCompleted(link::OperationCompleted {
                operation_id,
                kind,
            }));
    }

    fn recover_start(&mut self, now: Instant) -> anyhow::Result<()> {
        self.status.cube_session = None;
        self.clear_scan_and_solution();
        self.status.stand.pose = transitional_pose();
        self.command_rail_pair(
            [StandAxis::LeftRail, StandAxis::RightRail],
            link::Axis::Left,
            link::Axis::Right,
            RailPosition::FarOpen,
        )?;
        self.set_motion_phase(
            MotionPhase::RecoverFinishLeftRight,
            now + self.calibration.rail_duration(RailPosition::FarOpen),
            0,
        );
        self.emit_stand();
        Ok(())
    }

    fn recover_finish_left_right(&mut self, now: Instant) -> anyhow::Result<()> {
        self.output.disable_channels(&[
            StandAxis::LeftRail.channel(),
            StandAxis::RightRail.channel(),
        ])?;
        self.finish_rails(
            [link::Axis::Left, link::Axis::Right],
            link::RailPosition::Open,
        );
        self.command_rail_pair(
            [StandAxis::TopRail, StandAxis::BottomRail],
            link::Axis::Top,
            link::Axis::Bottom,
            RailPosition::FarOpen,
        )?;
        self.set_motion_phase(
            MotionPhase::RecoverFinishTopBottom,
            now + self.calibration.rail_duration(RailPosition::FarOpen),
            1,
        );
        self.emit_stand();
        Ok(())
    }

    fn recover_finish_top_bottom(&mut self, now: Instant) -> anyhow::Result<()> {
        self.output.disable_channels(&[
            StandAxis::TopRail.channel(),
            StandAxis::BottomRail.channel(),
        ])?;
        self.finish_rails(
            [link::Axis::Top, link::Axis::Bottom],
            link::RailPosition::Open,
        );
        self.command_all_grippers_perpendicular()?;
        self.set_motion_phase(
            MotionPhase::RecoverFinishGrippers,
            now + self.calibration.gripper_pose_duration(),
            2,
        );
        self.emit_stand();
        Ok(())
    }

    fn recover_finish_grippers(&mut self) -> anyhow::Result<()> {
        self.output.all_off()?;
        self.finish_all_grippers_perpendicular();
        self.status.stand.outputs_enabled = false;
        self.status.stand.pose = link::StandPose {
            kind: link::StandPoseKind::Open,
            camera_face: None,
        };
        self.complete_operation();
        Ok(())
    }

    fn grip_start(&mut self, now: Instant) -> anyhow::Result<()> {
        self.status.stand.pose = transitional_pose();
        self.command_all_grippers_perpendicular()?;
        self.set_motion_phase(
            MotionPhase::GripFinishGrippers,
            now + self.calibration.gripper_pose_duration(),
            0,
        );
        self.emit_stand();
        Ok(())
    }

    fn grip_finish_grippers(&mut self, now: Instant) -> anyhow::Result<()> {
        self.finish_all_grippers_perpendicular();
        let channels = StandAxis::RAILS
            .into_iter()
            .map(|axis| {
                (
                    axis.channel(),
                    self.calibration.rail_pulse(axis, RailPosition::NearGrip),
                )
            })
            .collect::<Vec<_>>();
        self.output.set_channels(&channels)?;
        for axis in [
            link::Axis::Left,
            link::Axis::Right,
            link::Axis::Top,
            link::Axis::Bottom,
        ] {
            let rail = &mut self.status.stand.rails[axis_index(axis)];
            rail.motion = link::AxisMotion::Moving;
            rail.target = Some(link::RailPosition::Grip);
        }
        self.status.stand.outputs_enabled = true;
        self.set_motion_phase(
            MotionPhase::GripFinishRails,
            now + self.calibration.rail_duration(RailPosition::NearGrip),
            1,
        );
        self.emit_stand();
        Ok(())
    }

    fn grip_finish_rails(&mut self) -> anyhow::Result<()> {
        let rail_channels = StandAxis::RAILS.map(StandAxis::channel);
        self.output.disable_channels(&rail_channels)?;
        self.finish_rails(
            [link::Axis::Left, link::Axis::Right],
            link::RailPosition::Grip,
        );
        self.finish_rails(
            [link::Axis::Top, link::Axis::Bottom],
            link::RailPosition::Grip,
        );
        self.status.stand.pose = link::StandPose {
            kind: link::StandPoseKind::CanonicalGrip,
            camera_face: Some(link::CubeFace::Front),
        };
        let session = link::CubeSessionStatus {
            id: self.next_session_id,
        };
        self.next_session_id = self.next_session_id.wrapping_add(1).max(1);
        self.status.cube_session = Some(session);
        self.events
            .push_back(EventMessage::CubeSessionChanged(link::CubeSessionChanged {
                session: Some(session),
            }));
        self.complete_operation();
        Ok(())
    }

    fn command_rail_pair(
        &mut self,
        physical: [StandAxis; 2],
        first: link::Axis,
        second: link::Axis,
        position: RailPosition,
    ) -> anyhow::Result<()> {
        let channels =
            physical.map(|axis| (axis.channel(), self.calibration.rail_pulse(axis, position)));
        self.output.set_channels(&channels)?;
        let target = match position {
            RailPosition::FarOpen => link::RailPosition::Open,
            RailPosition::NearGrip => link::RailPosition::Grip,
        };
        for axis in [first, second] {
            let rail = &mut self.status.stand.rails[axis_index(axis)];
            rail.motion = link::AxisMotion::Moving;
            rail.target = Some(target);
        }
        self.status.stand.outputs_enabled = true;
        Ok(())
    }

    fn command_all_grippers_perpendicular(&mut self) -> anyhow::Result<()> {
        let channels = [
            StandAxis::LeftGripper,
            StandAxis::RightGripper,
            StandAxis::TopGripper,
            StandAxis::BottomGripper,
        ]
        .into_iter()
        .map(|axis| {
            self.calibration
                .gripper_pulse(axis, GripperOrientation::FramePerpendicular)
                .map(|pulse| (axis.channel(), pulse))
                .ok_or_else(|| {
                    anyhow::anyhow!("missing perpendicular calibration for {}", axis.name())
                })
        })
        .collect::<anyhow::Result<Vec<_>>>()?;
        self.output.set_channels(&channels)?;
        for gripper in &mut self.status.stand.grippers {
            gripper.motion = link::AxisMotion::Moving;
            gripper.target = Some(link::GripperOrientation::FramePerpendicular);
        }
        self.status.stand.outputs_enabled = true;
        Ok(())
    }

    fn finish_all_grippers_perpendicular(&mut self) {
        for gripper in &mut self.status.stand.grippers {
            gripper.motion = link::AxisMotion::Stable;
            gripper.current = Some(link::GripperOrientation::FramePerpendicular);
            gripper.target = None;
        }
    }

    fn finish_rails(&mut self, axes: [link::Axis; 2], position: link::RailPosition) {
        for axis in axes {
            let rail = &mut self.status.stand.rails[axis_index(axis)];
            rail.motion = link::AxisMotion::Stable;
            rail.current = Some(position);
            rail.target = None;
        }
    }

    fn set_motion_phase(&mut self, phase: MotionPhase, deadline: Instant, current_action: u16) {
        let active = self.active_motion.as_mut().expect("active motion exists");
        active.phase = phase;
        active.deadline = Some(deadline);
        if let Some(operation) = self.status.active_operation.as_mut() {
            operation.current_action = current_action;
        }
    }

    fn complete_operation(&mut self) {
        let active = self.active_motion.take().expect("active motion exists");
        self.status.controller = link::ControllerState::Ready;
        self.status.active_operation = None;
        self.emit_stand();
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: None,
            }));
        self.events
            .push_back(EventMessage::OperationCompleted(link::OperationCompleted {
                operation_id: active.operation_id,
                kind: active.kind,
            }));
    }

    fn abort(&mut self, operation_id: Option<u32>) {
        self.scanner.abort();
        match self.output.all_off() {
            Ok(()) => {
                self.active_motion = None;
                self.active_scan = None;
                self.active_solve = None;
                self.active_execute = None;
                self.status.controller = link::ControllerState::Aborted;
                self.status.active_operation = None;
                self.status.stand = unknown_stand();
                self.status.cube_session = None;
                self.clear_scan_and_solution();
                self.events
                    .push_back(EventMessage::Aborted(link::Aborted { operation_id }));
                self.events
                    .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                        controller: self.status.controller,
                        active_operation: None,
                    }));
                self.emit_stand();
                self.events
                    .push_back(EventMessage::CubeSessionChanged(link::CubeSessionChanged {
                        session: None,
                    }));
            }
            Err(error) => self.enter_fault(
                operation_id.unwrap_or(0),
                error.context("failed to disable outputs during abort"),
            ),
        }
    }

    fn enter_fault(&mut self, operation_id: u32, error: anyhow::Error) {
        self.scanner.abort();
        let _ = self.output.all_off();
        self.active_motion = None;
        self.active_scan = None;
        self.active_solve = None;
        self.active_execute = None;
        self.status.controller = link::ControllerState::Faulted;
        self.status.active_operation = None;
        self.status.stand = unknown_stand();
        self.status.cube_session = None;
        self.clear_scan_and_solution();
        let fault = link::RobotFault {
            code: link::FaultCode::I2c,
            detail: 0,
        };
        self.status.fault = Some(fault);
        self.events.push_back(EventMessage::Fault(link::FaultEvent {
            operation_id: (operation_id != 0).then_some(operation_id),
            fault,
        }));
        self.events
            .push_back(EventMessage::RobotStateChanged(link::RobotStateChanged {
                controller: self.status.controller,
                active_operation: None,
            }));
        self.emit_stand();
        let _ = error;
    }

    fn clear_scan_and_solution(&mut self) {
        self.status.scan = empty_scan();
        self.status.solution = empty_solution();
    }

    fn emit_stand(&mut self) {
        self.events
            .push_back(EventMessage::StandStateChanged(link::StandStateChanged {
                stand: self.status.stand,
            }));
    }

    fn drain_events(&mut self) -> impl Iterator<Item = EventMessage> + '_ {
        self.events.drain(..)
    }

    fn accepted(&self, request_id: u32, operation_id: Option<u32>) -> ResponseMessage {
        ResponseMessage {
            request_id,
            opcode: link::ResponseOpcode::CommandAccepted,
            payload: ResponsePayload::Accepted(link::CommandAccepted { operation_id }),
        }
    }

    fn rejected(&self, request_id: u32, reason: link::RejectionReason) -> ResponseMessage {
        ResponseMessage {
            request_id,
            opcode: link::ResponseOpcode::CommandRejected,
            payload: ResponsePayload::Rejected(link::CommandRejected {
                reason,
                controller: self.status.controller,
            }),
        }
    }

    fn cache_response(&mut self, request_opcode: u16, response: &ResponseMessage) {
        if response.request_id == 0 {
            return;
        }
        if self.request_cache.len() == REQUEST_CACHE_CAPACITY {
            self.request_cache.pop_front();
        }
        self.request_cache.push_back(CachedResponse {
            request_opcode,
            response: response.clone(),
        });
    }
}

fn axis_index(axis: link::Axis) -> usize {
    axis as usize
}

fn stand_axis_index(axis: StandAxis) -> usize {
    match axis {
        StandAxis::LeftRail | StandAxis::LeftGripper => link::Axis::Left as usize,
        StandAxis::RightRail | StandAxis::RightGripper => link::Axis::Right as usize,
        StandAxis::TopRail | StandAxis::TopGripper => link::Axis::Top as usize,
        StandAxis::BottomRail | StandAxis::BottomGripper => link::Axis::Bottom as usize,
    }
}

fn rail_pair_axes(pair: RailPair) -> ([StandAxis; 2], [link::Axis; 2]) {
    match pair {
        RailPair::LeftRight => (
            [StandAxis::LeftRail, StandAxis::RightRail],
            [link::Axis::Left, link::Axis::Right],
        ),
        RailPair::TopBottom => (
            [StandAxis::TopRail, StandAxis::BottomRail],
            [link::Axis::Top, link::Axis::Bottom],
        ),
    }
}

fn link_gripper_orientation(value: GripperOrientation) -> link::GripperOrientation {
    match value {
        GripperOrientation::FrameParallel => link::GripperOrientation::FrameParallel,
        GripperOrientation::FramePerpendicular => link::GripperOrientation::FramePerpendicular,
        GripperOrientation::FrameParallelReversed => {
            link::GripperOrientation::FrameParallelReversed
        }
    }
}

fn link_rail_position(value: RailPosition) -> link::RailPosition {
    match value {
        RailPosition::FarOpen => link::RailPosition::Open,
        RailPosition::NearGrip => link::RailPosition::Grip,
    }
}

fn scan_steps() -> VecDeque<MotionStep> {
    use GripperOrientation::{
        FrameParallel as P, FrameParallelReversed as R, FramePerpendicular as X,
    };
    use RailPair::{LeftRight as LR, TopBottom as TB};
    use StandAxis::{BottomGripper as B, LeftGripper as L, RightGripper as Rg, TopGripper as T};

    VecDeque::from([
        MotionStep::SetRails(RailTarget::Pair(LR), RailPosition::FarOpen),
        MotionStep::SetGrippers(vec![(T, P), (B, R)]),
        MotionStep::Capture(link::CubeFace::Left),
        MotionStep::SetGrippers(vec![(T, X), (B, X)]),
        MotionStep::SetGrippers(vec![(T, R), (B, P)]),
        MotionStep::Capture(link::CubeFace::Right),
        MotionStep::SetGrippers(vec![(T, X), (B, X)]),
        MotionStep::SetRails(RailTarget::Pair(LR), RailPosition::NearGrip),
        MotionStep::SetRails(RailTarget::Pair(TB), RailPosition::FarOpen),
        MotionStep::SetGrippers(vec![(L, P), (Rg, R)]),
        MotionStep::Capture(link::CubeFace::Down),
        MotionStep::SetGrippers(vec![(L, X), (Rg, X)]),
        MotionStep::SetGrippers(vec![(L, R), (Rg, P)]),
        MotionStep::Capture(link::CubeFace::Up),
        MotionStep::SetGrippers(vec![(L, X), (Rg, X)]),
        MotionStep::SetGrippers(vec![(T, P), (B, R)]),
        MotionStep::SetRails(RailTarget::Pair(TB), RailPosition::NearGrip),
        MotionStep::SetRails(RailTarget::Pair(LR), RailPosition::FarOpen),
        MotionStep::Capture(link::CubeFace::Front),
        MotionStep::SetGrippers(vec![(T, R), (B, P)]),
        MotionStep::Capture(link::CubeFace::Back),
        // B -> F and collision-safe hand-off back to canonical grip.
        MotionStep::SetGrippers(vec![(T, P), (B, R)]),
        MotionStep::SetRails(RailTarget::Pair(LR), RailPosition::NearGrip),
        MotionStep::SetRails(RailTarget::Pair(TB), RailPosition::FarOpen),
        MotionStep::SetGrippers(vec![(T, X), (B, X)]),
        MotionStep::SetRails(RailTarget::Pair(TB), RailPosition::NearGrip),
    ])
}

fn record_recognized_face(
    cube: &mut CubeState,
    logical: link::CubeFace,
    recognized: link::RecognizedFace,
) -> anyhow::Result<()> {
    let symbols = recognized
        .colors
        .into_iter()
        .map(protocol_color_symbol)
        .collect::<String>();
    cube.record_scan(
        ScanPose {
            face: logical_face(logical),
            camera_to_face: QuarterTurns::Zero,
        },
        Face::from_symbols(&symbols)?,
    )
}

fn cube_from_scan(scan: &link::ScanStatus) -> anyhow::Result<CubeState> {
    let mut cube = CubeState::default();
    for face in [
        link::CubeFace::Up,
        link::CubeFace::Right,
        link::CubeFace::Front,
        link::CubeFace::Down,
        link::CubeFace::Left,
        link::CubeFace::Back,
    ] {
        let recognized = scan.faces[face as usize]
            .ok_or_else(|| anyhow::anyhow!("scan is missing {:?}", face))?;
        record_recognized_face(&mut cube, face, recognized)?;
    }
    Ok(cube)
}

fn protocol_move(cube_move: CubeMove) -> link::CubeMove {
    link::CubeMove {
        face: match cube_move.face {
            LogicalFace::Up => link::CubeFace::Up,
            LogicalFace::Right => link::CubeFace::Right,
            LogicalFace::Front => link::CubeFace::Front,
            LogicalFace::Down => link::CubeFace::Down,
            LogicalFace::Left => link::CubeFace::Left,
            LogicalFace::Back => link::CubeFace::Back,
        },
        turn: match cube_move.turn {
            MoveTurn::Clockwise => link::TurnAmount::Clockwise,
            MoveTurn::CounterClockwise => link::TurnAmount::CounterClockwise,
            MoveTurn::Half => link::TurnAmount::Half,
        },
    }
}

fn internal_move(cube_move: link::CubeMove) -> CubeMove {
    CubeMove {
        face: logical_face(cube_move.face),
        turn: match cube_move.turn {
            link::TurnAmount::Clockwise => MoveTurn::Clockwise,
            link::TurnAmount::CounterClockwise => MoveTurn::CounterClockwise,
            link::TurnAmount::Half => MoveTurn::Half,
        },
    }
}

fn execute_steps(moves: &[CubeMove]) -> VecDeque<MotionStep> {
    let mut plan = optimized_execute_steps(moves);
    append_open_steps(&mut plan);
    motion_steps(plan)
}

fn held_execute_steps(moves: &[CubeMove]) -> VecDeque<MotionStep> {
    motion_steps(optimized_held_steps(moves))
}

fn open_steps() -> VecDeque<MotionStep> {
    let mut plan = VecDeque::new();
    append_open_steps(&mut plan);
    motion_steps(plan)
}

fn motion_steps(plan: VecDeque<MovePlanStep>) -> VecDeque<MotionStep> {
    plan.into_iter()
        .map(|step| match step {
            MovePlanStep::SetRails(target, position) => MotionStep::SetRails(target, position),
            MovePlanStep::SetGrippers(poses) => MotionStep::SetGrippers(poses),
            MovePlanStep::MoveCompleted => MotionStep::MoveCompleted,
            MovePlanStep::AllOff => MotionStep::AllOff,
        })
        .collect()
}

fn logical_face(face: link::CubeFace) -> LogicalFace {
    match face {
        link::CubeFace::Up => LogicalFace::Up,
        link::CubeFace::Right => LogicalFace::Right,
        link::CubeFace::Front => LogicalFace::Front,
        link::CubeFace::Down => LogicalFace::Down,
        link::CubeFace::Left => LogicalFace::Left,
        link::CubeFace::Back => LogicalFace::Back,
    }
}

fn protocol_color_symbol(color: link::StickerColor) -> char {
    match color {
        link::StickerColor::White => 'W',
        link::StickerColor::Yellow => 'Y',
        link::StickerColor::Red => 'R',
        link::StickerColor::Orange => 'O',
        link::StickerColor::Green => 'G',
        link::StickerColor::Blue => 'B',
        link::StickerColor::Unknown => '?',
    }
}

fn add_color_counts(counts: &mut [u8; link::FACE_COUNT], face: link::RecognizedFace) {
    for color in face.colors {
        if color != link::StickerColor::Unknown {
            counts[color as usize] = counts[color as usize].saturating_add(1);
        }
    }
}

fn transitional_pose() -> link::StandPose {
    link::StandPose {
        kind: link::StandPoseKind::Transitional,
        camera_face: None,
    }
}

fn unknown_axis<T>() -> (link::AxisMotion, Option<T>, Option<T>) {
    (link::AxisMotion::Unknown, None, None)
}

fn unknown_stand() -> link::StandState {
    let rail = unknown_axis();
    let gripper = unknown_axis();
    link::StandState {
        pose: link::StandPose {
            kind: link::StandPoseKind::Unknown,
            camera_face: None,
        },
        rails: [link::RailStatus {
            motion: rail.0,
            current: rail.1,
            target: rail.2,
        }; link::AXIS_COUNT],
        grippers: [link::GripperStatus {
            motion: gripper.0,
            current: gripper.1,
            target: gripper.2,
        }; link::AXIS_COUNT],
        outputs_enabled: false,
    }
}

fn empty_scan() -> link::ScanStatus {
    link::ScanStatus {
        state: link::ScanStateKind::None,
        revision: None,
        current_face: None,
        camera_face: None,
        scanned_faces: 0,
        faces: [None; link::FACE_COUNT],
        color_counts: [0; link::FACE_COUNT],
        validation_error: None,
    }
}

fn empty_solution() -> link::SolutionStatus {
    let empty_move = link::CubeMove {
        face: link::CubeFace::Up,
        turn: link::TurnAmount::Clockwise,
    };
    link::SolutionStatus {
        state: link::SolutionStateKind::None,
        id: None,
        source_scan_revision: None,
        moves: [empty_move; link::MAX_SOLUTION_MOVES],
        move_count: 0,
        completed_moves: 0,
    }
}

pub(crate) fn unknown_status() -> link::StatusSnapshot {
    link::StatusSnapshot {
        controller: link::ControllerState::Ready,
        stand: unknown_stand(),
        cube_session: None,
        scan: empty_scan(),
        solution: empty_solution(),
        active_operation: None,
        plan: [None; link::MAX_PLAN_PREVIEW_ACTIONS],
        plan_count: 0,
        fault: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result;

    #[derive(Default)]
    struct MockOutput {
        sets: Vec<Vec<(u8, u16)>>,
        disabled: Vec<Vec<u8>>,
        all_off_count: usize,
    }

    impl PwmOutput for MockOutput {
        fn set_channels(&mut self, channels: &[(u8, u16)]) -> Result<()> {
            self.sets.push(channels.to_vec());
            Ok(())
        }

        fn disable_channels(&mut self, channels: &[u8]) -> Result<()> {
            self.disabled.push(channels.to_vec());
            Ok(())
        }

        fn all_off(&mut self) -> Result<()> {
            self.all_off_count += 1;
            Ok(())
        }
    }

    fn assert_rail_and_gripper_motion_are_serialized<S: FaceScanner>(
        service: &RobotService<MockOutput, S>,
    ) {
        let stand = &service.status().stand;
        let rail_moving = stand
            .rails
            .iter()
            .any(|axis| axis.motion == link::AxisMotion::Moving);
        let gripper_moving = stand
            .grippers
            .iter()
            .any(|axis| axis.motion == link::AxisMotion::Moving);
        assert!(!(rail_moving && gripper_moving));
    }

    fn packet(opcode: link::RequestOpcode, request_id: u32) -> ReceivedPacket {
        packet_with_payload(opcode, request_id, &[])
    }

    fn packet_with_payload(
        opcode: link::RequestOpcode,
        request_id: u32,
        payload: &[u8],
    ) -> ReceivedPacket {
        let inner = link::Packet {
            kind: link::MessageKind::Request,
            opcode: opcode.into(),
            request_id,
            payload,
        };
        let mut scratch = [0; link::MAX_PACKET_LEN];
        let mut frame = [0; link::MAX_UART_FRAME_LEN];
        let frame_len = link::frame_uart(inner, &mut scratch, &mut frame).unwrap();
        let mut decoder = crate::robot_link::UartStreamDecoder::default();
        frame[..frame_len]
            .iter()
            .find_map(|&byte| decoder.push(byte))
            .unwrap()
            .unwrap()
    }

    fn start_scan_packet(session_id: u32, request_id: u32) -> ReceivedPacket {
        let mut payload = [0; link::MAX_PAYLOAD_LEN];
        let payload =
            link::encode_payload(&link::StartScanCommand { session_id }, &mut payload).unwrap();
        packet_with_payload(link::RequestOpcode::StartScan, request_id, payload)
    }

    fn solve_packet(session_id: u32, scan_revision: u32, request_id: u32) -> ReceivedPacket {
        let mut payload = [0; link::MAX_PAYLOAD_LEN];
        let payload = link::encode_payload(
            &link::SolveCommand {
                session_id,
                scan_revision,
            },
            &mut payload,
        )
        .unwrap();
        packet_with_payload(link::RequestOpcode::Solve, request_id, payload)
    }

    fn execute_packet(
        session_id: u32,
        scan_revision: u32,
        solution_id: u32,
        request_id: u32,
    ) -> ReceivedPacket {
        let mut payload = [0; link::MAX_PAYLOAD_LEN];
        let payload = link::encode_payload(
            &link::ExecuteCommand {
                session_id,
                scan_revision,
                solution_id,
            },
            &mut payload,
        )
        .unwrap();
        packet_with_payload(link::RequestOpcode::Execute, request_id, payload)
    }

    fn execute_moves_packet(
        session_id: u32,
        moves: &[link::CubeMove],
        request_id: u32,
    ) -> ReceivedPacket {
        let empty_move = link::CubeMove {
            face: link::CubeFace::Up,
            turn: link::TurnAmount::Clockwise,
        };
        let mut command = link::ExecuteMovesCommand {
            session_id,
            moves: [empty_move; link::MAX_SOLUTION_MOVES],
            move_count: moves.len() as u8,
        };
        command.moves[..moves.len()].copy_from_slice(moves);
        let mut payload = [0; link::MAX_PAYLOAD_LEN];
        let payload = link::encode_payload(&command, &mut payload).unwrap();
        packet_with_payload(link::RequestOpcode::ExecuteMoves, request_id, payload)
    }

    fn automatic_packet(session_id: u32, request_id: u32) -> ReceivedPacket {
        let mut payload = [0; link::MAX_PAYLOAD_LEN];
        let payload =
            link::encode_payload(&link::ScanSolveExecuteCommand { session_id }, &mut payload)
                .unwrap();
        packet_with_payload(link::RequestOpcode::ScanSolveExecute, request_id, payload)
    }

    fn open_packet(session_id: u32, request_id: u32) -> ReceivedPacket {
        let mut payload = [0; link::MAX_PAYLOAD_LEN];
        let payload =
            link::encode_payload(&link::OpenCommand { session_id }, &mut payload).unwrap();
        packet_with_payload(link::RequestOpcode::Open, request_id, payload)
    }

    struct SolvedScanner;

    impl FaceScanner for SolvedScanner {
        fn capture(&mut self, face: link::CubeFace) -> anyhow::Result<link::RecognizedFace> {
            let color = match face {
                link::CubeFace::Up => link::StickerColor::White,
                link::CubeFace::Right => link::StickerColor::Red,
                link::CubeFace::Front => link::StickerColor::Green,
                link::CubeFace::Down => link::StickerColor::Yellow,
                link::CubeFace::Left => link::StickerColor::Orange,
                link::CubeFace::Back => link::StickerColor::Blue,
            };
            Ok(link::RecognizedFace {
                colors: [color; link::STICKERS_PER_FACE],
                confidence: [255; link::STICKERS_PER_FACE],
            })
        }
    }

    struct FailingScanner;

    impl FaceScanner for FailingScanner {
        fn capture(&mut self, _face: link::CubeFace) -> anyhow::Result<link::RecognizedFace> {
            anyhow::bail!("simulated recognition failure")
        }
    }

    fn recover_and_grip<S: FaceScanner>(service: &mut RobotService<MockOutput, S>, base: Instant) {
        service.handle_packet(&packet(link::RequestOpcode::RecoverToOpen, 1), base);
        for second in 0..=6 {
            service.tick(base + std::time::Duration::from_secs(second));
        }
        service.handle_packet(&packet(link::RequestOpcode::Grip, 2), base);
        for second in 0..=4 {
            service.tick(base + std::time::Duration::from_secs(7 + second));
        }
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
    }

    fn complete_scan<S: FaceScanner>(service: &mut RobotService<MockOutput, S>, base: Instant) {
        service.handle_packet(&start_scan_packet(1, 3), base);
        for step in 0..100 {
            service.tick(base + std::time::Duration::from_secs(10 + step));
            if service.status().active_operation.is_none() {
                break;
            }
        }
    }

    fn install_solution<S: FaceScanner>(
        service: &mut RobotService<MockOutput, S>,
        moves: &[link::CubeMove],
    ) {
        service.status.scan.state = link::ScanStateKind::Valid;
        service.status.scan.revision = Some(1);
        service.status.solution = empty_solution();
        service.status.solution.state = link::SolutionStateKind::Ready;
        service.status.solution.id = Some(1);
        service.status.solution.source_scan_revision = Some(1);
        service.status.solution.move_count = moves.len() as u8;
        service.status.solution.moves[..moves.len()].copy_from_slice(moves);
    }

    fn accepted_operation(messages: &[ServiceMessage]) -> u32 {
        match &messages[0] {
            ServiceMessage::Response(ResponseMessage {
                payload:
                    ResponsePayload::Accepted(link::CommandAccepted {
                        operation_id: Some(id),
                    }),
                ..
            }) => *id,
            other => panic!("expected accepted operation, got {other:?}"),
        }
    }

    #[test]
    fn recovery_is_deadline_driven_and_establishes_open_pose() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());
        let messages = service.handle_packet(&packet(link::RequestOpcode::RecoverToOpen, 1), base);
        assert_eq!(accepted_operation(&messages), 1);

        service.tick(base);
        assert_rail_and_gripper_motion_are_serialized(&service);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::Transitional
        );
        assert_eq!(service.output.sets[0], vec![(5, 2500), (7, 2500)]);

        service.tick(base + std::time::Duration::from_millis(1_200));
        assert_rail_and_gripper_motion_are_serialized(&service);
        assert_eq!(service.output.sets[1], vec![(4, 2500), (6, 2500)]);
        service.tick(base + std::time::Duration::from_millis(2_400));
        assert_rail_and_gripper_motion_are_serialized(&service);
        assert_eq!(
            service.output.sets[2],
            vec![(3, 1450), (0, 1500), (2, 1450), (1, 1450)]
        );
        service.tick(base + std::time::Duration::from_millis(3_400));
        assert_rail_and_gripper_motion_are_serialized(&service);

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(service.status().stand.pose.kind, link::StandPoseKind::Open);
        assert!(!service.status().stand.outputs_enabled);
        assert_eq!(service.output.all_off_count, 1);
    }

    #[test]
    fn grip_creates_session_only_after_deadline_completion() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());
        service.handle_packet(&packet(link::RequestOpcode::RecoverToOpen, 1), base);
        service.tick(base);
        service.tick(base + std::time::Duration::from_millis(1_200));
        service.tick(base + std::time::Duration::from_millis(2_400));
        service.tick(base + std::time::Duration::from_millis(3_400));

        service.handle_packet(&packet(link::RequestOpcode::Grip, 2), base);
        service.tick(base);
        assert!(service.status().cube_session.is_none());
        service.tick(base + std::time::Duration::from_millis(1_000));
        assert!(service.status().cube_session.is_none());
        service.tick(base + std::time::Duration::from_millis(2_200));

        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
        assert_eq!(
            service.status().cube_session,
            Some(link::CubeSessionStatus { id: 1 })
        );
    }

    #[test]
    fn half_turn_gripper_motion_gets_twice_the_quarter_turn_deadline() {
        let calibration = StandCalibration::default();
        let base_duration = calibration.gripper_pose_duration();
        let mut service = RobotService::new(MockOutput::default(), calibration);
        service.status.stand.grippers[link::Axis::Top as usize].current =
            Some(link::GripperOrientation::FrameParallel);

        let quarter = service.gripper_motion_duration(&[(
            StandAxis::TopGripper,
            GripperOrientation::FramePerpendicular,
        )]);
        let half = service.gripper_motion_duration(&[(
            StandAxis::TopGripper,
            GripperOrientation::FrameParallelReversed,
        )]);

        assert_eq!(quarter, base_duration);
        assert_eq!(half, base_duration * 2);
    }

    #[test]
    fn abort_disables_outputs_without_waiting_for_motion_deadline() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());
        service.handle_packet(&packet(link::RequestOpcode::RecoverToOpen, 1), base);
        service.tick(base);
        assert_eq!(service.output.all_off_count, 0);

        let messages = service.handle_packet(&packet(link::RequestOpcode::Abort, 2), base);

        assert!(matches!(
            &messages[0],
            ServiceMessage::Response(ResponseMessage {
                payload: ResponsePayload::Accepted(_),
                ..
            })
        ));
        assert_eq!(service.output.all_off_count, 1);
        assert_eq!(service.status().controller, link::ControllerState::Aborted);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::Unknown
        );
        assert!(service.active_motion.is_none());
    }

    #[test]
    fn local_control_uses_service_admission_and_emits_only_events() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());

        let (outcome, messages) = service.handle_local_command(LocalCommand::RecoverToOpen, base);

        assert_eq!(
            outcome,
            LocalCommandOutcome::Accepted {
                operation_id: Some(1)
            }
        );
        assert!(messages
            .iter()
            .all(|message| matches!(message, ServiceMessage::Event(_))));
        assert_eq!(service.status().controller, link::ControllerState::Busy);
        assert_eq!(
            service.status().active_operation.unwrap().kind,
            link::OperationKind::RecoverToOpen
        );
    }

    #[test]
    fn duplicate_request_id_does_not_start_a_second_operation() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());
        let first = service.handle_packet(&packet(link::RequestOpcode::RecoverToOpen, 41), base);
        let duplicate =
            service.handle_packet(&packet(link::RequestOpcode::RecoverToOpen, 41), base);

        assert_eq!(first[0], duplicate[0]);
        assert_eq!(service.next_operation_id, 2);
    }

    #[test]
    fn status_is_available_before_recovery() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());

        let messages = service.handle_packet(&packet(link::RequestOpcode::GetStatus, 1), base);

        match &messages[0] {
            ServiceMessage::Response(ResponseMessage {
                opcode: link::ResponseOpcode::StatusSnapshot,
                payload: ResponsePayload::Status(status),
                ..
            }) => {
                assert_eq!(status.controller, link::ControllerState::Ready);
                assert_eq!(status.stand.pose.kind, link::StandPoseKind::Unknown);
            }
            other => panic!("expected status snapshot, got {other:?}"),
        }
        assert!(service.output.sets.is_empty());
    }

    #[test]
    fn grip_is_rejected_until_recovery_establishes_open_pose() {
        let base = Instant::now();
        let mut service = RobotService::new(MockOutput::default(), StandCalibration::default());

        let messages = service.handle_packet(&packet(link::RequestOpcode::Grip, 1), base);

        assert!(matches!(
            &messages[0],
            ServiceMessage::Response(ResponseMessage {
                payload: ResponsePayload::Rejected(link::CommandRejected {
                    reason: link::RejectionReason::StandPositionUnknown,
                    ..
                }),
                ..
            })
        ));
        assert!(service.output.sets.is_empty());
    }

    #[test]
    fn scan_runs_all_faces_and_returns_to_canonical_grip() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);

        let messages = service.handle_packet(&start_scan_packet(1, 3), base);
        assert_eq!(accepted_operation(&messages), 3);
        let mut events = Vec::new();
        for step in 0..100 {
            events.extend(service.tick(base + std::time::Duration::from_secs(10 + step)));
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
        assert_eq!(service.status().scan.state, link::ScanStateKind::Valid);
        assert_eq!(service.status().scan.revision, Some(1));
        assert_eq!(service.status().scan.scanned_faces, 0b11_1111);
        assert_eq!(service.status().scan.color_counts, [9; 6]);
        assert_eq!(
            events
                .iter()
                .filter(|message| matches!(
                    message,
                    ServiceMessage::Event(EventMessage::FaceScanned(_))
                ))
                .count(),
            6
        );
    }

    #[test]
    fn scan_rejects_a_stale_session_id() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);

        let messages = service.handle_packet(&start_scan_packet(99, 3), base);
        assert!(matches!(
            &messages[0],
            ServiceMessage::Response(ResponseMessage {
                payload: ResponsePayload::Rejected(link::CommandRejected {
                    reason: link::RejectionReason::SessionMismatch,
                    ..
                }),
                ..
            })
        ));
    }

    #[test]
    fn abort_interrupts_scan_before_its_motion_deadline() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        service.handle_packet(&start_scan_packet(1, 3), base);
        service.tick(base + std::time::Duration::from_secs(10));

        service.handle_packet(&packet(link::RequestOpcode::Abort, 4), base);

        assert_eq!(service.status().controller, link::ControllerState::Aborted);
        assert!(service.active_scan.is_none());
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::Unknown
        );
    }

    #[test]
    fn recognition_failure_still_returns_the_held_cube_to_canonical_grip() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            FailingScanner,
        );
        recover_and_grip(&mut service, base);
        service.handle_packet(&start_scan_packet(1, 3), base);
        let mut events = Vec::new();
        for step in 0..100 {
            events.extend(service.tick(base + std::time::Duration::from_secs(10 + step)));
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
        assert_eq!(service.status().scan.state, link::ScanStateKind::Invalid);
        assert!(events.iter().any(|message| matches!(
            message,
            ServiceMessage::Event(EventMessage::OperationFailed(link::OperationFailed {
                kind: link::OperationFailureKind::Recognition,
                ..
            }))
        )));
    }

    #[test]
    fn solve_publishes_a_revision_bound_solution_without_moving_the_stand() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        complete_scan(&mut service, base);
        let pwm_writes_before_solve = service.output.sets.len();

        let messages = service.handle_packet(&solve_packet(1, 1, 4), base);
        assert_eq!(accepted_operation(&messages), 4);
        assert_eq!(
            service.status().solution.state,
            link::SolutionStateKind::Solving
        );
        for _ in 0..1_000 {
            service.tick(base);
            if service.status().active_operation.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(
            service.status().solution.state,
            link::SolutionStateKind::Ready
        );
        assert_eq!(service.status().solution.id, Some(1));
        assert_eq!(service.status().solution.source_scan_revision, Some(1));
        assert_eq!(service.status().solution.move_count, 0);
        assert_eq!(service.output.sets.len(), pwm_writes_before_solve);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
    }

    #[test]
    fn solve_rejects_a_stale_scan_revision() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        complete_scan(&mut service, base);

        let messages = service.handle_packet(&solve_packet(1, 99, 4), base);
        assert!(matches!(
            &messages[0],
            ServiceMessage::Response(ResponseMessage {
                payload: ResponsePayload::Rejected(link::CommandRejected {
                    reason: link::RejectionReason::ScanRevisionMismatch,
                    ..
                }),
                ..
            })
        ));
    }

    #[test]
    fn abort_cancels_an_in_flight_solver_result() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        complete_scan(&mut service, base);
        service.handle_packet(&solve_packet(1, 1, 4), base);

        service.handle_packet(&packet(link::RequestOpcode::Abort, 5), base);

        assert_eq!(service.status().controller, link::ControllerState::Aborted);
        assert!(service.active_solve.is_none());
        assert_eq!(
            service.status().solution.state,
            link::SolutionStateKind::None
        );
    }

    #[test]
    fn execute_runs_a_saved_move_then_opens_and_ends_the_session() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        let writes_before_execute = service.output.sets.len();
        install_solution(
            &mut service,
            &[link::CubeMove {
                face: link::CubeFace::Right,
                turn: link::TurnAmount::Clockwise,
            }],
        );

        let messages = service.handle_packet(&execute_packet(1, 1, 1, 4), base);
        assert_eq!(accepted_operation(&messages), 3);
        let mut saw_completed_move_in_move_pose = false;
        for step in 0..100 {
            service.tick(base + std::time::Duration::from_secs(20 + step));
            if service.status().solution.completed_moves == 1
                && service.status().stand.pose.kind == link::StandPoseKind::MovePose
            {
                saw_completed_move_in_move_pose = true;
            }
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert!(saw_completed_move_in_move_pose);
        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(service.status().stand.pose.kind, link::StandPoseKind::Open);
        assert!(!service.status().stand.outputs_enabled);
        assert!(service.status().cube_session.is_none());
        assert_eq!(service.status().scan.state, link::ScanStateKind::None);
        assert_eq!(
            service.status().solution.state,
            link::SolutionStateKind::None
        );
        assert_eq!(
            &service.output.sets[writes_before_execute..],
            &[
                vec![(0, 2500)],
                vec![(5, 2500), (7, 2500)],
                vec![(4, 2500), (6, 2500)],
            ]
        );
    }

    #[test]
    fn execute_rejects_a_stale_solution_id() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        install_solution(&mut service, &[]);

        let messages = service.handle_packet(&execute_packet(1, 1, 99, 4), base);
        assert!(matches!(
            &messages[0],
            ServiceMessage::Response(ResponseMessage {
                payload: ResponsePayload::Rejected(link::CommandRejected {
                    reason: link::RejectionReason::SolutionMismatch,
                    ..
                }),
                ..
            })
        ));
    }

    #[test]
    fn manual_moves_execute_without_opening_or_ending_the_session() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        install_solution(&mut service, &[]);
        let writes_before_execute = service.output.sets.len();
        let all_off_before_execute = service.output.all_off_count;

        let messages = service.handle_packet(
            &execute_moves_packet(
                1,
                &[link::CubeMove {
                    face: link::CubeFace::Right,
                    turn: link::TurnAmount::Clockwise,
                }],
                4,
            ),
            base,
        );
        assert_eq!(accepted_operation(&messages), 3);
        for step in 0..100 {
            service.tick(base + std::time::Duration::from_secs(20 + step));
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
        assert!(service.status().stand.outputs_enabled);
        assert_eq!(
            service.status().cube_session,
            Some(link::CubeSessionStatus { id: 1 })
        );
        assert_eq!(service.status().scan.state, link::ScanStateKind::None);
        assert_eq!(
            service.status().solution.state,
            link::SolutionStateKind::None
        );
        assert_eq!(service.output.all_off_count, all_off_before_execute);
        assert_eq!(
            &service.output.sets[writes_before_execute..],
            &[
                vec![(0, 2500)],
                vec![(7, 2500)],
                vec![(0, 1500)],
                vec![(7, 1200)],
            ]
        );
    }

    #[test]
    fn manual_moves_reject_an_empty_sequence() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);

        let messages = service.handle_packet(&execute_moves_packet(1, &[], 4), base);
        assert!(matches!(
            &messages[0],
            ServiceMessage::Response(ResponseMessage {
                payload: ResponsePayload::Rejected(link::CommandRejected {
                    reason: link::RejectionReason::InvalidPayload,
                    ..
                }),
                ..
            })
        ));
    }

    #[test]
    fn manual_opposite_faces_share_one_regrip() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        let writes_before_execute = service.output.sets.len();

        let messages = service.handle_packet(
            &execute_moves_packet(
                1,
                &[
                    link::CubeMove {
                        face: link::CubeFace::Right,
                        turn: link::TurnAmount::Clockwise,
                    },
                    link::CubeMove {
                        face: link::CubeFace::Left,
                        turn: link::TurnAmount::Clockwise,
                    },
                ],
                4,
            ),
            base,
        );
        assert_eq!(accepted_operation(&messages), 3);
        assert_eq!(service.status().active_operation.unwrap().action_count, 7);
        for step in 0..100 {
            service.tick(base + std::time::Duration::from_secs(20 + step));
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
        assert_eq!(
            &service.output.sets[writes_before_execute..],
            &[
                vec![(0, 2500)],
                vec![(3, 2450)],
                vec![(5, 2500), (7, 2500)],
                vec![(3, 1450), (0, 1500)],
                vec![(5, 1200), (7, 1200)],
            ]
        );
    }

    #[test]
    fn front_half_turn_plan_contains_both_whole_cube_regrips() {
        let steps = execute_steps(&[CubeMove {
            face: LogicalFace::Front,
            turn: MoveTurn::Half,
        }]);

        // 6 position + 4 half-turn + marker + 3 normal-open. Release finish
        // intentionally skips canonicalization and restore.
        assert_eq!(steps.len(), 14);
        assert!(matches!(
            steps[0],
            MotionStep::SetRails(RailTarget::Pair(RailPair::LeftRight), RailPosition::FarOpen)
        ));
        assert!(matches!(
            steps[6],
            MotionStep::SetRails(
                RailTarget::Single(StandAxis::RightRail),
                RailPosition::FarOpen
            )
        ));
        assert!(matches!(steps[10], MotionStep::MoveCompleted));
        assert!(matches!(steps[13], MotionStep::AllOff));
    }

    #[test]
    fn every_supported_move_plan_preserves_grip_and_collision_invariants() {
        for face in LogicalFace::ALL {
            for turn in [
                MoveTurn::Clockwise,
                MoveTurn::CounterClockwise,
                MoveTurn::Half,
            ] {
                assert_safe_execute_plan(execute_steps(&[CubeMove { face, turn }]));
            }
        }
    }

    fn assert_safe_execute_plan(steps: VecDeque<MotionStep>) {
        let mut rails = [RailPosition::NearGrip; 4];
        let mut grippers = [GripperOrientation::FramePerpendicular; 4];
        let mut saw_move_boundary = false;
        for step in steps {
            match step {
                MotionStep::SetRails(target, position) => match target {
                    RailTarget::Pair(pair) => {
                        let (_, axes) = rail_pair_axes(pair);
                        for axis in axes {
                            rails[axis as usize] = position;
                        }
                    }
                    RailTarget::Single(axis) => rails[stand_axis_index(axis)] = position,
                },
                MotionStep::SetGrippers(poses) => {
                    for (axis, orientation) in poses {
                        grippers[stand_axis_index(axis)] = orientation;
                    }
                }
                MotionStep::MoveCompleted => {
                    saw_move_boundary = true;
                }
                MotionStep::AllOff => {}
                MotionStep::Capture(_) => panic!("execute plan must not capture"),
            }

            for (first, second) in [(0, 2), (2, 1), (1, 3), (3, 0)] {
                let both_gripped = rails[first] == RailPosition::NearGrip
                    && rails[second] == RailPosition::NearGrip;
                let both_parallel =
                    grippers[first].is_frame_parallel() && grippers[second].is_frame_parallel();
                assert!(
                    !(both_gripped && both_parallel),
                    "adjacent grippers {first}/{second} collide"
                );
            }
        }
        assert!(saw_move_boundary);
        assert_eq!(rails, [RailPosition::FarOpen; 4]);
    }

    #[test]
    fn open_releases_a_held_cube_without_rotating_grippers() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);
        let writes_before_open = service.output.sets.len();

        let messages = service.handle_packet(&open_packet(1, 3), base);
        assert_eq!(accepted_operation(&messages), 3);
        for step in 0..20 {
            service.tick(base + std::time::Duration::from_secs(20 + step));
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert_eq!(service.status().stand.pose.kind, link::StandPoseKind::Open);
        assert!(service.status().cube_session.is_none());
        assert_eq!(
            &service.output.sets[writes_before_open..],
            &[vec![(5, 2500), (7, 2500)], vec![(4, 2500), (6, 2500)]]
        );
    }

    #[test]
    fn automatic_workflow_uses_one_operation_and_opens_after_execute() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            SolvedScanner,
        );
        recover_and_grip(&mut service, base);

        let mut messages = service.handle_packet(&automatic_packet(1, 3), base);
        assert_eq!(accepted_operation(&messages), 3);
        for step in 0..500 {
            messages.extend(service.tick(base + std::time::Duration::from_secs(20 + step)));
            if service.status().active_operation.is_none() {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(1));
        }

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(service.status().stand.pose.kind, link::StandPoseKind::Open);
        assert!(service.status().cube_session.is_none());
        let completions = messages
            .iter()
            .filter_map(|message| match message {
                ServiceMessage::Event(EventMessage::OperationCompleted(event)) => Some(*event),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(
            completions,
            vec![link::OperationCompleted {
                operation_id: 3,
                kind: link::OperationKind::ScanSolveExecute,
            }]
        );
        assert_eq!(
            messages
                .iter()
                .filter(|message| matches!(
                    message,
                    ServiceMessage::Event(EventMessage::FaceScanned(_))
                ))
                .count(),
            6
        );
    }

    #[test]
    fn automatic_scan_failure_keeps_the_session_and_canonical_grip() {
        let base = Instant::now();
        let mut service = RobotService::with_scanner(
            MockOutput::default(),
            StandCalibration::default(),
            FailingScanner,
        );
        recover_and_grip(&mut service, base);
        service.handle_packet(&automatic_packet(1, 3), base);
        for step in 0..100 {
            service.tick(base + std::time::Duration::from_secs(20 + step));
            if service.status().active_operation.is_none() {
                break;
            }
        }

        assert_eq!(service.status().controller, link::ControllerState::Ready);
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::CanonicalGrip
        );
        assert_eq!(
            service.status().cube_session,
            Some(link::CubeSessionStatus { id: 1 })
        );
        assert_eq!(service.status().scan.state, link::ScanStateKind::Invalid);
    }
}
