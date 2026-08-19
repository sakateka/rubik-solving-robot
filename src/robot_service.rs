//! Deadline-driven robot control service for UART/BLE commands.
//!
//! The service owns the PWM device and never blocks the event loop with motion
//! delays. Callers interleave [`RobotService::tick`] with transport processing,
//! which keeps `Abort` and `GetStatus` responsive while servo movement
//! deadlines are pending.

use crate::{
    pca9685::PwmOutput,
    robot_link::{FrameEncodeError, ReceivedPacket, UartFrameEncoder},
    stand::{GripperOrientation, RailPosition, StandAxis, StandCalibration},
};
use rubik_link_protocol as link;
use std::{collections::VecDeque, time::Instant};

const REQUEST_CACHE_CAPACITY: usize = 16;

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
    OperationCompleted(link::OperationCompleted),
    Aborted(link::Aborted),
    Fault(link::FaultEvent),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ServiceMessage {
    Response(ResponseMessage),
    Event(EventMessage),
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

pub struct RobotService<D> {
    output: D,
    calibration: StandCalibration,
    status: link::StatusSnapshot,
    active_motion: Option<ActiveMotion>,
    next_operation_id: u32,
    next_session_id: u32,
    request_cache: VecDeque<CachedResponse>,
    events: VecDeque<EventMessage>,
}

impl<D> RobotService<D>
where
    D: PwmOutput,
{
    pub fn new(output: D, calibration: StandCalibration) -> Self {
        Self {
            output,
            calibration,
            status: unknown_status(),
            active_motion: None,
            next_operation_id: 1,
            next_session_id: 1,
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
        self.drain_events().map(ServiceMessage::Event).collect()
    }

    pub fn shutdown(&mut self) -> anyhow::Result<()> {
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
            link::RequestOpcode::Abort => {
                if !packet.payload().is_empty() {
                    return self.rejected(packet.request_id, link::RejectionReason::InvalidPayload);
                }
                let operation_id = self.status.active_operation.map(|operation| operation.id);
                self.abort(operation_id);
                self.accepted(packet.request_id, None)
            }
            _ => self.rejected(packet.request_id, link::RejectionReason::UnsupportedCommand),
        }
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
        if self.active_motion.is_some() {
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
                    self.calibration.rail_pulse(RailPosition::NearGrip),
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
        let channels = physical.map(|axis| (axis.channel(), self.calibration.rail_pulse(position)));
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
        match self.output.all_off() {
            Ok(()) => {
                self.active_motion = None;
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
        let _ = self.output.all_off();
        self.active_motion = None;
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

fn unknown_status() -> link::StatusSnapshot {
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

    fn packet(opcode: link::RequestOpcode, request_id: u32) -> ReceivedPacket {
        let inner = link::Packet {
            kind: link::MessageKind::Request,
            opcode: opcode.into(),
            request_id,
            payload: &[],
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
        assert_eq!(
            service.status().stand.pose.kind,
            link::StandPoseKind::Transitional
        );
        assert_eq!(service.output.sets[0], vec![(5, 2500), (7, 2500)]);

        service.tick(base + std::time::Duration::from_millis(1_200));
        assert_eq!(service.output.sets[1], vec![(4, 2500), (6, 2500)]);
        service.tick(base + std::time::Duration::from_millis(2_400));
        assert_eq!(
            service.output.sets[2],
            vec![(3, 1450), (0, 1500), (2, 1450), (1, 1450)]
        );
        service.tick(base + std::time::Duration::from_millis(3_400));

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
}
