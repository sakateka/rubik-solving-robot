//! Embedded HTTP server for the robot simulator (axum).
//!
//! Serves the 3D operator UI, streams live status over server-sent events,
//! injects protocol commands into the daemon, and reports mechanical
//! collisions (adjacent parallel grippers, lost cube custody).

use crate::pca9685::PwmOutput;
use crate::robot_daemon::DaemonObserver;
use crate::robot_link::{UartFrameEncoder, UartStreamDecoder};
use crate::robot_service::{EventMessage, ResponseMessage, ResponsePayload, ServiceMessage};
use crate::robot_service::{FaceScanner, RobotService};
use crate::stand::StandCalibration;
use anyhow::Result;
use axum::{
    extract::{Query, State},
    http::{header, StatusCode},
    response::{
        sse::{KeepAlive, Sse},
        Html, IntoResponse, Response,
    },
    routing::{get, post},
    Router,
};
use rubik_link_protocol as link;
use serde::Deserialize;
use serde_json::{json, Value};
use std::{
    collections::HashMap,
    convert::Infallible,
    sync::{
        atomic::{AtomicBool, AtomicU32, Ordering},
        mpsc, Arc, Mutex,
    },
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
use tokio::sync::{broadcast, oneshot};
use tokio_stream::wrappers::BroadcastStream;
use tokio_stream::StreamExt;

pub const SIM_HTML: &str = include_str!("../web/sim.html");
pub const THREE_JS: &str = include_str!("../web/three.module.min.js");
pub const GRIPPER_STL: &[u8] = include_bytes!("../web/rcr_gripper-v5.stl");

const FIRST_HTTP_REQUEST_ID: u32 = 0x0100_0000;
const LAST_HTTP_REQUEST_ID: u32 = 0x0fff_ffff;
const COMMAND_TIMEOUT: Duration = Duration::from_secs(10);
const TICK: Duration = Duration::from_millis(20);
const SESSION_TTL: Duration = Duration::from_secs(8 * 60 * 60);
const MAX_SESSIONS: usize = 64;

/// Telemetry forwarded from the daemon thread to the HTTP thread.
pub enum SimUpdate {
    Status(Box<link::StatusSnapshot>),
    Event {
        opcode: u16,
        payload: Value,
    },
    Response {
        request_id: u32,
        opcode: u16,
        payload: Value,
    },
}

/// [`DaemonObserver`] side: forwards status changes and messages to the
/// HTTP pump thread. Runs on the daemon thread; must never block.
pub struct SimEngine {
    tx: mpsc::Sender<SimUpdate>,
    last: Option<link::StatusSnapshot>,
}

impl SimEngine {
    pub fn new(tx: mpsc::Sender<SimUpdate>) -> Self {
        Self { tx, last: None }
    }

    fn push(&self, update: SimUpdate) {
        let _ = self.tx.send(update);
    }
}

impl DaemonObserver for SimEngine {
    fn observe_status(&mut self, status: &link::StatusSnapshot) {
        if self.last != Some(*status) {
            self.last = Some(*status);
            self.push(SimUpdate::Status(Box::new(*status)));
        }
    }

    fn observe_messages(&mut self, messages: &[ServiceMessage]) {
        for message in messages {
            match message {
                ServiceMessage::Response(response) => self.push(SimUpdate::Response {
                    request_id: response.request_id,
                    opcode: u16::from(response.opcode),
                    payload: response_payload_json(response),
                }),
                ServiceMessage::Event(event) => self.push(SimUpdate::Event {
                    opcode: event_opcode(*event),
                    payload: event_payload_json(*event),
                }),
            }
        }
    }
}

fn response_payload_json(response: &ResponseMessage) -> Value {
    match &response.payload {
        ResponsePayload::Accepted(payload) => serde_json::to_value(payload).unwrap_or(Value::Null),
        ResponsePayload::Rejected(payload) => serde_json::to_value(payload).unwrap_or(Value::Null),
        ResponsePayload::Status(payload) => {
            serde_json::to_value(payload.as_ref()).unwrap_or(Value::Null)
        }
    }
}

fn event_opcode(event: EventMessage) -> u16 {
    match event {
        EventMessage::RobotStateChanged(_) => link::EventOpcode::RobotStateChanged.into(),
        EventMessage::StandStateChanged(_) => link::EventOpcode::StandStateChanged.into(),
        EventMessage::CubeSessionChanged(_) => link::EventOpcode::CubeSessionChanged.into(),
        EventMessage::FaceScanned(_) => link::EventOpcode::FaceScanned.into(),
        EventMessage::OperationFailed(_) => link::EventOpcode::OperationFailed.into(),
        EventMessage::OperationCompleted(_) => link::EventOpcode::OperationCompleted.into(),
        EventMessage::Aborted(_) => link::EventOpcode::Aborted.into(),
        EventMessage::Fault(_) => link::EventOpcode::Fault.into(),
    }
}

fn event_payload_json(event: EventMessage) -> Value {
    match event {
        EventMessage::RobotStateChanged(payload) => {
            serde_json::to_value(payload).unwrap_or(Value::Null)
        }
        EventMessage::StandStateChanged(payload) => {
            serde_json::to_value(payload).unwrap_or(Value::Null)
        }
        EventMessage::CubeSessionChanged(payload) => {
            serde_json::to_value(payload).unwrap_or(Value::Null)
        }
        EventMessage::FaceScanned(payload) => serde_json::to_value(payload).unwrap_or(Value::Null),
        EventMessage::OperationFailed(payload) => {
            serde_json::to_value(payload).unwrap_or(Value::Null)
        }
        EventMessage::OperationCompleted(payload) => {
            serde_json::to_value(payload).unwrap_or(Value::Null)
        }
        EventMessage::Aborted(payload) => serde_json::to_value(payload).unwrap_or(Value::Null),
        EventMessage::Fault(payload) => serde_json::to_value(payload).unwrap_or(Value::Null),
    }
}

fn orientation_angle(orientation: link::GripperOrientation) -> f32 {
    match orientation {
        link::GripperOrientation::FrameParallel => 0.0,
        link::GripperOrientation::FramePerpendicular => std::f32::consts::FRAC_PI_2,
        link::GripperOrientation::FrameParallelReversed => std::f32::consts::PI,
    }
}

fn orientation_index(orientation: link::GripperOrientation) -> i8 {
    match orientation {
        link::GripperOrientation::FrameParallel => 0,
        link::GripperOrientation::FramePerpendicular => 1,
        link::GripperOrientation::FrameParallelReversed => 2,
    }
}

#[derive(Clone, Copy)]
struct VisualMove {
    face: link::CubeFace,
    turns: i8,
}

#[derive(Clone, Copy)]
struct ActiveVisualTurn {
    operation_id: u32,
    move_index: usize,
    axis: usize,
    cube_move: VisualMove,
}

struct VisualReplay {
    frame_quarters: u8,
    previous_motion: [Option<link::AxisMotion>; 4],
    expected: Vec<VisualMove>,
    expected_index: usize,
    active: Option<ActiveVisualTurn>,
    invariant_violation: Option<String>,
    operation_id: Option<u32>,
    solution_id_at_start: Option<u32>,
}

impl Default for VisualReplay {
    fn default() -> Self {
        Self {
            frame_quarters: 0,
            previous_motion: [None; 4],
            expected: Vec::new(),
            expected_index: 0,
            active: None,
            invariant_violation: None,
            operation_id: None,
            solution_id_at_start: None,
        }
    }
}

impl VisualReplay {
    fn set_expected(&mut self, moves: &[link::CubeMove]) {
        self.expected = moves
            .iter()
            .map(|cube_move| VisualMove {
                face: cube_move.face,
                turns: match cube_move.turn {
                    link::TurnAmount::Clockwise => -1,
                    link::TurnAmount::CounterClockwise => 1,
                    link::TurnAmount::Half => 2,
                },
            })
            .collect();
        self.expected_index = 0;
        self.active = None;
        self.invariant_violation = None;
        self.previous_motion = [None; 4];
    }

    fn physical_face(&self, axis: usize) -> link::CubeFace {
        match (self.frame_quarters, axis) {
            (0, 0) => link::CubeFace::Left,
            (0, 1) => link::CubeFace::Right,
            (_, 0) => link::CubeFace::Back,
            (_, 1) => link::CubeFace::Front,
            (_, 2) => link::CubeFace::Up,
            (_, 3) => link::CubeFace::Down,
            _ => unreachable!("stand has exactly four gripper axes"),
        }
    }

    fn apply_status(&mut self, status: &link::StatusSnapshot) -> Option<VisualMove> {
        let mut completed_move = None;
        let canonical = status.stand.pose.kind == link::StandPoseKind::CanonicalGrip
            && status.stand.pose.camera_face == Some(link::CubeFace::Front)
            && status
                .stand
                .grippers
                .iter()
                .all(|gripper| gripper.motion != link::AxisMotion::Moving);
        if canonical {
            self.frame_quarters = 0;
        }
        let Some(operation) = status.active_operation else {
            self.previous_motion = status.stand.grippers.map(|axis| Some(axis.motion));
            self.active = None;
            self.operation_id = None;
            self.solution_id_at_start = None;
            return None;
        };

        if self.operation_id != Some(operation.id) {
            self.operation_id = Some(operation.id);
            if operation.kind == link::OperationKind::ScanSolveExecute {
                // Auto may initially expose the completed solution from the
                // previous cycle. It is not valid for this operation; wait
                // until the solver publishes a new solution id.
                self.expected.clear();
                self.expected_index = 0;
                self.active = None;
                self.solution_id_at_start = if status.solution.move_count > 0 {
                    status.solution.id
                } else {
                    None
                };
            } else {
                self.solution_id_at_start = None;
            }
        }

        if (self.expected.is_empty() || self.expected_index >= self.expected.len())
            && matches!(
                operation.kind,
                link::OperationKind::Execute | link::OperationKind::ScanSolveExecute
            )
            && status.solution.move_count > 0
            && self.solution_id_at_start != status.solution.id
        {
            self.set_expected(&status.solution.moves[..usize::from(status.solution.move_count)]);
        }

        let rails = &status.stand.rails;
        let grippers = &status.stand.grippers;
        let top_bottom_parallel = [2, 3].iter().all(|&index| {
            grippers[index].motion != link::AxisMotion::Moving
                && matches!(
                    grippers[index].current,
                    Some(
                        link::GripperOrientation::FrameParallel
                            | link::GripperOrientation::FrameParallelReversed
                    )
                )
        });
        if rails[0].current == Some(link::RailPosition::Open)
            && rails[1].current == Some(link::RailPosition::Open)
            && top_bottom_parallel
        {
            self.frame_quarters = match (grippers[2].current, grippers[3].current) {
                (
                    Some(link::GripperOrientation::FrameParallel),
                    Some(link::GripperOrientation::FrameParallelReversed),
                ) => 1,
                (
                    Some(link::GripperOrientation::FrameParallelReversed),
                    Some(link::GripperOrientation::FrameParallel),
                ) => 0,
                _ => self.frame_quarters,
            };
        }

        if matches!(
            operation.kind,
            link::OperationKind::Execute
                | link::OperationKind::ExecuteMoves
                | link::OperationKind::ScanSolveExecute
        ) {
            let started_axes = (0..4)
                .filter(|&axis| {
                    status.stand.grippers[axis].motion == link::AxisMotion::Moving
                        && self.previous_motion[axis] != Some(link::AxisMotion::Moving)
                })
                .collect::<Vec<_>>();
            for axis in 0..4 {
                let gripper = &grippers[axis];
                // A single moving gripper turns one layer. Two opposing
                // grippers moving together reorient the complete cube and
                // are handled by CubePresentation instead.
                let started = started_axes.len() == 1 && started_axes[0] == axis;
                let target_is_turn = matches!(
                    gripper.target,
                    Some(
                        link::GripperOrientation::FrameParallel
                            | link::GripperOrientation::FrameParallelReversed
                    )
                );
                if started && self.active.is_none() && target_is_turn {
                    if let Some(&expected) = self.expected.get(self.expected_index) {
                        if rails[axis].current == Some(link::RailPosition::Grip) {
                            // Servo-angle polarity is calibration/mounting
                            // space, not Singmaster space. Keep the requested
                            // turn sign and independently verify that the
                            // physical axis owns the requested logical face.
                            let physical_face = self.physical_face(axis);
                            if physical_face != expected.face {
                                self.invariant_violation = Some(format!(
                                    "expected {:?} but axis {axis} owns {:?}",
                                    expected.face, physical_face
                                ));
                                continue;
                            }
                            let mut rendered = expected;
                            if expected.turns.abs() == 2 {
                                let Some((current, target)) = gripper.current.zip(gripper.target)
                                else {
                                    continue;
                                };
                                let physical_turns =
                                    orientation_index(current) - orientation_index(target);
                                if physical_turns.abs() != 2 {
                                    self.invariant_violation = Some(format!(
                                        "expected a half-turn on axis {axis}, physical delta is {physical_turns}"
                                    ));
                                    continue;
                                }
                                // +180 and -180 are logically identical, but
                                // visually the layer must follow the claw.
                                rendered.turns = physical_turns;
                            }
                            self.active = Some(ActiveVisualTurn {
                                operation_id: operation.id,
                                move_index: self.expected_index,
                                axis,
                                cube_move: rendered,
                            });
                        }
                    }
                }
                if let Some(active) = self.active.as_mut() {
                    if active.axis == axis
                        && self.previous_motion[axis] == Some(link::AxisMotion::Moving)
                        && gripper.motion != link::AxisMotion::Moving
                    {
                        completed_move = Some(active.cube_move);
                        self.expected_index += 1;
                        self.active = None;
                    }
                }
                self.previous_motion[axis] = Some(gripper.motion);
            }
        } else {
            self.previous_motion = grippers.map(|axis| Some(axis.motion));
        }
        completed_move
    }
}

#[derive(Clone)]
struct ServerCubie {
    home: [i8; 3],
    position: [i8; 3],
    /// Images of the cubie's local X/Y/Z unit vectors in world coordinates.
    basis: [[i8; 3]; 3],
}

struct ServerCube {
    cubies: Vec<ServerCubie>,
    revision: u64,
}

#[derive(Clone, Copy)]
struct ActiveRigidTurn {
    operation_id: u32,
    driver: usize,
    axis: [i8; 3],
    turns: i8,
}

/// Presentation-only orientation of the complete cube while the scan plan
/// reorients it. This never changes logical facelets: only face turns do.
struct CubePresentation {
    basis: [[i8; 3]; 3],
    active: Option<ActiveRigidTurn>,
    previous_motion: [Option<link::AxisMotion>; 4],
}

impl Default for CubePresentation {
    fn default() -> Self {
        Self {
            basis: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            active: None,
            previous_motion: [None; 4],
        }
    }
}

impl CubePresentation {
    fn orientation_index(value: link::GripperOrientation) -> i8 {
        match value {
            link::GripperOrientation::FrameParallel => 0,
            link::GripperOrientation::FramePerpendicular => 1,
            link::GripperOrientation::FrameParallelReversed => 2,
        }
    }

    fn rotated_basis(&self, axis: [i8; 3], turns: i8) -> [[i8; 3]; 3] {
        self.basis
            .map(|vector| ServerCube::rotate(vector, axis, turns))
    }

    fn apply_status(&mut self, status: &link::StatusSnapshot) {
        if let Some(active) = self.active {
            if self.previous_motion[active.driver] == Some(link::AxisMotion::Moving)
                && status.stand.grippers[active.driver].motion != link::AxisMotion::Moving
            {
                self.basis = self.rotated_basis(active.axis, active.turns);
                self.active = None;
            }
        }

        let canonical = status.stand.pose.kind == link::StandPoseKind::CanonicalGrip
            && status.stand.pose.camera_face == Some(link::CubeFace::Front)
            && status
                .stand
                .grippers
                .iter()
                .all(|gripper| gripper.motion != link::AxisMotion::Moving);
        if canonical {
            self.basis = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
            self.active = None;
        }

        let mechanical_operation = status.active_operation.filter(|operation| {
            matches!(
                operation.kind,
                link::OperationKind::Scan
                    | link::OperationKind::Execute
                    | link::OperationKind::ExecuteMoves
                    | link::OperationKind::ScanSolveExecute
            )
        });
        if self.active.is_none() {
            if let Some(operation) = mechanical_operation {
                // A held top/bottom pair rotates the complete cube around Y.
                // Opposite servo mounting means cube turns are -top delta.
                self.try_start(status, operation.id, 2, 3, [0, 1, 0], -1);
                // A held left/right pair rotates it around X; left delta has
                // the same sign as the complete-cube rotation.
                if self.active.is_none() {
                    self.try_start(status, operation.id, 0, 1, [1, 0, 0], 1);
                }
            }
        }
        self.previous_motion = status.stand.grippers.map(|axis| Some(axis.motion));
    }

    fn try_start(
        &mut self,
        status: &link::StatusSnapshot,
        operation_id: u32,
        driver: usize,
        opposite: usize,
        axis: [i8; 3],
        sign: i8,
    ) {
        let gripper = &status.stand.grippers[driver];
        let started = gripper.motion == link::AxisMotion::Moving
            && self.previous_motion[driver] != Some(link::AxisMotion::Moving);
        let pair_holds = [driver, opposite].iter().all(|&index| {
            status.stand.rails[index].motion != link::AxisMotion::Moving
                && status.stand.rails[index].current == Some(link::RailPosition::Grip)
        });
        let pair_moves = status.stand.grippers[opposite].motion == link::AxisMotion::Moving;
        if !started || !pair_holds || !pair_moves {
            return;
        }
        let (Some(current), Some(target)) = (gripper.current, gripper.target) else {
            return;
        };
        let turns = (Self::orientation_index(target) - Self::orientation_index(current)) * sign;
        if turns != 0 {
            self.active = Some(ActiveRigidTurn {
                operation_id,
                driver,
                axis,
                turns,
            });
        }
    }
}

impl ServerCube {
    fn solved() -> Self {
        let mut cubies = Vec::with_capacity(26);
        for x in -1..=1 {
            for y in -1..=1 {
                for z in -1..=1 {
                    if x == 0 && y == 0 && z == 0 {
                        continue;
                    }
                    cubies.push(ServerCubie {
                        home: [x, y, z],
                        position: [x, y, z],
                        basis: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                    });
                }
            }
        }
        Self {
            cubies,
            revision: 0,
        }
    }

    fn face_normal(face: link::CubeFace) -> [i8; 3] {
        match face {
            link::CubeFace::Up => [0, 1, 0],
            link::CubeFace::Right => [1, 0, 0],
            link::CubeFace::Front => [0, 0, 1],
            link::CubeFace::Down => [0, -1, 0],
            link::CubeFace::Left => [-1, 0, 0],
            link::CubeFace::Back => [0, 0, -1],
        }
    }

    fn dot(first: [i8; 3], second: [i8; 3]) -> i8 {
        first[0] * second[0] + first[1] * second[1] + first[2] * second[2]
    }

    fn cross(first: [i8; 3], second: [i8; 3]) -> [i8; 3] {
        [
            first[1] * second[2] - first[2] * second[1],
            first[2] * second[0] - first[0] * second[2],
            first[0] * second[1] - first[1] * second[0],
        ]
    }

    fn rotate_quarter(vector: [i8; 3], axis: [i8; 3]) -> [i8; 3] {
        let cross = Self::cross(axis, vector);
        let parallel = Self::dot(axis, vector);
        [
            cross[0] + axis[0] * parallel,
            cross[1] + axis[1] * parallel,
            cross[2] + axis[2] * parallel,
        ]
    }

    fn rotate(vector: [i8; 3], axis: [i8; 3], turns: i8) -> [i8; 3] {
        let mut result = vector;
        for _ in 0..turns.rem_euclid(4) {
            result = Self::rotate_quarter(result, axis);
        }
        result
    }

    fn apply_move(&mut self, cube_move: VisualMove) {
        let normal = Self::face_normal(cube_move.face);
        for cubie in &mut self.cubies {
            if Self::dot(cubie.position, normal) <= 0 {
                continue;
            }
            cubie.position = Self::rotate(cubie.position, normal, cube_move.turns);
            for basis in &mut cubie.basis {
                *basis = Self::rotate(*basis, normal, cube_move.turns);
            }
        }
        self.revision = self.revision.wrapping_add(1);
    }

    fn sticker_color(local_normal: [i8; 3]) -> u8 {
        match local_normal {
            [0, 1, 0] => 0,
            [0, -1, 0] => 1,
            [1, 0, 0] => 2,
            [-1, 0, 0] => 3,
            [0, 0, 1] => 4,
            [0, 0, -1] => 5,
            _ => 255,
        }
    }

    fn transform_local(cubie: &ServerCubie, local: [i8; 3]) -> [i8; 3] {
        [
            cubie.basis[0][0] * local[0]
                + cubie.basis[1][0] * local[1]
                + cubie.basis[2][0] * local[2],
            cubie.basis[0][1] * local[0]
                + cubie.basis[1][1] * local[1]
                + cubie.basis[2][1] * local[2],
            cubie.basis[0][2] * local[0]
                + cubie.basis[1][2] * local[1]
                + cubie.basis[2][2] * local[2],
        ]
    }

    fn facelets(&self) -> [u8; 54] {
        let mut facelets = [255; 54];
        let views = [
            ([0, 1, 0], [1, 0, 0], [0, 0, -1]),
            ([1, 0, 0], [0, 0, -1], [0, 1, 0]),
            ([0, 0, 1], [1, 0, 0], [0, 1, 0]),
            ([0, -1, 0], [1, 0, 0], [0, 0, 1]),
            ([-1, 0, 0], [0, 0, 1], [0, 1, 0]),
            ([0, 0, -1], [-1, 0, 0], [0, 1, 0]),
        ];
        for cubie in &self.cubies {
            for axis in 0..3 {
                if cubie.home[axis] == 0 {
                    continue;
                }
                let mut local = [0, 0, 0];
                local[axis] = cubie.home[axis].signum();
                let world = Self::transform_local(cubie, local);
                if let Some((face, (_, horizontal, vertical))) = views
                    .iter()
                    .enumerate()
                    .find(|(_, (normal, _, _))| *normal == world)
                {
                    let column = Self::dot(cubie.position, *horizontal) + 1;
                    let row = 1 - Self::dot(cubie.position, *vertical);
                    let index = face * 9 + row as usize * 3 + column as usize;
                    facelets[index] = Self::sticker_color(local);
                }
            }
        }
        debug_assert!(facelets.iter().all(|&color| color != 255));
        facelets
    }

    fn snapshot_json(&self) -> Value {
        json!({
            "revision": self.revision,
            "facelets": self.facelets().to_vec(),
            "cubies": self.cubies.iter().map(|cubie| json!({
                "home": cubie.home,
                "position": cubie.position,
                "basis": cubie.basis,
            })).collect::<Vec<_>>(),
        })
    }
}

/// Server-side stand model: interpolates rail/gripper motion between status
/// updates and evaluates the collision rules.
struct SimState {
    rail_open_ms: f32,
    rail_grip_ms: f32,
    gripper_ms: f32,
    status: Option<link::StatusSnapshot>,
    last_solution: Option<link::SolutionStatus>,
    animation_speed: u32,
    rail_start: [Option<Instant>; 4],
    gripper_start: [Option<Instant>; 4],
    gripper_from: [Option<f32>; 4],
    gripper_angle: [Option<f32>; 4],
    rule1_active: bool,
    rule2_active: bool,
    concurrent_motion_active: bool,
    violations: Vec<String>,
    visual: VisualReplay,
    presentation: CubePresentation,
    cube: ServerCube,
    facelets: SharedFacelets,
    cube_dirty: bool,
    dirty: bool,
}

impl SimState {
    fn new(calibration: &StandCalibration, facelets: SharedFacelets) -> Self {
        Self {
            rail_open_ms: calibration.timing.rails_open_ms as f32,
            rail_grip_ms: calibration.timing.rails_grip_ms as f32,
            gripper_ms: calibration.timing.gripper_pose_ms as f32,
            status: None,
            last_solution: None,
            animation_speed: 1,
            rail_start: [None; 4],
            gripper_start: [None; 4],
            gripper_from: [None; 4],
            gripper_angle: [None; 4],
            rule1_active: false,
            rule2_active: false,
            concurrent_motion_active: false,
            violations: Vec::new(),
            visual: VisualReplay::default(),
            presentation: CubePresentation::default(),
            cube: ServerCube::solved(),
            facelets,
            cube_dirty: true,
            dirty: true,
        }
    }

    fn apply_status(&mut self, status: &link::StatusSnapshot) {
        let now = Instant::now();
        if status.solution.move_count > 0 {
            self.last_solution = Some(status.solution);
        }
        self.presentation.apply_status(status);
        if status.active_operation.is_none()
            && status.stand.pose.kind == link::StandPoseKind::CanonicalGrip
            && self.presentation.basis != [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            && !self
                .violations
                .iter()
                .any(|rule| rule == "scan-cube-pose-not-canonical")
        {
            self.violations
                .push("scan-cube-pose-not-canonical".to_owned());
        }
        for index in 0..4 {
            let rail = &status.stand.rails[index];
            if rail.motion == link::AxisMotion::Moving {
                if self.rail_start[index].is_none() {
                    self.rail_start[index] = Some(now);
                }
            } else {
                self.rail_start[index] = None;
            }
            let gripper = &status.stand.grippers[index];
            if gripper.motion == link::AxisMotion::Moving {
                if self.gripper_start[index].is_none() {
                    self.gripper_start[index] = Some(now);
                    self.gripper_from[index] = self.gripper_angle[index];
                }
            } else {
                self.gripper_start[index] = None;
                if let Some(orientation) = gripper.current {
                    self.gripper_angle[index] = Some(orientation_angle(orientation));
                }
            }
        }
        if let Some(cube_move) = self.visual.apply_status(status) {
            self.cube.apply_move(cube_move);
            *self.facelets.lock().unwrap() = self.cube.facelets();
            self.cube_dirty = true;
        }
        if let Some(detail) = self.visual.invariant_violation.take() {
            let violation = format!("layer-gripper-turn-mismatch: {detail}");
            if !self
                .violations
                .iter()
                .any(|existing| existing.starts_with("layer-gripper-turn-mismatch:"))
            {
                self.violations.push(violation);
            }
        }
        self.status = Some(*status);
        self.dirty = true;
    }

    /// Rail progress in `0.0..=1.0` (open..grip), interpolated while moving.
    fn rail_progress(&self, index: usize) -> f32 {
        let Some(status) = &self.status else {
            return 0.0;
        };
        let rail = &status.stand.rails[index];
        let rest = match rail.current {
            Some(link::RailPosition::Grip) => 1.0,
            _ => 0.0,
        };
        if rail.motion != link::AxisMotion::Moving {
            return rest;
        }
        let Some(start) = self.rail_start[index] else {
            return rest;
        };
        let to_grip = rail.target == Some(link::RailPosition::Grip);
        let duration = if to_grip {
            self.rail_grip_ms
        } else {
            self.rail_open_ms
        };
        let fraction = (start.elapsed().as_secs_f32() * 1000.0 * self.animation_speed as f32
            / duration)
            .clamp(0.0, 1.0);
        if to_grip {
            fraction
        } else {
            1.0 - fraction
        }
    }

    fn gripper_angles(&self) -> [Option<f32>; 4] {
        let mut angles = self.gripper_angle;
        let Some(status) = &self.status else {
            return angles;
        };
        for index in 0..4 {
            let gripper = &status.stand.grippers[index];
            if gripper.motion != link::AxisMotion::Moving {
                continue;
            }
            if let (Some(start), Some(target), Some(from)) = (
                self.gripper_start[index],
                gripper.target,
                self.gripper_from[index],
            ) {
                let duration = self.gripper_duration_ms(index, target);
                let fraction =
                    (start.elapsed().as_secs_f32() * 1000.0 * self.animation_speed as f32
                        / duration)
                        .clamp(0.0, 1.0);
                angles[index] = Some(from + (orientation_angle(target) - from) * fraction);
            }
        }
        angles
    }

    /// Claw-motion progress 0..=1 toward the current target pose.
    fn gripper_progress(&self, index: usize) -> f32 {
        let Some(status) = &self.status else {
            return 1.0;
        };
        let grip = &status.stand.grippers[index];
        if grip.motion != link::AxisMotion::Moving {
            return 1.0;
        }
        let Some(start) = self.gripper_start[index] else {
            return 0.0;
        };
        let duration = grip
            .target
            .map(|target| self.gripper_duration_ms(index, target))
            .unwrap_or(self.gripper_ms);
        (start.elapsed().as_secs_f32() * 1000.0 * self.animation_speed as f32 / duration)
            .clamp(0.0, 1.0)
    }

    fn gripper_duration_ms(&self, index: usize, target: link::GripperOrientation) -> f32 {
        let Some(from) = self.gripper_from[index] else {
            return self.gripper_ms;
        };
        let quarter_turns = ((orientation_angle(target) - from).abs()
            / std::f32::consts::FRAC_PI_2)
            .round()
            .max(1.0);
        self.gripper_ms * quarter_turns
    }

    fn any_axis_moving(&self) -> bool {
        let Some(status) = &self.status else {
            return false;
        };
        status
            .stand
            .rails
            .iter()
            .any(|axis| axis.motion == link::AxisMotion::Moving)
            || status
                .stand
                .grippers
                .iter()
                .any(|axis| axis.motion == link::AxisMotion::Moving)
    }

    fn status_json(&self) -> Value {
        let active_visual = self.visual.active.map(|active| {
            let progress = self.gripper_progress(active.axis);
            json!({
                "operation_id": active.operation_id,
                "move_index": active.move_index,
                "face": active.cube_move.face,
                "turns": active.cube_move.turns,
                "fraction": progress,
            })
        });
        let active_rigid = self.presentation.active.map(|active| {
            json!({
                "operation_id": active.operation_id,
                "axis": active.axis,
                "turns": active.turns,
                "fraction": self.gripper_progress(active.driver),
                "from_basis": self.presentation.basis,
                "to_basis": self.presentation.rotated_basis(active.axis, active.turns),
            })
        });
        json!({
            "type": "status",
            "status": self.status,
            "last_solution": self.last_solution,
            "animation_speed": self.animation_speed,
            "rails_progress": [
                self.rail_progress(0),
                self.rail_progress(1),
                self.rail_progress(2),
                self.rail_progress(3),
            ],
            "gripper_angle": self.gripper_angles(),
            "gripper_progress": [
                self.gripper_progress(0),
                self.gripper_progress(1),
                self.gripper_progress(2),
                self.gripper_progress(3),
            ],
            "safety": {
                "adjacent_gripper_collision": self.rule1_active,
                "cube_custody_lost": self.rule2_active,
                "concurrent_rail_gripper_motion": self.concurrent_motion_active,
                "violations": self.violations,
            },
            "visual": {
                "frame_quarters": self.visual.frame_quarters,
                "active_move": active_visual,
                "cube_pose": {
                    "basis": self.presentation.basis,
                    "active": active_rigid,
                },
            },
            "cube_revision": self.cube.revision,
        })
    }

    fn set_visual_moves(&mut self, moves: &[link::CubeMove]) {
        self.visual.set_expected(moves);
        if let Some(status) = self.status {
            if let Some(cube_move) = self.visual.apply_status(&status) {
                self.cube.apply_move(cube_move);
                *self.facelets.lock().unwrap() = self.cube.facelets();
                self.cube_dirty = true;
            }
        }
        self.dirty = true;
    }

    fn clear_last_solution(&mut self) {
        self.last_solution = None;
        self.dirty = true;
    }

    fn set_animation_speed(&mut self, multiplier: u32) {
        self.animation_speed = multiplier;
        self.dirty = true;
    }

    fn load_scramble(&mut self, moves: &[link::CubeMove]) {
        self.cube = ServerCube::solved();
        for cube_move in moves {
            self.cube.apply_move(VisualMove {
                face: cube_move.face,
                turns: match cube_move.turn {
                    link::TurnAmount::Clockwise => -1,
                    link::TurnAmount::CounterClockwise => 1,
                    link::TurnAmount::Half => 2,
                },
            });
        }
        *self.facelets.lock().unwrap() = self.cube.facelets();
        self.visual.set_expected(&[]);
        self.cube_dirty = true;
        self.dirty = true;
    }

    fn take_cube_event(&mut self) -> Option<Value> {
        if !self.cube_dirty {
            return None;
        }
        self.cube_dirty = false;
        Some(json!({
            "type": "cube",
            "cube": self.cube.snapshot_json(),
        }))
    }

    /// Evaluates both collision rules and returns transition events.
    ///
    /// Rule 1: adjacent gripper claws overlap only when both claws sit in a
    /// frame-parallel orientation while both rails are near the cube; legal
    /// poses never do this (a claw rotates parallel only after its rail
    /// opens, and opposite pairs are never adjacent).
    /// Rule 2: cube custody is lost when every rail releases while a cube
    /// session exists and no operation is running to explain it.
    fn collisions(&mut self) -> Vec<Value> {
        let Some(status) = self.status else {
            return Vec::new();
        };
        let progress: [f32; 4] = std::array::from_fn(|index| self.rail_progress(index));
        let angles = self.gripper_angles();
        let mut events = Vec::new();

        // 0.0 = exactly frame-parallel, 1.0 = perpendicular.
        let parallelness = |angle: f32| {
            (angle.abs().rem_euclid(std::f32::consts::PI) * 2.0 / std::f32::consts::PI).min(1.0)
        };
        let swung_flat = |index: usize| {
            angles[index].is_some_and(|angle| parallelness(angle) < 0.35) && progress[index] > 0.45
        };
        let pairs = [
            (0usize, 2usize, "left", "top"),
            (2, 1, "top", "right"),
            (1, 3, "right", "bottom"),
            (3, 0, "bottom", "left"),
        ];
        let mut overlap = None;
        for &(first, second, first_name, second_name) in &pairs {
            if swung_flat(first) && swung_flat(second) {
                overlap = Some(format!(
                    "{first_name} and {second_name} gripper claws overlap"
                ));
            }
        }
        if overlap.is_some() != self.rule1_active {
            self.rule1_active = overlap.is_some();
            events.push(json!({
                "type": "collision",
                "rule": "adjacent-parallel-grippers",
                "active": self.rule1_active,
                "description": overlap,
            }));
        }

        let intentional_release = status.active_operation.is_some_and(|operation| {
            matches!(
                operation.kind,
                link::OperationKind::Open | link::OperationKind::RecoverToOpen
            ) || (matches!(
                operation.kind,
                link::OperationKind::Execute | link::OperationKind::ScanSolveExecute
            ) && status.solution.completed_moves >= status.solution.move_count)
        });
        let held_left_right = progress[0] > 0.45 && progress[1] > 0.45;
        let held_top_bottom = progress[2] > 0.45 && progress[3] > 0.45;
        let custody_lost = status.cube_session.is_some()
            && !intentional_release
            && !held_left_right
            && !held_top_bottom;
        if custody_lost != self.rule2_active {
            self.rule2_active = custody_lost;
            events.push(json!({
                "type": "collision",
                "rule": "cube-custody-lost",
                "active": custody_lost,
                "description": custody_lost.then(|| {
                    format!(
                        "no opposing rail pair holds the cube; progress={progress:?}; operation={:?}",
                        status.active_operation.map(|operation| operation.kind)
                    )
                }),
            }));
        }

        // The mechanical plans intentionally serialize rail travel and claw
        // rotation. Overlap here means the simulator found a trajectory that
        // must not be deployed to the stand.
        let concurrent_motion = status
            .stand
            .rails
            .iter()
            .any(|axis| axis.motion == link::AxisMotion::Moving)
            && status
                .stand
                .grippers
                .iter()
                .any(|axis| axis.motion == link::AxisMotion::Moving);
        if concurrent_motion != self.concurrent_motion_active {
            self.concurrent_motion_active = concurrent_motion;
            events.push(json!({
                "type": "collision",
                "rule": "concurrent-rail-gripper-motion",
                "active": concurrent_motion,
                "description": concurrent_motion.then(|| {
                    "a rail and gripper are moving at the same time".to_owned()
                }),
            }));
        }

        for event in &events {
            if event.get("active").and_then(Value::as_bool) == Some(true) {
                if let Some(rule) = event.get("rule").and_then(Value::as_str) {
                    if !self.violations.iter().any(|seen| seen == rule) {
                        self.violations.push(rule.to_owned());
                    }
                }
            }
        }
        if !events.is_empty() {
            self.dirty = true;
        }
        events
    }
}

type Responders = Arc<Mutex<HashMap<u32, oneshot::Sender<Value>>>>;

/// Live facelet state of the displayed cube, posted by the UI after every
/// visual change. 54 entries in CubeFace order (U,R,F,D,L/B rows of 9),
/// values = StickerColor discriminants. The scanner reads this, so solving
/// works on the ACTUAL scrambled state shown on screen.
pub type SharedFacelets = Arc<Mutex<[u8; 54]>>;

pub fn new_facelets() -> SharedFacelets {
    // solved cube in link::CubeFace order
    Arc::new(Mutex::new([
        0, 0, 0, 0, 0, 0, 0, 0, 0, // U white
        2, 2, 2, 2, 2, 2, 2, 2, 2, // R red
        4, 4, 4, 4, 4, 4, 4, 4, 4, // F green
        1, 1, 1, 1, 1, 1, 1, 1, 1, // D yellow
        3, 3, 3, 3, 3, 3, 3, 3, 3, // L orange
        5, 5, 5, 5, 5, 5, 5, 5, 5, // B blue
    ]))
}

#[derive(Default)]
struct SimPwmOutput;

impl PwmOutput for SimPwmOutput {
    fn set_channels(&mut self, _channels: &[(u8, u16)]) -> Result<()> {
        Ok(())
    }

    fn disable_channels(&mut self, _channels: &[u8]) -> Result<()> {
        Ok(())
    }

    fn all_off(&mut self) -> Result<()> {
        Ok(())
    }
}

struct UiCubeScanner {
    facelets: SharedFacelets,
}

impl FaceScanner for UiCubeScanner {
    fn capture(&mut self, face: link::CubeFace) -> Result<link::RecognizedFace> {
        let base = face as usize * 9;
        let state = self.facelets.lock().unwrap();
        let mut colors = [link::StickerColor::Unknown; 9];
        for (index, color) in colors.iter_mut().enumerate() {
            *color = match state[base + index] {
                0 => link::StickerColor::White,
                1 => link::StickerColor::Yellow,
                2 => link::StickerColor::Red,
                3 => link::StickerColor::Orange,
                4 => link::StickerColor::Green,
                5 => link::StickerColor::Blue,
                _ => link::StickerColor::Unknown,
            };
        }
        Ok(link::RecognizedFace {
            colors,
            confidence: [255; 9],
        })
    }
}

struct SessionState {
    sim: Mutex<SimState>,
    subscribers: broadcast::Sender<String>,
    inbound: mpsc::Sender<Vec<u8>>,
    responders: Responders,
    encoder: Mutex<UartFrameEncoder>,
    next_request_id: AtomicU32,
    last_access: Mutex<Instant>,
    stop: Arc<AtomicBool>,
    animation_speed: Arc<AtomicU32>,
    server_instance: String,
}

struct AppState {
    calibration: StandCalibration,
    sessions: Mutex<HashMap<String, Arc<SessionState>>>,
    server_instance: String,
}

impl SessionState {
    fn broadcast(&self, text: String) {
        // A lack of subscribers is not an error; dropped frames are fine.
        let _ = self.subscribers.send(text);
    }
}

impl AppState {
    fn session(&self, id: &str) -> Arc<SessionState> {
        let mut sessions = self.sessions.lock().unwrap();
        let now = Instant::now();
        sessions.retain(|_, session| {
            let keep = now.duration_since(*session.last_access.lock().unwrap()) < SESSION_TTL;
            if !keep {
                session.stop.store(true, Ordering::SeqCst);
            }
            keep
        });
        if !sessions.contains_key(id) && sessions.len() >= MAX_SESSIONS {
            if let Some(oldest) = sessions
                .iter()
                .min_by_key(|(_, session)| *session.last_access.lock().unwrap())
                .map(|(id, _)| id.clone())
            {
                if let Some(session) = sessions.remove(&oldest) {
                    session.stop.store(true, Ordering::SeqCst);
                }
            }
        }
        let session = Arc::clone(
            sessions
                .entry(id.to_owned())
                .or_insert_with(|| spawn_session(&self.calibration, &self.server_instance)),
        );
        *session.last_access.lock().unwrap() = now;
        session
    }
}

fn spawn_session(calibration: &StandCalibration, server_instance: &str) -> Arc<SessionState> {
    let facelets = new_facelets();
    let (updates_tx, updates_rx) = mpsc::channel();
    let (inbound_tx, inbound_rx) = mpsc::channel();
    let (subscribers, _) = broadcast::channel(256);
    let stop = Arc::new(AtomicBool::new(false));
    let animation_speed = Arc::new(AtomicU32::new(1));
    let session = Arc::new(SessionState {
        sim: Mutex::new(SimState::new(calibration, Arc::clone(&facelets))),
        subscribers,
        inbound: inbound_tx,
        responders: Responders::default(),
        encoder: Mutex::new(UartFrameEncoder::default()),
        next_request_id: AtomicU32::new(FIRST_HTTP_REQUEST_ID),
        last_access: Mutex::new(Instant::now()),
        stop: Arc::clone(&stop),
        animation_speed: Arc::clone(&animation_speed),
        server_instance: server_instance.to_owned(),
    });

    std::thread::Builder::new()
        .name("sim-session-daemon".into())
        .spawn({
            let calibration = calibration.clone();
            move || {
                run_session_daemon(
                    calibration,
                    facelets,
                    inbound_rx,
                    updates_tx,
                    stop,
                    animation_speed,
                )
            }
        })
        .expect("failed to spawn simulation daemon");
    std::thread::Builder::new()
        .name("sim-session-pump".into())
        .spawn({
            let session = Arc::clone(&session);
            move || run_pump(updates_rx, session)
        })
        .expect("failed to spawn simulation pump");
    session
}

fn run_session_daemon(
    calibration: StandCalibration,
    facelets: SharedFacelets,
    inbound: mpsc::Receiver<Vec<u8>>,
    updates: mpsc::Sender<SimUpdate>,
    stop: Arc<AtomicBool>,
    animation_speed: Arc<AtomicU32>,
) {
    let mut service =
        RobotService::with_scanner(SimPwmOutput, calibration, UiCubeScanner { facelets });
    let mut decoder = UartStreamDecoder::default();
    let mut observer = SimEngine::new(updates);
    observer.observe_status(service.status());
    let mut simulated_now = Instant::now();
    let mut previous_wall = Instant::now();

    while !stop.load(Ordering::SeqCst) {
        match inbound.recv_timeout(Duration::from_millis(10)) {
            Ok(frame) => {
                for byte in frame {
                    match decoder.push(byte) {
                        Some(Ok(packet)) => {
                            let messages = service.handle_packet(&packet, simulated_now);
                            observer.observe_messages(&messages);
                        }
                        Some(Err(error)) => {
                            eprintln!("discarded simulation frame: {error:?}");
                        }
                        None => {}
                    }
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }

        let wall_now = Instant::now();
        let wall_elapsed = wall_now.saturating_duration_since(previous_wall);
        previous_wall = wall_now;
        simulated_now += wall_elapsed.mul_f64(animation_speed.load(Ordering::Relaxed) as f64);
        let messages = service.tick(simulated_now);
        observer.observe_status(service.status());
        observer.observe_messages(&messages);
    }
    let _ = service.shutdown();
}

#[derive(Deserialize, Default)]
struct SessionQuery {
    session: Option<String>,
}

fn session_id(query: &SessionQuery) -> std::result::Result<&str, &'static str> {
    let id = query.session.as_deref().unwrap_or("default");
    if id.is_empty()
        || id.len() > 80
        || !id
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_".contains(&byte))
    {
        return Err("invalid simulation session id");
    }
    Ok(id)
}

/// Bridges daemon telemetry into SSE broadcasts and pending command
/// responses. Runs on a dedicated OS thread; all channel sends are sync.
fn run_pump(updates: mpsc::Receiver<SimUpdate>, state: Arc<SessionState>) {
    while !state.stop.load(Ordering::SeqCst) {
        match updates.recv_timeout(TICK) {
            Ok(SimUpdate::Status(status)) => {
                state.sim.lock().unwrap().apply_status(&status);
            }
            Ok(SimUpdate::Event { opcode, payload }) => {
                state.broadcast(
                    json!({"type": "event", "opcode": opcode, "payload": payload}).to_string(),
                );
            }
            Ok(SimUpdate::Response {
                request_id,
                payload,
                ..
            }) => {
                if let Some(sender) = state.responders.lock().unwrap().remove(&request_id) {
                    let _ = sender.send(payload);
                }
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {}
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        }

        let mut sim = state.sim.lock().unwrap();
        for event in sim.collisions() {
            eprintln!("[sim:collision] {event}");
            state.broadcast(event.to_string());
        }
        if let Some(cube) = sim.take_cube_event() {
            state.broadcast(cube.to_string());
        }
        if sim.any_axis_moving() || (sim.dirty && sim.status.is_some()) {
            let mut status = sim.status_json();
            status["server_instance"] = json!(state.server_instance);
            state.broadcast(status.to_string());
            sim.dirty = false;
        }
        drop(sim);
    }
}

enum Command {
    Recover,
    Grip,
    Abort,
    Open {
        session_id: u32,
    },
    Scan {
        session_id: u32,
    },
    Solve {
        session_id: u32,
        scan_revision: u32,
    },
    Execute {
        session_id: u32,
        scan_revision: u32,
        solution_id: u32,
    },
    Auto {
        session_id: u32,
    },
    Moves {
        session_id: u32,
        sequence: String,
    },
    LoadScramble {
        sequence: String,
    },
    SetAnimationSpeed {
        multiplier: u32,
    },
}

fn field_u32(value: &Value, name: &str) -> std::result::Result<u32, String> {
    value
        .get(name)
        .and_then(Value::as_u64)
        .and_then(|raw| u32::try_from(raw).ok())
        .ok_or_else(|| format!("missing or invalid '{name}' field"))
}

fn parse_command(body: &str) -> std::result::Result<Command, String> {
    let value: Value =
        serde_json::from_str(body).map_err(|error| format!("invalid JSON: {error}"))?;
    let name = value
        .get("command")
        .and_then(Value::as_str)
        .ok_or("missing 'command' field")?;
    match name {
        "recover" => Ok(Command::Recover),
        "grip" => Ok(Command::Grip),
        "abort" => Ok(Command::Abort),
        "open" => Ok(Command::Open {
            session_id: field_u32(&value, "session_id")?,
        }),
        "scan" => Ok(Command::Scan {
            session_id: field_u32(&value, "session_id")?,
        }),
        "solve" => Ok(Command::Solve {
            session_id: field_u32(&value, "session_id")?,
            scan_revision: field_u32(&value, "scan_revision")?,
        }),
        "execute" => Ok(Command::Execute {
            session_id: field_u32(&value, "session_id")?,
            scan_revision: field_u32(&value, "scan_revision")?,
            solution_id: field_u32(&value, "solution_id")?,
        }),
        "auto" => Ok(Command::Auto {
            session_id: field_u32(&value, "session_id")?,
        }),
        "moves" => Ok(Command::Moves {
            session_id: field_u32(&value, "session_id")?,
            sequence: value
                .get("sequence")
                .and_then(Value::as_str)
                .ok_or("missing 'sequence' field")?
                .to_owned(),
        }),
        "load_scramble" => Ok(Command::LoadScramble {
            sequence: value
                .get("sequence")
                .and_then(Value::as_str)
                .ok_or("missing 'sequence' field")?
                .to_owned(),
        }),
        "set_animation_speed" => {
            let multiplier = field_u32(&value, "multiplier")?;
            if !matches!(multiplier, 1 | 2 | 4 | 8) {
                return Err("animation multiplier must be 1, 2, 4, or 8".to_owned());
            }
            Ok(Command::SetAnimationSpeed { multiplier })
        }
        other => Err(format!("unknown command {other:?}")),
    }
}

fn encode_with<T: serde::Serialize>(
    encoder: &mut UartFrameEncoder,
    request_id: u32,
    opcode: link::RequestOpcode,
    payload: &T,
) -> Result<Vec<u8>> {
    encoder
        .encode(
            link::MessageKind::Request,
            opcode.into(),
            request_id,
            payload,
        )
        .map(|frame| frame.to_vec())
        .map_err(|error| anyhow::anyhow!("{error}"))
}

fn normalize_sim_sequence(sequence: &str) -> Result<String> {
    let compact = sequence
        .chars()
        .filter(|character| !character.is_whitespace() && *character != ',')
        .collect::<Vec<_>>();
    let mut normalized = Vec::new();
    let mut index = 0;
    while index < compact.len() {
        let face = compact[index].to_ascii_uppercase();
        if !matches!(face, 'U' | 'R' | 'F' | 'D' | 'L' | 'B') {
            anyhow::bail!(
                "expected a face at character {}, got {:?}",
                index + 1,
                compact[index]
            );
        }
        index += 1;
        let suffix = compact
            .get(index)
            .copied()
            .filter(|value| matches!(value, '1' | '2' | '3' | '\''));
        if suffix.is_some() {
            index += 1;
        }
        let token = match suffix {
            None | Some('1') => face.to_string(),
            Some('2') => {
                if compact.get(index) == Some(&'\'') {
                    index += 1;
                }
                format!("{face}2")
            }
            Some('3' | '\'') => format!("{face}'"),
            Some(_) => unreachable!(),
        };
        normalized.push(token);
    }
    if normalized.is_empty() {
        anyhow::bail!("move sequence is empty");
    }
    Ok(normalized.join(" "))
}

fn encode_command(
    encoder: &mut UartFrameEncoder,
    request_id: u32,
    command: &Command,
) -> Result<Vec<u8>> {
    use link::{MessageKind, RequestOpcode};
    let mut empty = |opcode: RequestOpcode| {
        encoder
            .encode_empty(MessageKind::Request, opcode.into(), request_id)
            .map(|frame| frame.to_vec())
            .map_err(|error| anyhow::anyhow!("{error}"))
    };
    match command {
        Command::Recover => empty(RequestOpcode::RecoverToOpen),
        Command::Grip => empty(RequestOpcode::Grip),
        Command::Abort => empty(RequestOpcode::Abort),
        Command::Open { session_id } => encode_with(
            encoder,
            request_id,
            RequestOpcode::Open,
            &link::OpenCommand {
                session_id: *session_id,
            },
        ),
        Command::Scan { session_id } => encode_with(
            encoder,
            request_id,
            RequestOpcode::StartScan,
            &link::StartScanCommand {
                session_id: *session_id,
            },
        ),
        Command::Solve {
            session_id,
            scan_revision,
        } => encode_with(
            encoder,
            request_id,
            RequestOpcode::Solve,
            &link::SolveCommand {
                session_id: *session_id,
                scan_revision: *scan_revision,
            },
        ),
        Command::Execute {
            session_id,
            scan_revision,
            solution_id,
        } => encode_with(
            encoder,
            request_id,
            RequestOpcode::Execute,
            &link::ExecuteCommand {
                session_id: *session_id,
                scan_revision: *scan_revision,
                solution_id: *solution_id,
            },
        ),
        Command::Auto { session_id } => encode_with(
            encoder,
            request_id,
            RequestOpcode::ScanSolveExecute,
            &link::ScanSolveExecuteCommand {
                session_id: *session_id,
            },
        ),
        Command::Moves {
            session_id,
            sequence,
        } => {
            let normalized = normalize_sim_sequence(sequence)?;
            let (moves, move_count) = link::parse_singmaster(&normalized)
                .map_err(|error| anyhow::anyhow!("invalid sequence: {error:?}"))?;
            encode_with(
                encoder,
                request_id,
                RequestOpcode::ExecuteMoves,
                &link::ExecuteMovesCommand {
                    session_id: *session_id,
                    moves,
                    move_count,
                },
            )
        }
        Command::LoadScramble { .. } => {
            anyhow::bail!("load_scramble is handled directly by the simulator")
        }
        Command::SetAnimationSpeed { .. } => {
            anyhow::bail!("set_animation_speed is handled directly by the simulator")
        }
    }
}

async fn sse_events(
    State(state): State<Arc<AppState>>,
    Query(query): Query<SessionQuery>,
) -> Response {
    let id = match session_id(&query) {
        Ok(id) => id,
        Err(error) => return error_response(StatusCode::BAD_REQUEST, error),
    };
    let session = state.session(id);
    let rx = session.subscribers.subscribe();
    let hello = axum::response::sse::Event::default().data(
        json!({
            "type": "server",
            "server_instance": session.server_instance,
        })
        .to_string(),
    );
    let updates = BroadcastStream::new(rx).filter_map(|message| match message {
        Ok(text) => Some(Ok::<_, Infallible>(
            axum::response::sse::Event::default().data(text),
        )),
        Err(_) => None, // lagged receiver: skip, next status catches up
    });
    let stream = tokio_stream::once(Ok::<_, Infallible>(hello)).chain(updates);
    Sse::new(stream)
        .keep_alive(
            KeepAlive::new()
                .interval(Duration::from_secs(5))
                .text("keep-alive"),
        )
        .into_response()
}

async fn api_status(
    State(state): State<Arc<AppState>>,
    Query(query): Query<SessionQuery>,
) -> Response {
    let id = match session_id(&query) {
        Ok(id) => id,
        Err(error) => return error_response(StatusCode::BAD_REQUEST, error),
    };
    let session = state.session(id);
    let mut body = session.sim.lock().unwrap().status_json();
    body["server_instance"] = json!(session.server_instance);
    (
        [(header::CONTENT_TYPE, "application/json")],
        body.to_string(),
    )
        .into_response()
}

async fn post_command(
    State(state): State<Arc<AppState>>,
    Query(query): Query<SessionQuery>,
    body: String,
) -> Response {
    let id = match session_id(&query) {
        Ok(id) => id,
        Err(error) => return error_response(StatusCode::BAD_REQUEST, error),
    };
    let state = state.session(id);
    let command = match parse_command(&body) {
        Ok(command) => command,
        Err(error) => return error_response(StatusCode::BAD_REQUEST, &error),
    };

    if let Command::SetAnimationSpeed { multiplier } = &command {
        state.animation_speed.store(*multiplier, Ordering::Relaxed);
        state.sim.lock().unwrap().set_animation_speed(*multiplier);
        return (
            StatusCode::OK,
            axum::Json(json!({
                "ok": true,
                "response": { "animation_speed": multiplier },
            })),
        )
            .into_response();
    }

    if let Command::LoadScramble { sequence } = &command {
        let normalized = match normalize_sim_sequence(sequence) {
            Ok(normalized) => normalized,
            Err(error) => {
                return error_response(
                    StatusCode::BAD_REQUEST,
                    &format!("invalid scramble: {error:#}"),
                )
            }
        };
        let (moves, count) = match link::parse_singmaster(&normalized) {
            Ok(parsed) => parsed,
            Err(error) => {
                return error_response(
                    StatusCode::BAD_REQUEST,
                    &format!("invalid scramble: {error:?}"),
                )
            }
        };
        let mut sim = state.sim.lock().unwrap();
        let ready = sim.status.is_some_and(|status| {
            status.controller == link::ControllerState::Ready && status.active_operation.is_none()
        });
        if !ready || sim.any_axis_moving() {
            return error_response(
                StatusCode::CONFLICT,
                "cannot load a scramble while the robot is moving",
            );
        }
        sim.load_scramble(&moves[..usize::from(count)]);
        let cube = sim.cube.snapshot_json();
        return (
            StatusCode::OK,
            axum::Json(json!({
                "ok": true,
                "response": { "loaded_moves": count, "cube": cube },
            })),
        )
            .into_response();
    }

    let request_id = state
        .next_request_id
        .fetch_update(Ordering::SeqCst, Ordering::SeqCst, |id| {
            Some(if id >= LAST_HTTP_REQUEST_ID {
                FIRST_HTTP_REQUEST_ID
            } else {
                id + 1
            })
        });
    let request_id = request_id.unwrap_or(FIRST_HTTP_REQUEST_ID);

    let frame = {
        let mut encoder = state.encoder.lock().unwrap();
        match encode_command(&mut encoder, request_id, &command) {
            Ok(frame) => frame,
            Err(error) => return error_response(StatusCode::BAD_REQUEST, &format!("{error:#}")),
        }
    };

    let (tx, rx) = oneshot::channel();
    state.responders.lock().unwrap().insert(request_id, tx);
    if state.inbound.send(frame).is_err() {
        state.responders.lock().unwrap().remove(&request_id);
        return error_response(
            StatusCode::SERVICE_UNAVAILABLE,
            "daemon is not accepting commands",
        );
    }

    match tokio::time::timeout(COMMAND_TIMEOUT, rx).await {
        Ok(Ok(payload)) => {
            if payload
                .get("operation_id")
                .and_then(Value::as_u64)
                .is_some()
                && matches!(&command, Command::Scan { .. } | Command::Auto { .. })
            {
                state.sim.lock().unwrap().clear_last_solution();
            }
            if payload
                .get("operation_id")
                .and_then(Value::as_u64)
                .is_some()
            {
                let visual_moves = match &command {
                    Command::Moves { sequence, .. } => {
                        let normalized = normalize_sim_sequence(sequence)
                            .expect("accepted sequence was already simulator-validated");
                        let (moves, count) = link::parse_singmaster(&normalized)
                            .expect("accepted sequence was already protocol-validated");
                        Some(moves[..usize::from(count)].to_vec())
                    }
                    Command::Execute { .. } => {
                        let sim = state.sim.lock().unwrap();
                        sim.status.as_ref().map(|status| {
                            status.solution.moves[..usize::from(status.solution.move_count)]
                                .to_vec()
                        })
                    }
                    _ => None,
                };
                if let Some(moves) = visual_moves {
                    state.sim.lock().unwrap().set_visual_moves(&moves);
                }
            }
            (
                StatusCode::OK,
                axum::Json(json!({"ok": true, "response": payload})),
            )
                .into_response()
        }
        Ok(Err(_)) | Err(_) => error_response(
            StatusCode::GATEWAY_TIMEOUT,
            "timed out waiting for a controller response",
        ),
    }
}

async fn get_cube(
    State(state): State<Arc<AppState>>,
    Query(query): Query<SessionQuery>,
) -> Response {
    let id = match session_id(&query) {
        Ok(id) => id,
        Err(error) => return error_response(StatusCode::BAD_REQUEST, error),
    };
    let session = state.session(id);
    let cube = session.sim.lock().unwrap().cube.snapshot_json();
    (StatusCode::OK, axum::Json(cube)).into_response()
}

fn error_response(status: StatusCode, message: &str) -> Response {
    (status, axum::Json(json!({ "ok": false, "error": message }))).into_response()
}

/// Runs a multi-session simulation server. Every browser session gets an
/// independent protocol service, stand, scanner, cube, SSE stream and safety
/// monitor; no command or state is shared between tabs.
pub fn run_sim_server(addr: &str, calibration: StandCalibration) -> Result<()> {
    let started = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos();
    let state = Arc::new(AppState {
        calibration,
        sessions: Mutex::new(HashMap::new()),
        server_instance: format!("{}-{started}", std::process::id()),
    });

    let runtime = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;

    runtime.block_on(async move {
        let app = Router::new()
            .route(
                "/",
                get(|| async {
                    (
                        [
                            (header::CONTENT_TYPE, "text/html; charset=utf-8"),
                            (header::CACHE_CONTROL, "no-store"),
                        ],
                        Html(SIM_HTML),
                    )
                }),
            )
            .route(
                "/three.js",
                get(|| async {
                    (
                        [
                            (header::CONTENT_TYPE, "text/javascript; charset=utf-8"),
                            (header::CACHE_CONTROL, "no-store"),
                        ],
                        THREE_JS,
                    )
                }),
            )
            .route(
                "/gripper.stl",
                get(|| async {
                    (
                        [
                            (header::CONTENT_TYPE, "application/octet-stream"),
                            (header::CACHE_CONTROL, "no-store"),
                        ],
                        GRIPPER_STL.to_vec(),
                    )
                }),
            )
            .route("/events", get(sse_events))
            .route("/api/status", get(api_status))
            .route("/api/cube", get(get_cube))
            .route("/command", post(post_command))
            .with_state(state);

        let listener = tokio::net::TcpListener::bind(addr).await.map_err(|error| {
            anyhow::anyhow!("failed to bind simulation HTTP server on {addr}: {error}")
        })?;
        eprintln!("simulation UI available at http://{addr}/");
        axum::serve(listener, app)
            .await
            .map_err(|error| anyhow::anyhow!("HTTP server failed: {error}"))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn server_cube_moves_round_trip_without_browser_state() {
        let mut cube = ServerCube::solved();
        let solved = cube.facelets();
        for cube_move in [
            VisualMove {
                face: link::CubeFace::Right,
                turns: -1,
            },
            VisualMove {
                face: link::CubeFace::Up,
                turns: -1,
            },
            VisualMove {
                face: link::CubeFace::Front,
                turns: 1,
            },
            VisualMove {
                face: link::CubeFace::Down,
                turns: -1,
            },
        ] {
            cube.apply_move(cube_move);
        }
        assert_ne!(cube.facelets(), solved);
        for cube_move in [
            VisualMove {
                face: link::CubeFace::Down,
                turns: 1,
            },
            VisualMove {
                face: link::CubeFace::Front,
                turns: -1,
            },
            VisualMove {
                face: link::CubeFace::Up,
                turns: 1,
            },
            VisualMove {
                face: link::CubeFace::Right,
                turns: 1,
            },
        ] {
            cube.apply_move(cube_move);
        }
        assert_eq!(cube.facelets(), solved);
    }

    #[test]
    fn server_cube_facelets_match_min2phase_notation() {
        let mut cube = ServerCube::solved();
        cube.apply_move(VisualMove {
            face: link::CubeFace::Right,
            turns: -1,
        });
        cube.apply_move(VisualMove {
            face: link::CubeFace::Down,
            turns: -1,
        });
        let symbols = cube
            .facelets()
            .map(|color| match color {
                0 => 'U',
                2 => 'R',
                4 => 'F',
                1 => 'D',
                3 => 'L',
                5 => 'B',
                _ => unreachable!(),
            })
            .into_iter()
            .collect::<String>();
        assert_eq!(symbols, min2phase::from_moves(&"R D".to_owned()).unwrap());
    }

    #[test]
    fn compact_numbered_scramble_is_normalized_to_singmaster() {
        assert_eq!(
            normalize_sim_sequence("B2L1B3 R1,U3").unwrap(),
            "B2 L B' R U'"
        );
    }

    #[test]
    fn rejects_unsafe_session_identifiers() {
        assert!(session_id(&SessionQuery {
            session: Some("tab-1".into())
        })
        .is_ok());
        assert!(session_id(&SessionQuery {
            session: Some("../shared".into())
        })
        .is_err());
        assert!(session_id(&SessionQuery {
            session: Some(String::new())
        })
        .is_err());
    }
}
