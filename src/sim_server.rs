//! Embedded HTTP server for the robot simulator.
//!
//! Serves the 3D operator UI, streams live status over server-sent events,
//! injects protocol commands into the daemon, and reports mechanical
//! collisions (adjacent parallel grippers, lost cube custody).

use crate::robot_link::UartFrameEncoder;
use crate::robot_daemon::DaemonObserver;
use crate::robot_service::{EventMessage, ResponseMessage, ResponsePayload, ServiceMessage};
use crate::stand::StandCalibration;
use anyhow::Result;
use rubik_link_protocol as link;
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::Read;
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};
use tiny_http::{Header, Response, Server};

pub const SIM_HTML: &str = include_str!("../web/sim.html");
pub const THREE_JS: &str = include_str!("../web/three.module.min.js");

const FIRST_HTTP_REQUEST_ID: u32 = 0x0100_0000;
const LAST_HTTP_REQUEST_ID: u32 = 0x0fff_ffff;
const COMMAND_TIMEOUT: Duration = Duration::from_secs(10);
const TICK: Duration = Duration::from_millis(20);

/// Telemetry forwarded from the daemon thread to the HTTP thread.
pub enum SimUpdate {
    Status(Box<link::StatusSnapshot>),
    Event { opcode: u16, payload: Value },
    Response {
        request_id: u32,
        opcode: u16,
        payload: Value,
    },
}

/// [`DaemonObserver`] side: forwards status changes and messages to the
/// HTTP thread. Runs on the daemon thread; must never block.
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

/// Server-side stand model: interpolates rail/gripper motion between status
/// updates and evaluates the collision rules.
struct SimState {
    rail_open_ms: f32,
    rail_grip_ms: f32,
    gripper_ms: f32,
    status: Option<link::StatusSnapshot>,
    rail_start: [Option<Instant>; 4],
    gripper_start: [Option<Instant>; 4],
    gripper_from: [f32; 4],
    gripper_angle: [f32; 4],
    rule1_active: bool,
    rule2_active: bool,
    dirty: bool,
}

impl SimState {
    fn new(calibration: &StandCalibration) -> Self {
        Self {
            rail_open_ms: calibration.timing.rails_open_ms as f32,
            rail_grip_ms: calibration.timing.rails_grip_ms as f32,
            gripper_ms: calibration.timing.gripper_pose_ms as f32,
            status: None,
            rail_start: [None; 4],
            gripper_start: [None; 4],
            gripper_from: [0.0; 4],
            gripper_angle: [0.0; 4],
            rule1_active: false,
            rule2_active: false,
            dirty: true,
        }
    }

    fn apply_status(&mut self, status: &link::StatusSnapshot) {
        let now = Instant::now();
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
                    self.gripper_angle[index] = orientation_angle(orientation);
                }
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
        let fraction = (start.elapsed().as_secs_f32() * 1000.0 / duration).clamp(0.0, 1.0);
        if to_grip {
            fraction
        } else {
            1.0 - fraction
        }
    }

    fn gripper_angles(&self) -> [f32; 4] {
        let mut angles = self.gripper_angle;
        let Some(status) = &self.status else {
            return angles;
        };
        for index in 0..4 {
            let gripper = &status.stand.grippers[index];
            if gripper.motion != link::AxisMotion::Moving {
                continue;
            }
            if let (Some(start), Some(target)) = (self.gripper_start[index], gripper.target) {
                let fraction =
                    (start.elapsed().as_secs_f32() * 1000.0 / self.gripper_ms).clamp(0.0, 1.0);
                angles[index] = self.gripper_from[index]
                    + (orientation_angle(target) - self.gripper_from[index]) * fraction;
            }
        }
        angles
    }

    fn any_axis_moving(&self) -> bool {
        let Some(status) = &self.status else {
            return false;
        };
        status.stand.rails.iter().any(|axis| axis.motion == link::AxisMotion::Moving)
            || status.stand.grippers.iter().any(|axis| axis.motion == link::AxisMotion::Moving)
    }

    fn status_json(&self) -> Value {
        json!({
            "type": "status",
            "status": self.status,
            "rails_progress": [
                self.rail_progress(0),
                self.rail_progress(1),
                self.rail_progress(2),
                self.rail_progress(3),
            ],
            "gripper_angle": self.gripper_angles(),
        })
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
        let parallelness = |angle: f32| (angle.abs().rem_euclid(std::f32::consts::PI) * 2.0
            / std::f32::consts::PI)
            .min(1.0);
        let swung_flat = |index: usize| {
            parallelness(angles[index]) < 0.35 && progress[index] > 0.45
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

        let custody_lost = status.active_operation.is_none()
            && progress.iter().all(|&value| value < 0.1)
            && status.cube_session.is_some();
        if custody_lost != self.rule2_active {
            self.rule2_active = custody_lost;
            events.push(json!({
                "type": "collision",
                "rule": "cube-custody-lost",
                "active": custody_lost,
                "description": custody_lost.then(|| {
                    "all rails released while a cube session is active".to_owned()
                }),
            }));
        }

        if !events.is_empty() {
            self.dirty = true;
        }
        events
    }
}

type Subscribers = Arc<Mutex<Vec<mpsc::Sender<String>>>>;
type Responders = Arc<Mutex<HashMap<u32, mpsc::Sender<Value>>>>;

fn broadcast(subscribers: &Subscribers, text: &str) {
    let mut guards = subscribers.lock().unwrap();
    guards.retain(|sender| sender.send(text.to_owned()).is_ok());
}

/// Blocking reader handed to tiny-http for one SSE connection.
struct SseStream {
    rx: mpsc::Receiver<String>,
    pending: Vec<u8>,
    offset: usize,
}

impl Read for SseStream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if buf.is_empty() {
            return Ok(0);
        }
        loop {
            if self.offset < self.pending.len() {
                let count = buf.len().min(self.pending.len() - self.offset);
                buf[..count].copy_from_slice(&self.pending[self.offset..self.offset + count]);
                self.offset += count;
                if self.offset >= self.pending.len() {
                    self.pending.clear();
                    self.offset = 0;
                }
                return Ok(count);
            }
            match self.rx.recv() {
                Ok(text) => {
                    self.pending = format!("data: {text}\n\n").into_bytes();
                    self.offset = 0;
                }
                Err(_) => return Ok(0),
            }
        }
    }
}

enum Command {
    Status,
    Recover,
    Grip,
    Abort,
    Open { session_id: u32 },
    Scan { session_id: u32 },
    Solve {
        session_id: u32,
        scan_revision: u32,
    },
    Execute {
        session_id: u32,
        scan_revision: u32,
        solution_id: u32,
    },
    Auto { session_id: u32 },
    Moves {
        session_id: u32,
        sequence: String,
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
        "status" => Ok(Command::Status),
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
        Command::Status => empty(RequestOpcode::GetStatus),
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
            let (moves, move_count) = link::parse_singmaster(sequence)
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
    }
}

fn header(name: &str, value: &str) -> Header {
    Header::from_bytes(name.as_bytes(), value.as_bytes())
        .expect("static header is always valid")
}

fn respond_static(request: tiny_http::Request, content_type: &str, body: &'static str) {
    let response = Response::from_string(body)
        .with_header(header("Content-Type", content_type))
        .with_header(header("Cache-Control", "no-cache"));
    let _ = request.respond(response);
}

fn respond_json(request: tiny_http::Request, status: u16, body: &Value) {
    let response = Response::from_string(body.to_string())
        .with_status_code(tiny_http::StatusCode(status))
        .with_header(header("Content-Type", "application/json"));
    let _ = request.respond(response);
}

fn respond_error(request: tiny_http::Request, status: u16, message: &str) {
    respond_json(request, status, &json!({ "ok": false, "error": message }));
}

fn spawn_command_waiter(request: tiny_http::Request, rx: mpsc::Receiver<Value>) {
    std::thread::spawn(move || {
        let (status, payload) = match rx.recv_timeout(COMMAND_TIMEOUT) {
            Ok(value) => (200u16, json!({ "ok": true, "response": value })),
            Err(_) => (
                504u16,
                json!({"ok": false, "error": "timed out waiting for a controller response"}),
            ),
        };
        respond_json(request, status, &payload);
    });
}

fn handle_request(
    mut request: tiny_http::Request,
    state: &mut SimState,
    subscribers: &Subscribers,
    responders: &Responders,
    inbound: &mpsc::Sender<Vec<u8>>,
    encoder: &mut UartFrameEncoder,
    next_request_id: &mut u32,
) {
    let method = request.method().as_str().to_owned();
    let url = request.url().split('?').next().unwrap_or("/").to_owned();
    match (method.as_str(), url.as_str()) {
        ("GET", "/") => respond_static(request, "text/html; charset=utf-8", SIM_HTML),
        ("GET", "/three.js") => {
            respond_static(request, "text/javascript; charset=utf-8", THREE_JS)
        }
        ("GET", "/events") => {
            // SSE responses block until the client disconnects, so they must
            // not occupy the shared accept loop.
            let subscribers = Arc::clone(subscribers);
            std::thread::spawn(move || {
                let (tx, rx) = mpsc::channel();
                subscribers.lock().unwrap().push(tx);
                let response = Response::new(
                    tiny_http::StatusCode(200),
                    vec![
                        header("Content-Type", "text/event-stream"),
                        header("Cache-Control", "no-cache"),
                    ],
                    SseStream {
                        rx,
                        pending: Vec::new(),
                        offset: 0,
                    },
                    None,
                    None,
                );
                let _ = request.respond(response);
            });
        }
        ("GET", "/api/status") => {
            let body = state.status_json();
            respond_json(request, 200, &body);
        }
        ("POST", "/command") => {
            let mut body = String::new();
            let _ = request.as_reader().take(65_536).read_to_string(&mut body);
            match parse_command(&body) {
                Ok(command) => {
                    let request_id = *next_request_id;
                    *next_request_id = if *next_request_id >= LAST_HTTP_REQUEST_ID {
                        FIRST_HTTP_REQUEST_ID
                    } else {
                        *next_request_id + 1
                    };
                    match encode_command(encoder, request_id, &command) {
                        Ok(frame) => {
                            let (tx, rx) = mpsc::channel();
                            responders.lock().unwrap().insert(request_id, tx);
                            if inbound.send(frame).is_err() {
                                responders.lock().unwrap().remove(&request_id);
                                respond_error(request, 503, "daemon is not accepting commands");
                                return;
                            }
                            spawn_command_waiter(request, rx);
                        }
                        Err(error) => respond_error(request, 400, &format!("{error:#}")),
                    }
                }
                Err(error) => respond_error(request, 400, &error),
            }
        }
        _ => respond_error(request, 404, "not found"),
    }
}

/// Runs until the process exits. `updates` carries daemon telemetry,
/// `inbound` forwards encoded command frames into the daemon loop.
pub fn run_sim_server(
    addr: &str,
    updates: mpsc::Receiver<SimUpdate>,
    inbound: mpsc::Sender<Vec<u8>>,
    calibration: &StandCalibration,
) -> Result<()> {
    let server = Server::http(addr).map_err(|error| {
        anyhow::anyhow!("failed to bind simulation HTTP server on {addr}: {error}")
    })?;
    eprintln!("simulation UI available at http://{addr}/");
    let subscribers: Subscribers = Arc::default();
    let responders: Responders = Arc::default();
    let mut state = SimState::new(calibration);
    let mut encoder = UartFrameEncoder::default();
    let mut next_request_id = FIRST_HTTP_REQUEST_ID;

    loop {
        while let Some(request) = server
            .try_recv()
            .map_err(|error| anyhow::anyhow!("HTTP accept failed: {error}"))?
        {
            handle_request(
                request,
                &mut state,
                &subscribers,
                &responders,
                &inbound,
                &mut encoder,
                &mut next_request_id,
            );
        }

        while let Ok(update) = updates.try_recv() {
            match update {
                SimUpdate::Status(status) => state.apply_status(&status),
                SimUpdate::Event { opcode, payload } => {
                    broadcast(
                        &subscribers,
                        &json!({"type": "event", "opcode": opcode, "payload": payload})
                            .to_string(),
                    );
                }
                SimUpdate::Response {
                    request_id,
                    opcode,
                    payload,
                } => {
                    let responder = responders.lock().unwrap().remove(&request_id);
                    if let Some(sender) = responder {
                        let _ = sender.send(json!({"opcode": opcode, "payload": payload}));
                    }
                }
            }
        }

        for event in state.collisions() {
            eprintln!("[sim:collision] {event}");
            broadcast(&subscribers, &event.to_string());
        }
        if state.any_axis_moving() || (state.dirty && state.status.is_some()) {
            broadcast(&subscribers, &state.status_json().to_string());
            state.dirty = false;
        }

        std::thread::sleep(TICK);
    }
}
