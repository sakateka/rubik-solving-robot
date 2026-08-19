use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};

pub const AXIS_COUNT: usize = 4;
pub const FACE_COUNT: usize = 6;
pub const STICKERS_PER_FACE: usize = 9;
pub const MAX_SOLUTION_MOVES: usize = 32;
pub const MAX_PLAN_PREVIEW_ACTIONS: usize = 16;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SchemaError {
    InvalidScannedFaceMask(u8),
    TooManySolutionMoves(u8),
    CompletedMovesExceedTotal { completed: u8, total: u8 },
    TooManyPreviewActions(u8),
    TooManyRequestedMoves(u8),
    CurrentActionExceedsTotal { current: u16, total: u16 },
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum Axis {
    Left = 0,
    Right = 1,
    Top = 2,
    Bottom = 3,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum CubeFace {
    Up = 0,
    Right = 1,
    Front = 2,
    Down = 3,
    Left = 4,
    Back = 5,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum StickerColor {
    White = 0,
    Yellow = 1,
    Red = 2,
    Orange = 3,
    Green = 4,
    Blue = 5,
    Unknown = 255,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum RailPosition {
    Open = 0,
    Grip = 1,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum GripperOrientation {
    FrameParallel = 0,
    FramePerpendicular = 1,
    FrameParallelReversed = 2,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum AxisMotion {
    Unknown = 0,
    Stable = 1,
    Moving = 2,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct RailStatus {
    pub motion: AxisMotion,
    pub current: Option<RailPosition>,
    pub target: Option<RailPosition>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct GripperStatus {
    pub motion: AxisMotion,
    pub current: Option<GripperOrientation>,
    pub target: Option<GripperOrientation>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct StandState {
    pub pose: StandPose,
    pub rails: [RailStatus; AXIS_COUNT],
    pub grippers: [GripperStatus; AXIS_COUNT],
    pub outputs_enabled: bool,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum StandPoseKind {
    Unknown = 0,
    Open = 1,
    CanonicalGrip = 2,
    ScanPose = 3,
    MovePose = 4,
    Transitional = 5,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct StandPose {
    pub kind: StandPoseKind,
    /// Known logical face looking into the camera; absent when not meaningful.
    pub camera_face: Option<CubeFace>,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum ControllerState {
    Booting = 0,
    Ready = 1,
    Busy = 2,
    Aborted = 3,
    Faulted = 4,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum OperationKind {
    Grip = 0,
    Scan = 1,
    Solve = 2,
    Execute = 3,
    ScanSolveExecute = 4,
    Open = 5,
    RecoverToOpen = 6,
    ExecuteMoves = 7,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CubeSessionStatus {
    pub id: u32,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum ScanStateKind {
    None = 0,
    InProgress = 1,
    Valid = 2,
    Invalid = 3,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum ScanValidationError {
    WrongDetectionCount = 0,
    WrongColorCount = 1,
    DuplicateCenterColor = 2,
    InvalidFacelet = 3,
    CameraFailure = 4,
    InferenceFailure = 5,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct RecognizedFace {
    pub colors: [StickerColor; STICKERS_PER_FACE],
    pub confidence: [u8; STICKERS_PER_FACE],
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ScanStatus {
    pub state: ScanStateKind,
    pub revision: Option<u32>,
    pub current_face: Option<CubeFace>,
    pub camera_face: Option<RecognizedFace>,
    /// Bit `CubeFace as u8` is set after that face has been scanned.
    pub scanned_faces: u8,
    pub faces: [Option<RecognizedFace>; FACE_COUNT],
    pub color_counts: [u8; FACE_COUNT],
    pub validation_error: Option<ScanValidationError>,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum TurnAmount {
    Clockwise = 0,
    CounterClockwise = 1,
    Half = 2,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CubeMove {
    pub face: CubeFace,
    pub turn: TurnAmount,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum SolutionStateKind {
    None = 0,
    Solving = 1,
    Ready = 2,
    Executing = 3,
    Completed = 4,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SolutionStatus {
    pub state: SolutionStateKind,
    pub id: Option<u32>,
    pub source_scan_revision: Option<u32>,
    pub moves: [CubeMove; MAX_SOLUTION_MOVES],
    pub move_count: u8,
    pub completed_moves: u8,
}

impl SolutionStatus {
    pub fn validate(&self) -> Result<(), SchemaError> {
        if usize::from(self.move_count) > MAX_SOLUTION_MOVES {
            return Err(SchemaError::TooManySolutionMoves(self.move_count));
        }
        if self.completed_moves > self.move_count {
            return Err(SchemaError::CompletedMovesExceedTotal {
                completed: self.completed_moves,
                total: self.move_count,
            });
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum MechanicalActionKind {
    SetRail = 0,
    SetGripper = 1,
    Wait = 2,
    CaptureFace = 3,
    RecognizeFace = 4,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum ActionState {
    Pending = 0,
    Running = 1,
    Completed = 2,
    Cancelled = 3,
    Failed = 4,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct MechanicalAction {
    pub id: u32,
    pub kind: MechanicalActionKind,
    pub state: ActionState,
    pub axis: Option<Axis>,
    pub rail_target: Option<RailPosition>,
    pub gripper_target: Option<GripperOrientation>,
    pub face: Option<CubeFace>,
    pub duration_ms: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct OperationStatus {
    pub id: u32,
    pub kind: OperationKind,
    pub current_action: u16,
    pub action_count: u16,
}

impl OperationStatus {
    pub fn validate(&self) -> Result<(), SchemaError> {
        if self.current_action > self.action_count {
            return Err(SchemaError::CurrentActionExceedsTotal {
                current: self.current_action,
                total: self.action_count,
            });
        }
        Ok(())
    }
}

impl ScanStatus {
    pub fn validate(&self) -> Result<(), SchemaError> {
        const ALL_FACES_MASK: u8 = (1 << FACE_COUNT) - 1;
        if self.scanned_faces & !ALL_FACES_MASK != 0 {
            return Err(SchemaError::InvalidScannedFaceMask(self.scanned_faces));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum FaultCode {
    InvalidState = 0,
    I2c = 1,
    Camera = 2,
    Tpu = 3,
    Recognition = 4,
    InvalidFacelet = 5,
    Solver = 6,
    MotionPlan = 7,
    Internal = 255,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct RobotFault {
    pub code: FaultCode,
    /// Component-specific numeric detail; zero when unavailable.
    pub detail: u32,
}
