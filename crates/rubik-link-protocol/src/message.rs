use serde::{Deserialize, Serialize};
use serde_repr::{Deserialize_repr, Serialize_repr};

use crate::state::{
    ControllerState, CubeFace, CubeMove, CubeSessionStatus, MechanicalAction, OperationKind,
    OperationStatus, RecognizedFace, RobotFault, ScanStatus, SchemaError, SolutionStatus,
    StandState, MAX_PLAN_PREVIEW_ACTIONS, MAX_SOLUTION_MOVES,
};

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct StartScanCommand {
    pub session_id: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct SolveCommand {
    pub session_id: u32,
    pub scan_revision: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ExecuteCommand {
    pub session_id: u32,
    pub scan_revision: u32,
    pub solution_id: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ScanSolveExecuteCommand {
    pub session_id: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct OpenCommand {
    pub session_id: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ExecuteMovesCommand {
    pub session_id: u32,
    pub moves: [CubeMove; MAX_SOLUTION_MOVES],
    pub move_count: u8,
}

impl ExecuteMovesCommand {
    pub fn validate(&self) -> Result<(), SchemaError> {
        if usize::from(self.move_count) > MAX_SOLUTION_MOVES {
            return Err(SchemaError::TooManyRequestedMoves(self.move_count));
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CommandAccepted {
    /// Commands such as `GetStatus` complete in their response and have no ID.
    pub operation_id: Option<u32>,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum RejectionReason {
    InvalidControllerState = 0,
    StandPositionUnknown = 1,
    StandPoseMismatch = 2,
    SessionUnavailable = 3,
    SessionMismatch = 4,
    ScanUnavailable = 5,
    ScanRevisionMismatch = 6,
    SolutionUnavailable = 7,
    SolutionMismatch = 8,
    OperationAlreadyActive = 9,
    InvalidPayload = 10,
    UnsupportedCommand = 11,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CommandRejected {
    pub reason: RejectionReason,
    pub controller: ControllerState,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct StatusSnapshot {
    pub controller: ControllerState,
    pub stand: StandState,
    pub cube_session: Option<CubeSessionStatus>,
    pub scan: ScanStatus,
    pub solution: SolutionStatus,
    pub active_operation: Option<OperationStatus>,
    pub plan: [Option<MechanicalAction>; MAX_PLAN_PREVIEW_ACTIONS],
    pub plan_count: u8,
    pub fault: Option<RobotFault>,
}

impl StatusSnapshot {
    pub fn validate(&self) -> Result<(), SchemaError> {
        if usize::from(self.plan_count) > MAX_PLAN_PREVIEW_ACTIONS {
            return Err(SchemaError::TooManyPreviewActions(self.plan_count));
        }
        self.scan.validate()?;
        self.solution.validate()?;
        if let Some(operation) = self.active_operation {
            operation.validate()?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct RobotStateChanged {
    pub controller: ControllerState,
    pub active_operation: Option<OperationStatus>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct StandStateChanged {
    pub stand: StandState,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct CubeSessionChanged {
    pub session: Option<CubeSessionStatus>,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct FaceScanned {
    pub operation_id: u32,
    pub face: CubeFace,
    pub recognized: RecognizedFace,
    pub scanned_faces: u8,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct PlanChanged {
    pub operation_id: u32,
    pub actions: [Option<MechanicalAction>; MAX_PLAN_PREVIEW_ACTIONS],
    pub action_count: u8,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ActionProgress {
    pub operation_id: u32,
    pub action: MechanicalAction,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct OperationCompleted {
    pub operation_id: u32,
    pub kind: OperationKind,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct Aborted {
    pub operation_id: Option<u32>,
}

#[derive(Clone, Copy, Debug, Deserialize_repr, Eq, PartialEq, Serialize_repr)]
#[repr(u8)]
pub enum OperationFailureKind {
    Recognition = 0,
    InvalidFacelet = 1,
    SolverNoSolution = 2,
    Camera = 3,
    Inference = 4,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct OperationFailed {
    pub operation_id: u32,
    pub kind: OperationFailureKind,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct FaultEvent {
    pub operation_id: Option<u32>,
    pub fault: RobotFault,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        encode_payload, AxisMotion, CubeMove, GripperOrientation, GripperStatus, RailPosition,
        RailStatus, ScanStateKind, SchemaError, SolutionStateKind, StandPose, StandPoseKind,
        StickerColor, TurnAmount, MAX_PAYLOAD_LEN, MAX_SOLUTION_MOVES,
    };

    fn ready_snapshot() -> StatusSnapshot {
        let rail = RailStatus {
            motion: AxisMotion::Stable,
            current: Some(RailPosition::Open),
            target: None,
        };
        let gripper = GripperStatus {
            motion: AxisMotion::Stable,
            current: Some(GripperOrientation::FramePerpendicular),
            target: None,
        };
        let empty_move = CubeMove {
            face: CubeFace::Up,
            turn: TurnAmount::Clockwise,
        };

        StatusSnapshot {
            controller: ControllerState::Ready,
            stand: StandState {
                pose: StandPose {
                    kind: StandPoseKind::Open,
                    camera_face: None,
                },
                rails: [rail; 4],
                grippers: [gripper; 4],
                outputs_enabled: false,
            },
            scan: ScanStatus {
                state: ScanStateKind::None,
                revision: None,
                current_face: None,
                camera_face: None,
                scanned_faces: 0,
                faces: [None; 6],
                color_counts: [0; 6],
                validation_error: None,
            },
            solution: SolutionStatus {
                state: SolutionStateKind::None,
                id: None,
                source_scan_revision: None,
                moves: [empty_move; MAX_SOLUTION_MOVES],
                move_count: 0,
                completed_moves: 0,
            },
            cube_session: None,
            active_operation: None,
            plan: [None; MAX_PLAN_PREVIEW_ACTIONS],
            plan_count: 0,
            fault: None,
        }
    }

    #[test]
    fn snapshot_round_trips_and_fits_one_protocol_payload() {
        let snapshot = ready_snapshot();
        let mut output = [0u8; MAX_PAYLOAD_LEN];

        snapshot.validate().unwrap();
        let encoded = encode_payload(&snapshot, &mut output).unwrap();
        let decoded: StatusSnapshot = crate::decode_payload(encoded).unwrap();

        assert_eq!(decoded, snapshot);
        assert!(encoded.len() < MAX_PAYLOAD_LEN);
    }

    #[test]
    fn snapshot_rejects_out_of_bounds_counts() {
        let mut snapshot = ready_snapshot();
        snapshot.plan_count = (MAX_PLAN_PREVIEW_ACTIONS + 1) as u8;
        assert_eq!(
            snapshot.validate(),
            Err(SchemaError::TooManyPreviewActions(
                (MAX_PLAN_PREVIEW_ACTIONS + 1) as u8
            ))
        );

        snapshot.plan_count = 0;
        snapshot.solution.move_count = (MAX_SOLUTION_MOVES + 1) as u8;
        assert_eq!(
            snapshot.validate(),
            Err(SchemaError::TooManySolutionMoves(
                (MAX_SOLUTION_MOVES + 1) as u8
            ))
        );
    }

    #[test]
    fn unknown_color_keeps_its_explicit_wire_value() {
        let mut output = [0u8; 4];
        let encoded = encode_payload(&StickerColor::Unknown, &mut output).unwrap();
        assert_eq!(encoded, &[0xff]);
    }

    #[test]
    fn session_bound_solve_command_has_stable_encoding() {
        let command = SolveCommand {
            session_id: 42,
            scan_revision: 3,
        };
        let mut output = [0u8; 8];

        let encoded = encode_payload(&command, &mut output).unwrap();

        assert_eq!(encoded, &[42, 3]);
        assert_eq!(
            crate::decode_payload::<SolveCommand>(encoded).unwrap(),
            command
        );
    }

    #[test]
    fn manual_move_command_rejects_out_of_bounds_count() {
        let empty_move = CubeMove {
            face: CubeFace::Up,
            turn: TurnAmount::Clockwise,
        };
        let command = ExecuteMovesCommand {
            session_id: 7,
            moves: [empty_move; MAX_SOLUTION_MOVES],
            move_count: (MAX_SOLUTION_MOVES + 1) as u8,
        };

        assert_eq!(
            command.validate(),
            Err(SchemaError::TooManyRequestedMoves(
                (MAX_SOLUTION_MOVES + 1) as u8
            ))
        );
    }
}
