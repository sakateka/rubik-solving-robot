//! Mechanical plans for logical Singmaster move sequences.
//!
//! The baseline planner restores canonical grip after every move. The
//! stateful planner keeps only opposite grippers in a non-canonical endpoint
//! and performs one shared regrip when the next move needs another axis pair.

use crate::{
    cube::{CubeMove, LogicalFace, MoveTurn},
    stand::{GripperOrientation, RailPosition, StandAxis, StandCalibration},
};
use std::{collections::VecDeque, time::Duration};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RailPair {
    LeftRight,
    TopBottom,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum RailTarget {
    Pair(RailPair),
    Single(StandAxis),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MovePlanStep {
    SetRails(RailTarget, RailPosition),
    SetGrippers(Vec<(StandAxis, GripperOrientation)>),
    MoveCompleted,
    AllOff,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum AxisGroup {
    LeftRight,
    TopBottom,
}

#[derive(Default)]
struct PlannerState {
    left_dirty: bool,
    right_dirty: bool,
    top_dirty: bool,
    bottom_dirty: bool,
}

impl PlannerState {
    fn is_dirty(&self, gripper: StandAxis) -> bool {
        match gripper {
            StandAxis::LeftGripper => self.left_dirty,
            StandAxis::RightGripper => self.right_dirty,
            StandAxis::TopGripper => self.top_dirty,
            StandAxis::BottomGripper => self.bottom_dirty,
            _ => false,
        }
    }

    fn set_dirty(&mut self, gripper: StandAxis, dirty: bool) {
        match gripper {
            StandAxis::LeftGripper => self.left_dirty = dirty,
            StandAxis::RightGripper => self.right_dirty = dirty,
            StandAxis::TopGripper => self.top_dirty = dirty,
            StandAxis::BottomGripper => self.bottom_dirty = dirty,
            _ => unreachable!("rail cannot be a dirty gripper"),
        }
    }

    fn dirty_grippers(&self, group: AxisGroup) -> Vec<StandAxis> {
        group_grippers(group)
            .into_iter()
            .filter(|axis| self.is_dirty(*axis))
            .collect()
    }
}

pub fn baseline_held_steps(moves: &[CubeMove]) -> VecDeque<MovePlanStep> {
    let mut steps = VecDeque::new();
    for &cube_move in moves {
        match cube_move.face {
            LogicalFace::Left => baseline_direct_turn(
                &mut steps,
                StandAxis::LeftGripper,
                StandAxis::LeftRail,
                cube_move.turn,
            ),
            LogicalFace::Right => baseline_direct_turn(
                &mut steps,
                StandAxis::RightGripper,
                StandAxis::RightRail,
                cube_move.turn,
            ),
            LogicalFace::Up => baseline_direct_turn(
                &mut steps,
                StandAxis::TopGripper,
                StandAxis::TopRail,
                cube_move.turn,
            ),
            LogicalFace::Down => baseline_direct_turn(
                &mut steps,
                StandAxis::BottomGripper,
                StandAxis::BottomRail,
                cube_move.turn,
            ),
            LogicalFace::Front => {
                position_front_back(&mut steps);
                baseline_direct_turn(
                    &mut steps,
                    StandAxis::RightGripper,
                    StandAxis::RightRail,
                    cube_move.turn,
                );
                restore_front(&mut steps);
            }
            LogicalFace::Back => {
                position_front_back(&mut steps);
                baseline_direct_turn(
                    &mut steps,
                    StandAxis::LeftGripper,
                    StandAxis::LeftRail,
                    cube_move.turn,
                );
                restore_front(&mut steps);
            }
        }
        steps.push_back(MovePlanStep::MoveCompleted);
    }
    steps
}

pub fn optimized_held_steps(moves: &[CubeMove]) -> VecDeque<MovePlanStep> {
    let mut steps = VecDeque::new();
    let mut state = PlannerState::default();
    let mut index = 0;

    while index < moves.len() {
        if matches!(moves[index].face, LogicalFace::Front | LogicalFace::Back) {
            flush_group(&mut steps, &mut state, AxisGroup::LeftRight);
            flush_group(&mut steps, &mut state, AxisGroup::TopBottom);
            position_front_back(&mut steps);
            while index < moves.len()
                && matches!(moves[index].face, LogicalFace::Front | LogicalFace::Back)
            {
                let cube_move = moves[index];
                let (gripper, rail) = match cube_move.face {
                    LogicalFace::Front => (StandAxis::RightGripper, StandAxis::RightRail),
                    LogicalFace::Back => (StandAxis::LeftGripper, StandAxis::LeftRail),
                    _ => unreachable!(),
                };
                lazy_direct_turn(&mut steps, &mut state, gripper, rail, cube_move.turn);
                steps.push_back(MovePlanStep::MoveCompleted);
                index += 1;
            }
            flush_group(&mut steps, &mut state, AxisGroup::LeftRight);
            restore_front(&mut steps);
            continue;
        }

        let cube_move = moves[index];
        let (group, gripper, rail) = direct_axis(cube_move.face);
        let other = match group {
            AxisGroup::LeftRight => AxisGroup::TopBottom,
            AxisGroup::TopBottom => AxisGroup::LeftRight,
        };
        flush_group(&mut steps, &mut state, other);
        lazy_direct_turn(&mut steps, &mut state, gripper, rail, cube_move.turn);
        steps.push_back(MovePlanStep::MoveCompleted);
        index += 1;
    }

    flush_group(&mut steps, &mut state, AxisGroup::LeftRight);
    flush_group(&mut steps, &mut state, AxisGroup::TopBottom);
    steps
}

pub fn append_open_steps(steps: &mut VecDeque<MovePlanStep>) {
    steps.extend([
        MovePlanStep::SetRails(RailTarget::Pair(RailPair::LeftRight), RailPosition::FarOpen),
        MovePlanStep::SetRails(RailTarget::Pair(RailPair::TopBottom), RailPosition::FarOpen),
        MovePlanStep::AllOff,
    ]);
}

pub fn estimated_duration(
    steps: &VecDeque<MovePlanStep>,
    calibration: &StandCalibration,
) -> Duration {
    steps.iter().fold(Duration::ZERO, |total, step| {
        total
            + match step {
                MovePlanStep::SetRails(_, position) => calibration.rail_duration(*position),
                MovePlanStep::SetGrippers(_) => calibration.gripper_pose_duration(),
                MovePlanStep::MoveCompleted | MovePlanStep::AllOff => Duration::ZERO,
            }
    })
}

pub fn servo_target_count(steps: &VecDeque<MovePlanStep>) -> usize {
    steps
        .iter()
        .map(|step| match step {
            MovePlanStep::SetRails(RailTarget::Pair(_), _) => 2,
            MovePlanStep::SetRails(RailTarget::Single(_), _) => 1,
            MovePlanStep::SetGrippers(poses) => poses.len(),
            MovePlanStep::MoveCompleted | MovePlanStep::AllOff => 0,
        })
        .sum()
}

fn direct_axis(face: LogicalFace) -> (AxisGroup, StandAxis, StandAxis) {
    match face {
        LogicalFace::Left => (
            AxisGroup::LeftRight,
            StandAxis::LeftGripper,
            StandAxis::LeftRail,
        ),
        LogicalFace::Right => (
            AxisGroup::LeftRight,
            StandAxis::RightGripper,
            StandAxis::RightRail,
        ),
        LogicalFace::Up => (
            AxisGroup::TopBottom,
            StandAxis::TopGripper,
            StandAxis::TopRail,
        ),
        LogicalFace::Down => (
            AxisGroup::TopBottom,
            StandAxis::BottomGripper,
            StandAxis::BottomRail,
        ),
        LogicalFace::Front | LogicalFace::Back => unreachable!("front/back require reorientation"),
    }
}

fn lazy_direct_turn(
    steps: &mut VecDeque<MovePlanStep>,
    state: &mut PlannerState,
    gripper: StandAxis,
    rail: StandAxis,
    turn: MoveTurn,
) {
    if state.is_dirty(gripper) {
        flush_grippers(steps, state, vec![gripper]);
    }
    match turn {
        MoveTurn::Clockwise => steps.push_back(MovePlanStep::SetGrippers(vec![(
            gripper,
            GripperOrientation::FrameParallelReversed,
        )])),
        MoveTurn::CounterClockwise => steps.push_back(MovePlanStep::SetGrippers(vec![(
            gripper,
            GripperOrientation::FrameParallel,
        )])),
        MoveTurn::Half => {
            steps.push_back(MovePlanStep::SetRails(
                RailTarget::Single(rail),
                RailPosition::FarOpen,
            ));
            steps.push_back(MovePlanStep::SetGrippers(vec![(
                gripper,
                GripperOrientation::FrameParallel,
            )]));
            steps.push_back(MovePlanStep::SetRails(
                RailTarget::Single(rail),
                RailPosition::NearGrip,
            ));
            steps.push_back(MovePlanStep::SetGrippers(vec![(
                gripper,
                GripperOrientation::FrameParallelReversed,
            )]));
        }
    }
    state.set_dirty(gripper, true);
}

fn flush_group(steps: &mut VecDeque<MovePlanStep>, state: &mut PlannerState, group: AxisGroup) {
    let dirty = state.dirty_grippers(group);
    if !dirty.is_empty() {
        flush_grippers(steps, state, dirty);
    }
}

fn flush_grippers(
    steps: &mut VecDeque<MovePlanStep>,
    state: &mut PlannerState,
    grippers: Vec<StandAxis>,
) {
    let rail_target = if grippers.len() == 2 {
        RailTarget::Pair(match group_for_gripper(grippers[0]) {
            AxisGroup::LeftRight => RailPair::LeftRight,
            AxisGroup::TopBottom => RailPair::TopBottom,
        })
    } else {
        RailTarget::Single(rail_for_gripper(grippers[0]))
    };
    steps.push_back(MovePlanStep::SetRails(
        rail_target.clone(),
        RailPosition::FarOpen,
    ));
    steps.push_back(MovePlanStep::SetGrippers(
        grippers
            .iter()
            .copied()
            .map(|axis| (axis, GripperOrientation::FramePerpendicular))
            .collect(),
    ));
    steps.push_back(MovePlanStep::SetRails(rail_target, RailPosition::NearGrip));
    for gripper in grippers {
        state.set_dirty(gripper, false);
    }
}

fn baseline_direct_turn(
    steps: &mut VecDeque<MovePlanStep>,
    gripper: StandAxis,
    rail: StandAxis,
    turn: MoveTurn,
) {
    let mut state = PlannerState::default();
    lazy_direct_turn(steps, &mut state, gripper, rail, turn);
    flush_grippers(steps, &mut state, vec![gripper]);
}

fn group_for_gripper(gripper: StandAxis) -> AxisGroup {
    match gripper {
        StandAxis::LeftGripper | StandAxis::RightGripper => AxisGroup::LeftRight,
        StandAxis::TopGripper | StandAxis::BottomGripper => AxisGroup::TopBottom,
        _ => unreachable!("rail is not a gripper"),
    }
}

fn group_grippers(group: AxisGroup) -> [StandAxis; 2] {
    match group {
        AxisGroup::LeftRight => [StandAxis::LeftGripper, StandAxis::RightGripper],
        AxisGroup::TopBottom => [StandAxis::TopGripper, StandAxis::BottomGripper],
    }
}

fn rail_for_gripper(gripper: StandAxis) -> StandAxis {
    match gripper {
        StandAxis::LeftGripper => StandAxis::LeftRail,
        StandAxis::RightGripper => StandAxis::RightRail,
        StandAxis::TopGripper => StandAxis::TopRail,
        StandAxis::BottomGripper => StandAxis::BottomRail,
        _ => unreachable!("rail is not a gripper"),
    }
}

fn position_front_back(steps: &mut VecDeque<MovePlanStep>) {
    use GripperOrientation::{FrameParallel as P, FrameParallelReversed as R};
    steps.extend([
        MovePlanStep::SetRails(RailTarget::Pair(RailPair::LeftRight), RailPosition::FarOpen),
        MovePlanStep::SetGrippers(vec![
            (StandAxis::TopGripper, P),
            (StandAxis::BottomGripper, R),
        ]),
        MovePlanStep::SetRails(
            RailTarget::Pair(RailPair::LeftRight),
            RailPosition::NearGrip,
        ),
        MovePlanStep::SetRails(RailTarget::Pair(RailPair::TopBottom), RailPosition::FarOpen),
        MovePlanStep::SetGrippers(vec![
            (
                StandAxis::TopGripper,
                GripperOrientation::FramePerpendicular,
            ),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FramePerpendicular,
            ),
        ]),
        MovePlanStep::SetRails(
            RailTarget::Pair(RailPair::TopBottom),
            RailPosition::NearGrip,
        ),
    ]);
}

fn restore_front(steps: &mut VecDeque<MovePlanStep>) {
    use GripperOrientation::{FrameParallel as P, FrameParallelReversed as R};
    steps.extend([
        MovePlanStep::SetRails(RailTarget::Pair(RailPair::LeftRight), RailPosition::FarOpen),
        MovePlanStep::SetGrippers(vec![
            (StandAxis::TopGripper, R),
            (StandAxis::BottomGripper, P),
        ]),
        MovePlanStep::SetRails(
            RailTarget::Pair(RailPair::LeftRight),
            RailPosition::NearGrip,
        ),
        MovePlanStep::SetRails(RailTarget::Pair(RailPair::TopBottom), RailPosition::FarOpen),
        MovePlanStep::SetGrippers(vec![
            (
                StandAxis::TopGripper,
                GripperOrientation::FramePerpendicular,
            ),
            (
                StandAxis::BottomGripper,
                GripperOrientation::FramePerpendicular,
            ),
        ]),
        MovePlanStep::SetRails(
            RailTarget::Pair(RailPair::TopBottom),
            RailPosition::NearGrip,
        ),
    ]);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn moves(sequence: &str) -> Vec<CubeMove> {
        crate::cube::parse_solution(sequence).unwrap()
    }

    #[test]
    fn batches_opposite_faces_into_one_shared_regrip() {
        let baseline = baseline_held_steps(&moves("R L"));
        let optimized = optimized_held_steps(&moves("R L"));

        assert_eq!(baseline.len(), 10);
        assert_eq!(optimized.len(), 7);
        assert!(
            estimated_duration(&optimized, &StandCalibration::default())
                < estimated_duration(&baseline, &StandCalibration::default())
        );
    }

    #[test]
    fn keeps_one_whole_cube_reorientation_for_front_back_block() {
        let baseline = baseline_held_steps(&moves("F B"));
        let optimized = optimized_held_steps(&moves("F B"));

        assert_eq!(baseline.len(), 34);
        assert_eq!(optimized.len(), 19);
        assert_eq!(
            optimized
                .iter()
                .filter(|step| matches!(step, MovePlanStep::MoveCompleted))
                .count(),
            2
        );
    }

    #[test]
    fn optimized_plans_preserve_safety_for_all_three_move_sequences() {
        let all_moves = LogicalFace::ALL
            .into_iter()
            .flat_map(|face| {
                [
                    MoveTurn::Clockwise,
                    MoveTurn::CounterClockwise,
                    MoveTurn::Half,
                ]
                .map(move |turn| CubeMove { face, turn })
            })
            .collect::<Vec<_>>();
        let calibration = StandCalibration::default();

        for &first in &all_moves {
            for &second in &all_moves {
                for &third in &all_moves {
                    let moves = [first, second, third];
                    let baseline = baseline_held_steps(&moves);
                    let optimized = optimized_held_steps(&moves);
                    assert_safe_held_plan(&optimized, moves.len());
                    assert!(
                        estimated_duration(&optimized, &calibration)
                            <= estimated_duration(&baseline, &calibration),
                        "optimized plan is slower for {moves:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn optimized_plans_preserve_safety_for_all_four_move_sequences() {
        let all_moves = all_moves();

        for &first in &all_moves {
            for &second in &all_moves {
                for &third in &all_moves {
                    for &fourth in &all_moves {
                        let moves = [first, second, third, fourth];
                        let optimized = optimized_held_steps(&moves);
                        assert_safe_held_plan(&optimized, moves.len());
                    }
                }
            }
        }
    }

    #[test]
    fn mechanical_plan_preserves_logical_moves_and_turn_directions() {
        let all_moves = all_moves();

        for cube_move in all_moves {
            let moves = [cube_move];
            assert_decodes_to_moves(&optimized_held_steps(&moves), &moves);
        }

        for sequence in ["F U B", "F U R", "F R B", "F U D B"] {
            let moves = moves(sequence);
            assert_decodes_to_moves(&optimized_held_steps(&moves), &moves);
        }
    }

    fn all_moves() -> Vec<CubeMove> {
        LogicalFace::ALL
            .into_iter()
            .flat_map(|face| {
                [
                    MoveTurn::Clockwise,
                    MoveTurn::CounterClockwise,
                    MoveTurn::Half,
                ]
                .map(move |turn| CubeMove { face, turn })
            })
            .collect()
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq)]
    enum TestFrame {
        Canonical,
        RotatedFb,
    }

    fn assert_decodes_to_moves(steps: &VecDeque<MovePlanStep>, expected: &[CubeMove]) {
        use GripperOrientation::{FrameParallel, FrameParallelReversed};

        let mut rails = [RailPosition::NearGrip; 4];
        let mut frame = TestFrame::Canonical;
        let mut parallel_events = Vec::new();
        let mut decoded = Vec::new();

        for step in steps {
            match step {
                MovePlanStep::SetRails(target, position) => match target {
                    RailTarget::Pair(RailPair::LeftRight) => {
                        rails[0] = *position;
                        rails[1] = *position;
                    }
                    RailTarget::Pair(RailPair::TopBottom) => {
                        rails[2] = *position;
                        rails[3] = *position;
                    }
                    RailTarget::Single(axis) => rails[axis_index(*axis)] = *position,
                },
                MovePlanStep::SetGrippers(poses) => {
                    let is_whole_cube_reorientation = poses.len() == 2
                        && poses.iter().all(|(axis, orientation)| {
                            matches!(
                                axis,
                                StandAxis::TopGripper | StandAxis::BottomGripper
                            ) && orientation.is_frame_parallel()
                        });

                    if is_whole_cube_reorientation {
                        assert_eq!(
                            rails[0],
                            RailPosition::FarOpen,
                            "cube rotation must happen while LR rails are open"
                        );
                        let top = poses
                            .iter()
                            .find(|(axis, _)| *axis == StandAxis::TopGripper)
                            .map(|(_, orientation)| *orientation)
                            .unwrap();
                        let bottom = poses
                            .iter()
                            .find(|(axis, _)| *axis == StandAxis::BottomGripper)
                            .map(|(_, orientation)| *orientation)
                            .unwrap();

                        frame = match (top, bottom) {
                            (FrameParallel, FrameParallelReversed) => TestFrame::RotatedFb,
                            (FrameParallelReversed, FrameParallel) => TestFrame::Canonical,
                            orientations => panic!(
                                "unexpected whole-cube orientation pair: {orientations:?}"
                            ),
                        };
                    } else {
                        parallel_events.extend(
                            poses
                                .iter()
                                .filter(|(_, orientation)| orientation.is_frame_parallel())
                                .copied(),
                        );
                    }
                }
                MovePlanStep::MoveCompleted => {
                    let (axis, turn) = match parallel_events.as_slice() {
                        [(axis, FrameParallel)] => (*axis, MoveTurn::CounterClockwise),
                        [(axis, FrameParallelReversed)] => (*axis, MoveTurn::Clockwise),
                        [
                            (axis @ _, FrameParallel),
                            (same_axis, FrameParallelReversed),
                        ] if axis == same_axis => (*axis, MoveTurn::Half),
                        events => panic!(
                            "expected one quarter-turn or two half-turn events before marker, got {events:?}"
                        ),
                    };
                    decoded.push(CubeMove {
                        face: logical_face(frame, axis),
                        turn,
                    });
                    parallel_events.clear();
                }
                MovePlanStep::AllOff => panic!("held plan must not disable outputs"),
            }
        }

        assert!(
            parallel_events.is_empty(),
            "plan ended with an uncompleted mechanical turn: {parallel_events:?}"
        );
        assert_eq!(decoded, expected);
        assert_eq!(frame, TestFrame::Canonical);
    }

    fn logical_face(frame: TestFrame, gripper: StandAxis) -> LogicalFace {
        match (frame, gripper) {
            (_, StandAxis::TopGripper) => LogicalFace::Up,
            (_, StandAxis::BottomGripper) => LogicalFace::Down,
            (TestFrame::Canonical, StandAxis::LeftGripper) => LogicalFace::Left,
            (TestFrame::Canonical, StandAxis::RightGripper) => LogicalFace::Right,
            (TestFrame::RotatedFb, StandAxis::LeftGripper) => LogicalFace::Back,
            (TestFrame::RotatedFb, StandAxis::RightGripper) => LogicalFace::Front,
            (_, axis) => panic!("rail is not a gripper: {axis:?}"),
        }
    }

    fn assert_safe_held_plan(steps: &VecDeque<MovePlanStep>, expected_moves: usize) {
        let mut rails = [RailPosition::NearGrip; 4];
        let mut grippers = [GripperOrientation::FramePerpendicular; 4];
        let mut completed_moves = 0;

        for step in steps {
            match step {
                MovePlanStep::SetRails(target, position) => match target {
                    RailTarget::Pair(RailPair::LeftRight) => {
                        rails[0] = *position;
                        rails[1] = *position;
                    }
                    RailTarget::Pair(RailPair::TopBottom) => {
                        rails[2] = *position;
                        rails[3] = *position;
                    }
                    RailTarget::Single(axis) => rails[axis_index(*axis)] = *position,
                },
                MovePlanStep::SetGrippers(poses) => {
                    for &(axis, orientation) in poses {
                        grippers[axis_index(axis)] = orientation;
                    }
                }
                MovePlanStep::MoveCompleted => completed_moves += 1,
                MovePlanStep::AllOff => panic!("held plan must not disable outputs"),
            }

            assert!(
                (rails[0] == RailPosition::NearGrip && rails[1] == RailPosition::NearGrip)
                    || (rails[2] == RailPosition::NearGrip && rails[3] == RailPosition::NearGrip),
                "cube custody was lost after {step:?}"
            );
            for (first, second) in [(0, 2), (2, 1), (1, 3), (3, 0)] {
                let both_gripped = rails[first] == RailPosition::NearGrip
                    && rails[second] == RailPosition::NearGrip;
                let both_parallel =
                    grippers[first].is_frame_parallel() && grippers[second].is_frame_parallel();
                assert!(
                    !(both_gripped && both_parallel),
                    "adjacent grippers {first}/{second} collide after {step:?}"
                );
            }
        }

        assert_eq!(completed_moves, expected_moves);
        assert_eq!(rails, [RailPosition::NearGrip; 4]);
        assert_eq!(grippers, [GripperOrientation::FramePerpendicular; 4]);
    }

    fn axis_index(axis: StandAxis) -> usize {
        match axis {
            StandAxis::LeftRail | StandAxis::LeftGripper => 0,
            StandAxis::RightRail | StandAxis::RightGripper => 1,
            StandAxis::TopRail | StandAxis::TopGripper => 2,
            StandAxis::BottomRail | StandAxis::BottomGripper => 3,
        }
    }
}
