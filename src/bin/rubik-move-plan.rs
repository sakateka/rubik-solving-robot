//! Compare conservative and stateful mechanical plans without touching hardware.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    cube::{parse_solution, CubeMove},
    move_planner::{
        append_open_steps, baseline_held_steps, estimated_duration, optimized_execute_steps,
        optimized_held_steps, servo_target_count, MovePlanStep, RailPair, RailTarget,
    },
    stand::StandCalibration,
};
use std::{collections::VecDeque, path::PathBuf};

#[derive(Parser)]
#[command(about = "Compare baseline and stateful Rubik move plans without servo output")]
struct Cli {
    /// Quoted Singmaster sequence, for example `F B R2 U'`.
    sequence: String,

    /// Optional stand calibration used for duration estimates.
    #[arg(long)]
    config: Option<PathBuf>,

    /// Include the normal final stand opening after the sequence.
    #[arg(long)]
    open_after: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let moves = parse_solution(&cli.sequence)?;
    if moves.is_empty() {
        bail!("sequence must contain at least one Singmaster move");
    }
    let calibration = match cli.config {
        Some(path) => StandCalibration::load(&path)?,
        None => StandCalibration::default(),
    };
    let mut baseline = baseline_held_steps(&moves);
    let mut optimized = if cli.open_after {
        optimized_execute_steps(&moves)
    } else {
        optimized_held_steps(&moves)
    };
    if cli.open_after {
        append_open_steps(&mut baseline);
        append_open_steps(&mut optimized);
    }

    println!("sequence  {}", format_moves(&moves));
    println!(
        "finish    {}",
        if cli.open_after {
            "normal open"
        } else {
            "canonical grip"
        }
    );
    println!();
    print_summary("baseline", &baseline, &calibration);
    print_summary("optimized", &optimized, &calibration);
    let baseline_duration = estimated_duration(&baseline, &calibration);
    let optimized_duration = estimated_duration(&optimized, &calibration);
    println!(
        "saved     {} actions, {} servo targets, {}",
        action_count(&baseline).saturating_sub(action_count(&optimized)),
        servo_target_count(&baseline).saturating_sub(servo_target_count(&optimized)),
        humantime::format_duration(baseline_duration.saturating_sub(optimized_duration))
    );

    print_trace("Baseline trace", &baseline, &moves, &calibration);
    print_trace("Optimized trace", &optimized, &moves, &calibration);
    Ok(())
}

fn print_summary(name: &str, steps: &VecDeque<MovePlanStep>, calibration: &StandCalibration) {
    println!(
        "{name:<9} actions={:<3} servo_targets={:<3} estimated={}",
        action_count(steps),
        servo_target_count(steps),
        humantime::format_duration(estimated_duration(steps, calibration))
    );
}

fn print_trace(
    title: &str,
    steps: &VecDeque<MovePlanStep>,
    moves: &[CubeMove],
    calibration: &StandCalibration,
) {
    println!();
    println!("{title}");
    let mut move_index = 0;
    let mut action_index = 0;
    for step in steps {
        match step {
            MovePlanStep::SetRails(target, position) => {
                action_index += 1;
                println!(
                    "  {action_index:>2}. rails {:<12} -> {:<9} ({})",
                    rail_target(target),
                    rail_position(*position),
                    humantime::format_duration(calibration.rail_duration(*position))
                );
            }
            MovePlanStep::SetGrippers(poses) => {
                action_index += 1;
                println!(
                    "  {action_index:>2}. grippers {:<31} ({})",
                    poses
                        .iter()
                        .map(|(axis, pose)| format!("{}={}", axis.name(), pose.name()))
                        .collect::<Vec<_>>()
                        .join(", "),
                    humantime::format_duration(calibration.gripper_pose_duration())
                );
            }
            MovePlanStep::MoveCompleted => {
                println!(
                    "      move {}/{} complete: {}",
                    move_index + 1,
                    moves.len(),
                    move_name(moves[move_index])
                );
                move_index += 1;
            }
            MovePlanStep::AllOff => {
                action_index += 1;
                println!("  {action_index:>2}. PWM all off");
            }
        }
    }
}

fn action_count(steps: &VecDeque<MovePlanStep>) -> usize {
    steps
        .iter()
        .filter(|step| !matches!(step, MovePlanStep::MoveCompleted))
        .count()
}

fn rail_target(target: &RailTarget) -> &'static str {
    match target {
        RailTarget::Pair(RailPair::LeftRight) => "left+right",
        RailTarget::Pair(RailPair::TopBottom) => "top+bottom",
        RailTarget::Single(axis) => axis.name(),
    }
}

fn rail_position(position: rubik_scan::stand::RailPosition) -> &'static str {
    match position {
        rubik_scan::stand::RailPosition::FarOpen => "far-open",
        rubik_scan::stand::RailPosition::NearGrip => "near-grip",
    }
}

fn format_moves(moves: &[CubeMove]) -> String {
    moves
        .iter()
        .copied()
        .map(move_name)
        .collect::<Vec<_>>()
        .join(" ")
}

fn move_name(cube_move: CubeMove) -> String {
    let suffix = match cube_move.turn {
        rubik_scan::cube::MoveTurn::Clockwise => "",
        rubik_scan::cube::MoveTurn::CounterClockwise => "'",
        rubik_scan::cube::MoveTurn::Half => "2",
    };
    format!("{}{}", cube_move.face.symbol(), suffix)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn formats_moves_in_singmaster_notation() {
        let moves = parse_solution("F B' R2").unwrap();
        assert_eq!(format_moves(&moves), "F B' R2");
    }
}
