//! Offline verifier for the scanner → facelet → min2phase contract.
//!
//! It intentionally takes camera-row-major colors, not URFDLB letters. Face
//! centres establish the color mapping exactly as they will on the real stand.

use anyhow::Result;
use clap::Parser;
use rubik_scan::cube::{parse_solution, CubeState, Face, LogicalFace, QuarterTurns, ScanPose};

#[derive(Parser)]
#[command(about = "Build URFDLB facelets from six scanned faces and solve them")]
struct Cli {
    /// Camera row-major colors for logical U, e.g. WWWWWWWWW
    #[arg(long)]
    up: String,
    /// Camera row-major colors for logical R
    #[arg(long)]
    right: String,
    /// Camera row-major colors for logical F
    #[arg(long)]
    front: String,
    /// Camera row-major colors for logical D
    #[arg(long)]
    down: String,
    /// Camera row-major colors for logical L
    #[arg(long)]
    left: String,
    /// Camera row-major colors for logical B
    #[arg(long)]
    back: String,

    /// Clockwise camera-to-canonical rotation for U: 0, 1, 2, or 3
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=3))]
    up_rotation: u8,
    /// Clockwise camera-to-canonical rotation for R: 0, 1, 2, or 3
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=3))]
    right_rotation: u8,
    /// Clockwise camera-to-canonical rotation for F: 0, 1, 2, or 3
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=3))]
    front_rotation: u8,
    /// Clockwise camera-to-canonical rotation for D: 0, 1, 2, or 3
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=3))]
    down_rotation: u8,
    /// Clockwise camera-to-canonical rotation for L: 0, 1, 2, or 3
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=3))]
    left_rotation: u8,
    /// Clockwise camera-to-canonical rotation for B: 0, 1, 2, or 3
    #[arg(long, default_value_t = 0, value_parser = clap::value_parser!(u8).range(0..=3))]
    back_rotation: u8,

    /// Maximum solution length requested from min2phase
    #[arg(long, default_value_t = 21)]
    max_moves: u8,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let mut state = CubeState::default();
    for (logical, colors, rotation) in [
        (LogicalFace::Up, &cli.up, cli.up_rotation),
        (LogicalFace::Right, &cli.right, cli.right_rotation),
        (LogicalFace::Front, &cli.front, cli.front_rotation),
        (LogicalFace::Down, &cli.down, cli.down_rotation),
        (LogicalFace::Left, &cli.left, cli.left_rotation),
        (LogicalFace::Back, &cli.back, cli.back_rotation),
    ] {
        state.record_scan(
            ScanPose {
                face: logical,
                camera_to_face: QuarterTurns::try_from(rotation)?,
            },
            Face::from_symbols(colors)?,
        )?;
    }

    let facelets = state.facelet_string()?;
    let solution = state.solve(cli.max_moves)?;
    let moves = parse_solution(&solution)?;
    println!("facelets: {facelets}");
    println!("solution: {solution}");
    println!("moves: {moves:?}");
    Ok(())
}
