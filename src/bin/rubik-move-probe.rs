//! Execute one Singmaster move and restore the stand's canonical grip pose.

use anyhow::{bail, Result};
use clap::Parser;
use rubik_scan::{
    cube::{parse_solution, CubeMove},
    pca9685::Pca9685,
    stand::StandCalibration,
    stand_runtime::StandRuntime,
};
use std::{
    io::{self, Write},
    path::PathBuf,
};

#[derive(Parser)]
#[command(about = "Probe one calibrated Rubik move and restore canonical grip")]
struct Cli {
    /// One Singmaster move: U, R, F, D, L, B; add ' or 2 for inverse/half turn
    #[arg(
        value_name = "MOVE",
        required_unless_present = "sequence",
        conflicts_with = "sequence"
    )]
    move_token: Option<String>,

    /// A quoted Singmaster sequence, for example: "R U R' U'"
    #[arg(long, value_name = "MOVES")]
    sequence: Option<String>,

    /// Linux I²C device connected to PCA9685
    #[arg(long, default_value = "/dev/i2c-1")]
    i2c_device: PathBuf,

    /// PCA9685 7-bit I²C address, decimal or 0x-prefixed hexadecimal
    #[arg(long, default_value = "0x40", value_parser = parse_address)]
    address: u16,

    /// Optional TOML file overriding built-in stand calibration
    #[arg(long)]
    config: Option<PathBuf>,

    /// Servo PWM frequency used when this runtime starts
    #[arg(long, default_value_t = 50.0)]
    pwm_hz: f64,

    /// Required acknowledgement that this command will move the stand
    #[arg(long)]
    confirm_stand_motion: bool,
}

fn parse_address(value: &str) -> Result<u16, String> {
    let value = value.trim();
    let parsed = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
        .map_or_else(|| value.parse(), |hex| u16::from_str_radix(hex, 16))
        .map_err(|error| format!("invalid I²C address {value:?}: {error}"))?;
    if parsed > 0x7f {
        return Err(format!("I²C address must be 7-bit, got 0x{parsed:x}"));
    }
    Ok(parsed)
}

fn parse_single_move(token: &str) -> Result<CubeMove> {
    let moves = parse_solution(token)?;
    if moves.len() != 1 {
        bail!("expected exactly one Singmaster move, got {token:?}");
    }
    Ok(moves[0])
}

fn parse_moves(cli: &Cli) -> Result<Vec<CubeMove>> {
    match (&cli.move_token, &cli.sequence) {
        (Some(token), None) => Ok(vec![parse_single_move(token)?]),
        (None, Some(sequence)) => {
            let moves = parse_solution(sequence)?;
            if moves.is_empty() {
                bail!("--sequence must contain at least one Singmaster move");
            }
            Ok(moves)
        }
        _ => bail!("pass one MOVE or --sequence \"MOVES\""),
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if !cli.confirm_stand_motion {
        bail!("refusing to move the stand; pass --confirm-stand-motion after checking it");
    }
    let moves = parse_moves(&cli)?;
    let calibration = match &cli.config {
        Some(path) => StandCalibration::load(path)?,
        None => StandCalibration::default(),
    };

    let mut controller = Pca9685::open(&cli.i2c_device, cli.address)?;
    let status = controller.initialize_safe_pwm(cli.pwm_hz)?;
    let mut stand = StandRuntime::new(controller, calibration);
    stand.reset()?;
    stand.safe_open()?;
    print!(
        "stand=safe-open; place cube, then press Enter to execute {} move(s): ",
        moves.len()
    );
    io::stdout().flush()?;
    let mut confirmation = String::new();
    io::stdin().read_line(&mut confirmation)?;
    stand.grip()?;
    for (index, cube_move) in moves.iter().copied().enumerate() {
        println!(
            "move {}/{}: {}",
            index + 1,
            moves.len(),
            move_name(cube_move)
        );
        stand.execute_probe_move(cube_move)?;
    }

    println!("moves complete; stand=gripped front-facing (ready for physical inspection)");
    println!("pwm_hz={:.3}", status.pwm_hz());
    Ok(())
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
    fn parses_a_multi_move_sequence() {
        let moves = parse_solution("R U2 F'").unwrap();
        assert_eq!(moves.len(), 3);
        assert_eq!(move_name(moves[0]), "R");
        assert_eq!(move_name(moves[1]), "U2");
        assert_eq!(move_name(moves[2]), "F'");
    }
}
