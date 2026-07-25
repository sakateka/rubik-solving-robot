//! Domain model between vision, the scan protocol, and the cube solver.
//!
//! Vision returns a `Face`: exactly nine physical sticker colours in camera
//! row-major order. A calibrated scan pose says which logical face was shown
//! to the camera and how camera orientation maps to cube orientation. Once all
//! six faces are present, their centres define the colour → URFDLB mapping.

use anyhow::{bail, Context, Result};
use std::fmt;

const FACELET_LEN: usize = 54;
#[cfg(test)]
const SOLVED_FACELET: &str = "UUUUUUUUURRRRRRRRRFFFFFFFFFDDDDDDDDDLLLLLLLLLBBBBBBBBB";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum StickerColor {
    White,
    Yellow,
    Red,
    Orange,
    Green,
    Blue,
}

impl StickerColor {
    pub const ALL: [Self; 6] = [
        Self::White,
        Self::Yellow,
        Self::Red,
        Self::Orange,
        Self::Green,
        Self::Blue,
    ];

    pub const fn symbol(self) -> char {
        match self {
            Self::White => 'W',
            Self::Yellow => 'Y',
            Self::Red => 'R',
            Self::Orange => 'O',
            Self::Green => 'G',
            Self::Blue => 'B',
        }
    }
}

impl TryFrom<char> for StickerColor {
    type Error = anyhow::Error;

    fn try_from(value: char) -> Result<Self> {
        match value {
            'W' => Ok(Self::White),
            'Y' => Ok(Self::Yellow),
            'R' => Ok(Self::Red),
            'O' => Ok(Self::Orange),
            'G' => Ok(Self::Green),
            'B' => Ok(Self::Blue),
            _ => bail!("unknown sticker color {value:?}"),
        }
    }
}

impl fmt::Display for StickerColor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.symbol())
    }
}

/// Logical faces in the exact min2phase facelet order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(usize)]
pub enum LogicalFace {
    Up = 0,
    Right = 1,
    Front = 2,
    Down = 3,
    Left = 4,
    Back = 5,
}

impl LogicalFace {
    pub const ALL: [Self; 6] = [
        Self::Up,
        Self::Right,
        Self::Front,
        Self::Down,
        Self::Left,
        Self::Back,
    ];

    pub const fn symbol(self) -> char {
        match self {
            Self::Up => 'U',
            Self::Right => 'R',
            Self::Front => 'F',
            Self::Down => 'D',
            Self::Left => 'L',
            Self::Back => 'B',
        }
    }
}

/// One camera scan, arranged from top-left to bottom-right.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Face {
    stickers: [StickerColor; 9],
}

impl Face {
    pub fn from_symbols(symbols: &str) -> Result<Self> {
        let symbols: Vec<_> = symbols.chars().collect();
        if symbols.len() != 9 {
            bail!("face has {} stickers, expected 9", symbols.len());
        }
        let mut stickers = [StickerColor::White; 9];
        for (destination, symbol) in stickers.iter_mut().zip(symbols) {
            *destination = StickerColor::try_from(symbol)?;
        }
        Ok(Self { stickers })
    }

    pub const fn center(self) -> StickerColor {
        self.stickers[4]
    }

    pub const fn stickers(self) -> [StickerColor; 9] {
        self.stickers
    }

    pub fn compact(self) -> String {
        self.stickers.iter().map(|color| color.symbol()).collect()
    }

    /// Rotates camera row-major clockwise into canonical face orientation.
    pub fn rotated(self, quarter_turns: QuarterTurns) -> Self {
        let mut result = self;
        for _ in 0..quarter_turns.as_u8() {
            result.stickers = [
                result.stickers[6],
                result.stickers[3],
                result.stickers[0],
                result.stickers[7],
                result.stickers[4],
                result.stickers[1],
                result.stickers[8],
                result.stickers[5],
                result.stickers[2],
            ];
        }
        result
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QuarterTurns {
    Zero,
    One,
    Two,
    Three,
}

impl QuarterTurns {
    const fn as_u8(self) -> u8 {
        match self {
            Self::Zero => 0,
            Self::One => 1,
            Self::Two => 2,
            Self::Three => 3,
        }
    }
}

impl TryFrom<u8> for QuarterTurns {
    type Error = anyhow::Error;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(Self::Zero),
            1 => Ok(Self::One),
            2 => Ok(Self::Two),
            3 => Ok(Self::Three),
            _ => bail!("quarter turns must be 0, 1, 2, or 3; got {value}"),
        }
    }
}

/// Mechanical pose calibration: logical face plus camera orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScanPose {
    pub face: LogicalFace,
    pub camera_to_face: QuarterTurns,
}

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct CubeState {
    faces: [Option<Face>; 6],
}

impl CubeState {
    pub fn record_scan(&mut self, pose: ScanPose, camera_face: Face) -> Result<()> {
        let slot = &mut self.faces[pose.face as usize];
        if slot.is_some() {
            bail!(
                "logical face {} has already been scanned",
                pose.face.symbol()
            );
        }
        *slot = Some(camera_face.rotated(pose.camera_to_face));
        Ok(())
    }

    /// Builds validated URFDLB facelets. Logical letters are inferred from
    /// centre stickers, not from an assumed physical colour scheme.
    pub fn facelet_string(&self) -> Result<String> {
        let faces: Vec<Face> = self
            .faces
            .iter()
            .enumerate()
            .map(|(index, face)| {
                face.with_context(|| {
                    format!(
                        "missing scan for logical face {}",
                        LogicalFace::ALL[index].symbol()
                    )
                })
            })
            .collect::<Result<_>>()?;

        let mut center_to_face = [None; 6];
        for (index, face) in faces.iter().enumerate() {
            let color_index = color_index(face.center());
            if center_to_face[color_index]
                .replace(LogicalFace::ALL[index])
                .is_some()
            {
                bail!(
                    "two scanned faces have {} as their center color",
                    face.center()
                );
            }
        }

        let mut counts = [0_u8; 6];
        let mut facelets = String::with_capacity(FACELET_LEN);
        for face in faces {
            for color in face.stickers() {
                let color_index = color_index(color);
                counts[color_index] += 1;
                let logical = center_to_face[color_index]
                    .expect("all sticker colours occur as centres in a complete state");
                facelets.push(logical.symbol());
            }
        }
        for color in StickerColor::ALL {
            if counts[color_index(color)] != 9 {
                bail!(
                    "color {color} appears {} times, expected 9",
                    counts[color_index(color)]
                );
            }
        }
        Ok(facelets)
    }

    pub fn solve(&self, max_moves: u8) -> Result<String> {
        solve_facelets(&self.facelet_string()?, max_moves)
    }
}

/// Thin adapter around min2phase, whose public API reports errors as strings.
pub fn solve_facelets(facelets: &str, max_moves: u8) -> Result<String> {
    if facelets.len() != FACELET_LEN {
        bail!(
            "facelet string has {} symbols, expected {FACELET_LEN}",
            facelets.len()
        );
    }
    let solution = min2phase::solve(&facelets.to_owned(), max_moves);
    if let Some(error) = solution.strip_prefix("Error ") {
        bail!("min2phase rejected cube state: error {error}");
    }
    Ok(solution.trim().to_owned())
}

/// One move in standard Singmaster notation, ready for the mechanical planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CubeMove {
    pub face: LogicalFace,
    pub turn: MoveTurn,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoveTurn {
    Clockwise,
    CounterClockwise,
    Half,
}

pub fn parse_solution(solution: &str) -> Result<Vec<CubeMove>> {
    solution
        .split_whitespace()
        .map(parse_move)
        .collect::<Result<Vec<_>>>()
}

fn parse_move(token: &str) -> Result<CubeMove> {
    let mut chars = token.chars();
    let face = match chars.next() {
        Some('U') => LogicalFace::Up,
        Some('R') => LogicalFace::Right,
        Some('F') => LogicalFace::Front,
        Some('D') => LogicalFace::Down,
        Some('L') => LogicalFace::Left,
        Some('B') => LogicalFace::Back,
        _ => bail!("invalid solver move {token:?}"),
    };
    let turn = match chars.next() {
        None => MoveTurn::Clockwise,
        Some('\'') if chars.next().is_none() => MoveTurn::CounterClockwise,
        Some('2') if chars.next().is_none() => MoveTurn::Half,
        _ => bail!("invalid solver move {token:?}"),
    };
    Ok(CubeMove { face, turn })
}

const fn color_index(color: StickerColor) -> usize {
    match color {
        StickerColor::White => 0,
        StickerColor::Yellow => 1,
        StickerColor::Red => 2,
        StickerColor::Orange => 3,
        StickerColor::Green => 4,
        StickerColor::Blue => 5,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn face(color: char) -> Face {
        Face::from_symbols(&color.to_string().repeat(9)).unwrap()
    }

    fn solved_state() -> CubeState {
        let mut state = CubeState::default();
        for (logical, color) in [
            (LogicalFace::Up, 'W'),
            (LogicalFace::Right, 'R'),
            (LogicalFace::Front, 'G'),
            (LogicalFace::Down, 'Y'),
            (LogicalFace::Left, 'O'),
            (LogicalFace::Back, 'B'),
        ] {
            state
                .record_scan(
                    ScanPose {
                        face: logical,
                        camera_to_face: QuarterTurns::Zero,
                    },
                    face(color),
                )
                .unwrap();
        }
        state
    }

    #[test]
    fn rotates_camera_face_clockwise() {
        let face = Face::from_symbols("WRGYOBWRG").unwrap();
        assert_eq!(face.rotated(QuarterTurns::One).compact(), "WYWRORGBG");
        assert_eq!(
            face.rotated(QuarterTurns::Two).rotated(QuarterTurns::Two),
            face
        );
        assert_eq!(
            face.rotated(QuarterTurns::Three).rotated(QuarterTurns::One),
            face
        );
    }

    #[test]
    fn centers_define_solved_facelets() {
        let state = solved_state();
        assert_eq!(state.facelet_string().unwrap(), SOLVED_FACELET);
        assert_eq!(state.solve(21).unwrap(), "");
    }

    #[test]
    fn rejects_wrong_color_count() {
        let mut state = solved_state();
        state.faces[LogicalFace::Up as usize] = Some(Face::from_symbols("RWWWWWWWW").unwrap());
        assert!(state.facelet_string().is_err());
    }

    #[test]
    fn solver_solution_returns_to_solved_cube() {
        let scrambled = min2phase::from_moves(&"R U F2".to_owned()).unwrap();
        let solution = solve_facelets(&scrambled, 21).unwrap();
        assert_eq!(
            min2phase::apply_moves(&scrambled, &solution).unwrap(),
            SOLVED_FACELET
        );
    }

    #[test]
    fn parses_solver_move_notation() {
        assert_eq!(
            parse_solution("R U' F2").unwrap(),
            vec![
                CubeMove {
                    face: LogicalFace::Right,
                    turn: MoveTurn::Clockwise
                },
                CubeMove {
                    face: LogicalFace::Up,
                    turn: MoveTurn::CounterClockwise
                },
                CubeMove {
                    face: LogicalFace::Front,
                    turn: MoveTurn::Half
                },
            ]
        );
    }
}
