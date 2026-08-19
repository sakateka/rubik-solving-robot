use crate::{CubeFace, CubeMove, TurnAmount, MAX_SOLUTION_MOVES};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SingmasterError {
    Empty,
    TooManyMoves,
    InvalidToken { index: u8 },
}

/// Parses a whitespace-separated sequence such as `R U R' U'` into the
/// protocol's bounded move representation without allocating.
pub fn parse_singmaster(
    sequence: &str,
) -> Result<([CubeMove; MAX_SOLUTION_MOVES], u8), SingmasterError> {
    let empty = CubeMove {
        face: CubeFace::Up,
        turn: TurnAmount::Clockwise,
    };
    let mut moves = [empty; MAX_SOLUTION_MOVES];
    let mut count = 0usize;

    for token in sequence.split_ascii_whitespace() {
        if count == MAX_SOLUTION_MOVES {
            return Err(SingmasterError::TooManyMoves);
        }
        moves[count] =
            parse_token(token).ok_or(SingmasterError::InvalidToken { index: count as u8 })?;
        count += 1;
    }

    if count == 0 {
        return Err(SingmasterError::Empty);
    }
    Ok((moves, count as u8))
}

fn parse_token(token: &str) -> Option<CubeMove> {
    let bytes = token.as_bytes();
    let face = match bytes.first()?.to_ascii_uppercase() {
        b'U' => CubeFace::Up,
        b'R' => CubeFace::Right,
        b'F' => CubeFace::Front,
        b'D' => CubeFace::Down,
        b'L' => CubeFace::Left,
        b'B' => CubeFace::Back,
        _ => return None,
    };
    let turn = match bytes.get(1..) {
        Some([]) => TurnAmount::Clockwise,
        Some([b'\'']) => TurnAmount::CounterClockwise,
        Some([b'2']) => TurnAmount::Half,
        _ => return None,
    };
    Some(CubeMove { face, turn })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_standard_sequence_case_insensitively() {
        let (moves, count) = parse_singmaster("u' F B r2").unwrap();
        assert_eq!(count, 4);
        assert_eq!(
            &moves[..4],
            &[
                CubeMove {
                    face: CubeFace::Up,
                    turn: TurnAmount::CounterClockwise,
                },
                CubeMove {
                    face: CubeFace::Front,
                    turn: TurnAmount::Clockwise,
                },
                CubeMove {
                    face: CubeFace::Back,
                    turn: TurnAmount::Clockwise,
                },
                CubeMove {
                    face: CubeFace::Right,
                    turn: TurnAmount::Half,
                },
            ]
        );
    }

    #[test]
    fn rejects_empty_invalid_and_oversized_sequences() {
        assert_eq!(parse_singmaster("  "), Err(SingmasterError::Empty));
        assert_eq!(
            parse_singmaster("R X"),
            Err(SingmasterError::InvalidToken { index: 1 })
        );
        assert_eq!(
            parse_singmaster("R2'"),
            Err(SingmasterError::InvalidToken { index: 0 })
        );

        let too_many = "R R R R R R R R R R R R R R R R R R R R R R R R R R R R R R R R R";
        assert_eq!(
            parse_singmaster(too_many),
            Err(SingmasterError::TooManyMoves)
        );
    }
}
