use core::fmt;

use serde::{Deserialize, Serialize};

#[derive(Debug)]
pub enum PayloadError {
    Serialize(postcard::Error),
    Deserialize(postcard::Error),
    TrailingBytes,
}

impl fmt::Display for PayloadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

pub fn encode_payload<'a, T>(value: &T, output: &'a mut [u8]) -> Result<&'a mut [u8], PayloadError>
where
    T: Serialize + ?Sized,
{
    postcard::to_slice(value, output).map_err(PayloadError::Serialize)
}

pub fn decode_payload<'a, T>(input: &'a [u8]) -> Result<T, PayloadError>
where
    T: Deserialize<'a>,
{
    let (value, remaining) = postcard::take_from_bytes(input).map_err(PayloadError::Deserialize)?;
    if !remaining.is_empty() {
        return Err(PayloadError::TrailingBytes);
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{CommandAccepted, CubeFace, CubeMove, RejectionReason, TurnAmount};

    #[test]
    fn command_accepted_has_stable_golden_encoding() {
        let accepted = CommandAccepted {
            operation_id: Some(0x1234),
        };
        let mut output = [0u8; 8];

        let encoded = encode_payload(&accepted, &mut output).unwrap();

        assert_eq!(encoded, &[0x01, 0xb4, 0x24]);
        assert_eq!(
            decode_payload::<CommandAccepted>(encoded).unwrap(),
            accepted
        );
    }

    #[test]
    fn explicit_repr_enums_have_stable_golden_encoding() {
        let cube_move = CubeMove {
            face: CubeFace::Front,
            turn: TurnAmount::CounterClockwise,
        };
        let mut output = [0u8; 8];

        let encoded = encode_payload(&cube_move, &mut output).unwrap();

        assert_eq!(encoded, &[0x02, 0x01]);
        assert_eq!(decode_payload::<CubeMove>(encoded).unwrap(), cube_move);
    }

    #[test]
    fn rejects_trailing_payload_bytes() {
        let result = decode_payload::<RejectionReason>(&[0x01, 0x00]);
        assert!(matches!(result, Err(PayloadError::TrailingBytes)));
    }
}
