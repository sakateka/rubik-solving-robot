use core::fmt;

use crate::{cobs, crc::crc16_ccitt_false};

pub const PROTOCOL_VERSION: u8 = 1;
pub const HEADER_LEN: usize = 10;
pub const CRC_LEN: usize = 2;
pub const MAX_PAYLOAD_LEN: usize = 1024;
pub const MAX_PACKET_LEN: usize = HEADER_LEN + MAX_PAYLOAD_LEN + CRC_LEN;
pub const MAX_UART_FRAME_LEN: usize = MAX_PACKET_LEN + MAX_PACKET_LEN / 254 + 2;
pub const UART_DELIMITER: u8 = 0;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum MessageKind {
    Request = 1,
    Response = 2,
    Event = 3,
}

impl TryFrom<u8> for MessageKind {
    type Error = WireError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Request),
            2 => Ok(Self::Response),
            3 => Ok(Self::Event),
            _ => Err(WireError::UnknownMessageKind(value)),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Packet<'a> {
    pub kind: MessageKind,
    pub opcode: u16,
    pub request_id: u32,
    pub payload: &'a [u8],
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WireError {
    OutputTooSmall,
    PayloadTooLarge,
    PacketTooShort,
    UnsupportedVersion(u8),
    UnknownMessageKind(u8),
    LengthMismatch,
    CrcMismatch { expected: u16, actual: u16 },
    MissingUartDelimiter,
    MalformedCobs,
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

pub fn encode_packet(packet: Packet<'_>, output: &mut [u8]) -> Result<usize, WireError> {
    if packet.payload.len() > MAX_PAYLOAD_LEN || packet.payload.len() > usize::from(u16::MAX) {
        return Err(WireError::PayloadTooLarge);
    }

    let packet_len = HEADER_LEN + packet.payload.len() + CRC_LEN;
    if output.len() < packet_len {
        return Err(WireError::OutputTooSmall);
    }

    output[0] = PROTOCOL_VERSION;
    output[1] = packet.kind as u8;
    output[2..4].copy_from_slice(&packet.opcode.to_le_bytes());
    output[4..8].copy_from_slice(&packet.request_id.to_le_bytes());
    output[8..10].copy_from_slice(&(packet.payload.len() as u16).to_le_bytes());
    output[HEADER_LEN..HEADER_LEN + packet.payload.len()].copy_from_slice(packet.payload);

    let crc_offset = HEADER_LEN + packet.payload.len();
    let crc = crc16_ccitt_false(&output[..crc_offset]);
    output[crc_offset..packet_len].copy_from_slice(&crc.to_le_bytes());
    Ok(packet_len)
}

pub fn decode_packet(input: &[u8]) -> Result<Packet<'_>, WireError> {
    if input.len() < HEADER_LEN + CRC_LEN {
        return Err(WireError::PacketTooShort);
    }
    if input[0] != PROTOCOL_VERSION {
        return Err(WireError::UnsupportedVersion(input[0]));
    }

    let payload_len = usize::from(u16::from_le_bytes([input[8], input[9]]));
    if payload_len > MAX_PAYLOAD_LEN {
        return Err(WireError::PayloadTooLarge);
    }
    let expected_len = HEADER_LEN + payload_len + CRC_LEN;
    if input.len() != expected_len {
        return Err(WireError::LengthMismatch);
    }

    let crc_offset = HEADER_LEN + payload_len;
    let expected = u16::from_le_bytes([input[crc_offset], input[crc_offset + 1]]);
    let actual = crc16_ccitt_false(&input[..crc_offset]);
    if expected != actual {
        return Err(WireError::CrcMismatch { expected, actual });
    }

    Ok(Packet {
        kind: MessageKind::try_from(input[1])?,
        opcode: u16::from_le_bytes([input[2], input[3]]),
        request_id: u32::from_le_bytes([input[4], input[5], input[6], input[7]]),
        payload: &input[HEADER_LEN..crc_offset],
    })
}

/// Encodes a transport-neutral packet as `COBS(packet) + 0x00`.
///
/// `packet_scratch` must fit the unframed packet. Keeping it separate from
/// `output` makes the function usable without allocation in `no_std` systems.
pub fn frame_uart(
    packet: Packet<'_>,
    packet_scratch: &mut [u8],
    output: &mut [u8],
) -> Result<usize, WireError> {
    let packet_len = encode_packet(packet, packet_scratch)?;
    let encoded_max = cobs::max_encoded_len(packet_len);
    if output.len() < encoded_max + 1 {
        return Err(WireError::OutputTooSmall);
    }

    let encoded_len = cobs::encode(&packet_scratch[..packet_len], output)?;
    output[encoded_len] = UART_DELIMITER;
    Ok(encoded_len + 1)
}

/// Decodes one complete UART frame, including its trailing `0x00` delimiter.
pub fn parse_uart_frame<'a>(frame: &[u8], decoded: &'a mut [u8]) -> Result<Packet<'a>, WireError> {
    if frame.last().copied() != Some(UART_DELIMITER) {
        return Err(WireError::MissingUartDelimiter);
    }

    let decoded_len = cobs::decode(&frame[..frame.len() - 1], decoded)?;
    decode_packet(&decoded[..decoded_len])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn transport_packet_round_trip() {
        let original = Packet {
            kind: MessageKind::Request,
            opcode: 0x0010,
            request_id: 0x1234_5678,
            payload: &[0, 1, 2, 0, 3],
        };
        let mut bytes = [0u8; 64];

        let len = encode_packet(original, &mut bytes).unwrap();
        let decoded = decode_packet(&bytes[..len]).unwrap();

        assert_eq!(decoded, original);
    }

    #[test]
    fn uart_frame_has_one_trailing_zero_and_round_trips() {
        let original = Packet {
            kind: MessageKind::Event,
            opcode: 0x0201,
            request_id: 0,
            payload: &[0, 10, 0, 20],
        };
        let mut packet_scratch = [0u8; 64];
        let mut framed = [0u8; 80];
        let mut decoded = [0u8; 64];

        let len = frame_uart(original, &mut packet_scratch, &mut framed).unwrap();
        assert_eq!(framed[len - 1], UART_DELIMITER);
        assert!(!framed[..len - 1].contains(&UART_DELIMITER));
        assert_eq!(parse_uart_frame(&framed[..len], &mut decoded), Ok(original));
    }

    #[test]
    fn detects_corrupted_packet() {
        let original = Packet {
            kind: MessageKind::Response,
            opcode: 1,
            request_id: 7,
            payload: b"status",
        };
        let mut bytes = [0u8; 64];
        let len = encode_packet(original, &mut bytes).unwrap();
        bytes[HEADER_LEN] ^= 0x80;

        assert!(matches!(
            decode_packet(&bytes[..len]),
            Err(WireError::CrcMismatch { .. })
        ));
    }

    #[test]
    fn rejects_truncated_uart_frame() {
        let mut decoded = [0u8; 64];
        assert_eq!(
            parse_uart_frame(&[1, 2, 3], &mut decoded),
            Err(WireError::MissingUartDelimiter)
        );
    }

    #[test]
    fn maximum_payload_fits_public_static_buffer_sizes() {
        let payload = [0u8; MAX_PAYLOAD_LEN];
        let original = Packet {
            kind: MessageKind::Response,
            opcode: 1,
            request_id: 42,
            payload: &payload,
        };
        let mut packet_scratch = [0u8; MAX_PACKET_LEN];
        let mut framed = [0u8; MAX_UART_FRAME_LEN];
        let mut decoded = [0u8; MAX_PACKET_LEN];

        let len = frame_uart(original, &mut packet_scratch, &mut framed).unwrap();
        assert!(len <= MAX_UART_FRAME_LEN);
        assert_eq!(parse_uart_frame(&framed[..len], &mut decoded), Ok(original));
    }
}
