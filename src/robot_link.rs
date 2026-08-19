//! Allocation-free packet framing at a byte-stream boundary.

use rubik_link_protocol::{
    parse_uart_frame, MessageKind, WireError, MAX_PACKET_LEN, MAX_PAYLOAD_LEN, MAX_UART_FRAME_LEN,
    UART_DELIMITER,
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReceivedPacket {
    pub kind: MessageKind,
    pub opcode: u16,
    pub request_id: u32,
    payload_len: usize,
    payload: [u8; MAX_PAYLOAD_LEN],
}

impl ReceivedPacket {
    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StreamDecodeError {
    FrameTooLong,
    Wire(WireError),
}

/// Incrementally recovers COBS frames from an arbitrary UART byte stream.
///
/// After an oversized or malformed frame, input is discarded only until the
/// next zero delimiter. The following frame starts from a clean boundary.
pub struct UartStreamDecoder {
    encoded: [u8; MAX_UART_FRAME_LEN],
    encoded_len: usize,
    decoded: [u8; MAX_PACKET_LEN],
    overflowed: bool,
}

impl Default for UartStreamDecoder {
    fn default() -> Self {
        Self {
            encoded: [0; MAX_UART_FRAME_LEN],
            encoded_len: 0,
            decoded: [0; MAX_PACKET_LEN],
            overflowed: false,
        }
    }
}

impl UartStreamDecoder {
    pub fn push(&mut self, byte: u8) -> Option<Result<ReceivedPacket, StreamDecodeError>> {
        if byte != UART_DELIMITER {
            if self.overflowed {
                return None;
            }
            if self.encoded_len >= MAX_UART_FRAME_LEN - 1 {
                self.overflowed = true;
                return None;
            }
            self.encoded[self.encoded_len] = byte;
            self.encoded_len += 1;
            return None;
        }

        if self.overflowed {
            self.reset_frame();
            return Some(Err(StreamDecodeError::FrameTooLong));
        }
        if self.encoded_len == 0 {
            return None;
        }

        self.encoded[self.encoded_len] = UART_DELIMITER;
        let frame_len = self.encoded_len + 1;
        let result = parse_uart_frame(&self.encoded[..frame_len], &mut self.decoded)
            .map_err(StreamDecodeError::Wire)
            .map(|packet| {
                let mut payload = [0; MAX_PAYLOAD_LEN];
                payload[..packet.payload.len()].copy_from_slice(packet.payload);
                ReceivedPacket {
                    kind: packet.kind,
                    opcode: packet.opcode,
                    request_id: packet.request_id,
                    payload_len: packet.payload.len(),
                    payload,
                }
            });
        self.reset_frame();
        Some(result)
    }

    fn reset_frame(&mut self) {
        self.encoded_len = 0;
        self.overflowed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rubik_link_protocol::{frame_uart, Packet};

    fn frame(request_id: u32, payload: &[u8]) -> Vec<u8> {
        let packet = Packet {
            kind: MessageKind::Request,
            opcode: 0x0010,
            request_id,
            payload,
        };
        let mut scratch = [0; MAX_PACKET_LEN];
        let mut output = [0; MAX_UART_FRAME_LEN];
        let len = frame_uart(packet, &mut scratch, &mut output).unwrap();
        output[..len].to_vec()
    }

    #[test]
    fn decodes_frames_split_at_every_byte_boundary() {
        let bytes = frame(17, &[0, 1, 0, 2]);
        let mut decoder = UartStreamDecoder::default();
        let mut result = None;

        for byte in bytes {
            if let Some(packet) = decoder.push(byte) {
                result = Some(packet.unwrap());
            }
        }

        let packet = result.unwrap();
        assert_eq!(packet.request_id, 17);
        assert_eq!(packet.payload(), &[0, 1, 0, 2]);
    }

    #[test]
    fn malformed_frame_does_not_poison_the_next_frame() {
        let valid = frame(23, &[]);
        let mut decoder = UartStreamDecoder::default();

        assert!(decoder.push(0x02).is_none());
        assert!(decoder.push(0xff).is_none());
        assert!(decoder.push(0).unwrap().is_err());

        let mut result = None;
        for byte in valid {
            if let Some(packet) = decoder.push(byte) {
                result = Some(packet.unwrap());
            }
        }
        assert_eq!(result.unwrap().request_id, 23);
    }

    #[test]
    fn oversized_frame_is_dropped_at_one_delimiter() {
        let mut decoder = UartStreamDecoder::default();
        for _ in 0..MAX_UART_FRAME_LEN + 10 {
            assert!(decoder.push(1).is_none());
        }
        assert_eq!(decoder.push(0), Some(Err(StreamDecodeError::FrameTooLong)));

        let mut result = None;
        for byte in frame(29, &[]) {
            if let Some(packet) = decoder.push(byte) {
                result = Some(packet.unwrap());
            }
        }
        assert_eq!(result.unwrap().request_id, 29);
    }
}
