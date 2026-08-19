#![no_std]

//! Allocation-free routing core for the ESP32-C6 BLE/USB ↔ Duo UART gateway.

use rubik_link_protocol as link;

pub const NORMAL_QUEUE_CAPACITY: usize = 4;
pub const URGENT_QUEUE_CAPACITY: usize = 2;
pub const UPSTREAM_QUEUE_CAPACITY: usize = 4;
pub const ABORT_RETRY_INTERVAL_MS: u64 = 100;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueueKind {
    NormalToDuo,
    UrgentToDuo,
    ToUpstream,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GatewayError {
    FrameTooLong,
    Wire(link::WireError),
    QueueFull(QueueKind),
    UpstreamPacketMustBeRequest,
    UpstreamRequestIdMustBeNonZero,
    DuoPacketMustBeResponseOrEvent,
}

impl From<link::WireError> for GatewayError {
    fn from(value: link::WireError) -> Self {
        Self::Wire(value)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteOutcome {
    Queued,
    DuplicateAbortIgnored,
}

#[derive(Clone, Copy)]
struct FixedPacket {
    bytes: [u8; link::MAX_PACKET_LEN],
    len: usize,
}

impl FixedPacket {
    const EMPTY: Self = Self {
        bytes: [0; link::MAX_PACKET_LEN],
        len: 0,
    };

    fn from_packet(packet: link::Packet<'_>) -> Result<Self, GatewayError> {
        let mut stored = Self::EMPTY;
        stored.len = link::encode_packet(packet, &mut stored.bytes)?;
        Ok(stored)
    }

    fn from_bytes(bytes: &[u8]) -> Result<Self, GatewayError> {
        let packet = link::decode_packet(bytes)?;
        Self::from_packet(packet)
    }

    fn packet(&self) -> link::Packet<'_> {
        link::decode_packet(&self.bytes[..self.len]).expect("stored packet was validated")
    }

    fn request_id(&self) -> u32 {
        self.packet().request_id
    }

    fn is_abort_request(&self) -> bool {
        let packet = self.packet();
        packet.kind == link::MessageKind::Request
            && packet.opcode == u16::from(link::RequestOpcode::Abort)
    }
}

struct PacketQueue<const N: usize> {
    slots: [FixedPacket; N],
    head: usize,
    len: usize,
}

impl<const N: usize> PacketQueue<N> {
    const fn new() -> Self {
        Self {
            slots: [FixedPacket::EMPTY; N],
            head: 0,
            len: 0,
        }
    }

    fn push(&mut self, packet: FixedPacket) -> Result<(), ()> {
        if self.len == N {
            return Err(());
        }
        let tail = (self.head + self.len) % N;
        self.slots[tail] = packet;
        self.len += 1;
        Ok(())
    }

    fn pop(&mut self) -> Option<FixedPacket> {
        if self.len == 0 {
            return None;
        }
        let packet = self.slots[self.head];
        self.head = (self.head + 1) % N;
        self.len -= 1;
        Some(packet)
    }

    fn peek(&self) -> Option<&FixedPacket> {
        (self.len != 0).then(|| &self.slots[self.head])
    }

    fn contains_request_id(&self, request_id: u32) -> bool {
        (0..self.len).any(|offset| self.slots[(self.head + offset) % N].request_id() == request_id)
    }
}

struct StreamDecoder {
    encoded: [u8; link::MAX_UART_FRAME_LEN],
    encoded_len: usize,
    decoded: [u8; link::MAX_PACKET_LEN],
    overflowed: bool,
}

impl StreamDecoder {
    const fn new() -> Self {
        Self {
            encoded: [0; link::MAX_UART_FRAME_LEN],
            encoded_len: 0,
            decoded: [0; link::MAX_PACKET_LEN],
            overflowed: false,
        }
    }

    fn push(&mut self, byte: u8) -> Option<Result<FixedPacket, GatewayError>> {
        if byte != link::UART_DELIMITER {
            if self.overflowed {
                return None;
            }
            if self.encoded_len >= link::MAX_UART_FRAME_LEN - 1 {
                self.overflowed = true;
                return None;
            }
            self.encoded[self.encoded_len] = byte;
            self.encoded_len += 1;
            return None;
        }

        if self.overflowed {
            self.reset();
            return Some(Err(GatewayError::FrameTooLong));
        }
        if self.encoded_len == 0 {
            return None;
        }

        self.encoded[self.encoded_len] = link::UART_DELIMITER;
        let frame_len = self.encoded_len + 1;
        let result = link::parse_uart_frame(&self.encoded[..frame_len], &mut self.decoded)
            .map_err(GatewayError::Wire)
            .and_then(FixedPacket::from_packet);
        self.reset();
        Some(result)
    }

    fn reset(&mut self) {
        self.encoded_len = 0;
        self.overflowed = false;
    }
}

#[derive(Clone, Copy)]
struct PendingAbort {
    packet: FixedPacket,
    next_retry_ms: u64,
}

/// Protocol-aware forwarding core. Hardware drivers feed bytes/packets into
/// it and drain complete frames without exposing UART or BLE types here.
pub struct Gateway {
    usb_decoder: StreamDecoder,
    duo_decoder: StreamDecoder,
    urgent_to_duo: PacketQueue<URGENT_QUEUE_CAPACITY>,
    normal_to_duo: PacketQueue<NORMAL_QUEUE_CAPACITY>,
    to_upstream: PacketQueue<UPSTREAM_QUEUE_CAPACITY>,
    pending_abort: Option<PendingAbort>,
    packet_scratch: [u8; link::MAX_PACKET_LEN],
}

impl Default for Gateway {
    fn default() -> Self {
        Self::new()
    }
}

impl Gateway {
    pub const fn new() -> Self {
        Self {
            usb_decoder: StreamDecoder::new(),
            duo_decoder: StreamDecoder::new(),
            urgent_to_duo: PacketQueue::new(),
            normal_to_duo: PacketQueue::new(),
            to_upstream: PacketQueue::new(),
            pending_abort: None,
            packet_scratch: [0; link::MAX_PACKET_LEN],
        }
    }

    /// Feeds development USB Serial/JTAG bytes encoded exactly like UART.
    pub fn push_usb_byte(&mut self, byte: u8) -> Option<Result<RouteOutcome, GatewayError>> {
        let packet = match self.usb_decoder.push(byte)? {
            Ok(packet) => packet,
            Err(error) => return Some(Err(error)),
        };
        Some(self.route_upstream_request(packet))
    }

    /// Queues one transport-neutral packet received from BLE.
    pub fn push_ble_packet(&mut self, bytes: &[u8]) -> Result<RouteOutcome, GatewayError> {
        self.route_upstream_request(FixedPacket::from_bytes(bytes)?)
    }

    /// Feeds bytes returned by Duo. Complete responses/events are queued for
    /// either USB framing or future BLE delivery.
    pub fn push_duo_byte(&mut self, byte: u8) -> Option<Result<(), GatewayError>> {
        let packet = match self.duo_decoder.push(byte)? {
            Ok(packet) => packet,
            Err(error) => return Some(Err(error)),
        };
        Some(self.route_duo_packet(packet))
    }

    /// Produces the next complete COBS frame for Duo. Abort traffic overtakes
    /// queued normal requests and is retried until a matching response arrives.
    pub fn dequeue_duo_uart_frame(
        &mut self,
        now_ms: u64,
        output: &mut [u8],
    ) -> Result<Option<usize>, GatewayError> {
        let packet = if let Some(packet) = self.urgent_to_duo.pop() {
            Some(packet)
        } else if let Some(pending) = self.pending_abort {
            (now_ms >= pending.next_retry_ms).then_some(pending.packet)
        } else {
            None
        }
        .or_else(|| self.normal_to_duo.pop());

        let Some(packet) = packet else {
            return Ok(None);
        };
        if packet.is_abort_request() {
            self.pending_abort = Some(PendingAbort {
                packet,
                next_retry_ms: now_ms.saturating_add(ABORT_RETRY_INTERVAL_MS),
            });
        }
        self.frame_packet(packet, output).map(Some)
    }

    /// Produces the next complete COBS frame for the development USB link.
    pub fn dequeue_usb_frame(&mut self, output: &mut [u8]) -> Result<Option<usize>, GatewayError> {
        let Some(packet) = self.to_upstream.pop() else {
            return Ok(None);
        };
        self.frame_packet(packet, output).map(Some)
    }

    /// Produces the next transport-neutral packet for BLE. COBS is UART/USB
    /// specific and is intentionally absent here.
    pub fn dequeue_ble_packet(&mut self, output: &mut [u8]) -> Result<Option<usize>, GatewayError> {
        let Some(packet) = self.to_upstream.peek() else {
            return Ok(None);
        };
        if output.len() < packet.len {
            return Err(GatewayError::Wire(link::WireError::OutputTooSmall));
        }
        let packet = self.to_upstream.pop().expect("queue was non-empty");
        output[..packet.len].copy_from_slice(&packet.bytes[..packet.len]);
        Ok(Some(packet.len))
    }

    pub const fn abort_pending(&self) -> bool {
        self.pending_abort.is_some()
    }

    fn route_upstream_request(
        &mut self,
        packet: FixedPacket,
    ) -> Result<RouteOutcome, GatewayError> {
        let decoded = packet.packet();
        if decoded.kind != link::MessageKind::Request {
            return Err(GatewayError::UpstreamPacketMustBeRequest);
        }
        if decoded.request_id == 0 {
            return Err(GatewayError::UpstreamRequestIdMustBeNonZero);
        }

        if packet.is_abort_request() {
            let duplicate_pending = self
                .pending_abort
                .is_some_and(|pending| pending.packet.request_id() == decoded.request_id);
            if duplicate_pending || self.urgent_to_duo.contains_request_id(decoded.request_id) {
                return Ok(RouteOutcome::DuplicateAbortIgnored);
            }
            self.urgent_to_duo
                .push(packet)
                .map_err(|()| GatewayError::QueueFull(QueueKind::UrgentToDuo))?;
        } else {
            self.normal_to_duo
                .push(packet)
                .map_err(|()| GatewayError::QueueFull(QueueKind::NormalToDuo))?;
        }
        Ok(RouteOutcome::Queued)
    }

    fn route_duo_packet(&mut self, packet: FixedPacket) -> Result<(), GatewayError> {
        let decoded = packet.packet();
        if decoded.kind == link::MessageKind::Request {
            return Err(GatewayError::DuoPacketMustBeResponseOrEvent);
        }
        if decoded.kind == link::MessageKind::Response
            && self
                .pending_abort
                .is_some_and(|pending| pending.packet.request_id() == decoded.request_id)
        {
            self.pending_abort = None;
        }
        self.to_upstream
            .push(packet)
            .map_err(|()| GatewayError::QueueFull(QueueKind::ToUpstream))
    }

    fn frame_packet(
        &mut self,
        packet: FixedPacket,
        output: &mut [u8],
    ) -> Result<usize, GatewayError> {
        link::frame_uart(packet.packet(), &mut self.packet_scratch, output)
            .map_err(GatewayError::Wire)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec::Vec;

    fn uart_frame(kind: link::MessageKind, opcode: u16, request_id: u32) -> Vec<u8> {
        let packet = link::Packet {
            kind,
            opcode,
            request_id,
            payload: &[],
        };
        let mut scratch = [0; link::MAX_PACKET_LEN];
        let mut frame = [0; link::MAX_UART_FRAME_LEN];
        let len = link::frame_uart(packet, &mut scratch, &mut frame).unwrap();
        frame[..len].to_vec()
    }

    fn push_usb(gateway: &mut Gateway, frame: &[u8]) -> RouteOutcome {
        frame
            .iter()
            .find_map(|&byte| gateway.push_usb_byte(byte))
            .unwrap()
            .unwrap()
    }

    fn push_duo(gateway: &mut Gateway, frame: &[u8]) {
        frame
            .iter()
            .find_map(|&byte| gateway.push_duo_byte(byte))
            .unwrap()
            .unwrap();
    }

    fn dequeue_duo(gateway: &mut Gateway, now_ms: u64) -> link::Packet<'static> {
        let mut frame = [0; link::MAX_UART_FRAME_LEN];
        let len = gateway
            .dequeue_duo_uart_frame(now_ms, &mut frame)
            .unwrap()
            .unwrap();
        let mut decoded = [0; link::MAX_PACKET_LEN];
        let packet = link::parse_uart_frame(&frame[..len], &mut decoded).unwrap();
        // Tests only inspect the header; avoid returning a borrow into `decoded`.
        link::Packet {
            kind: packet.kind,
            opcode: packet.opcode,
            request_id: packet.request_id,
            payload: &[],
        }
    }

    #[test]
    fn abort_overtakes_normal_requests_without_corrupting_frames() {
        let mut gateway = Gateway::new();
        push_usb(
            &mut gateway,
            &uart_frame(link::MessageKind::Request, 0x0010, 10),
        );
        push_usb(
            &mut gateway,
            &uart_frame(
                link::MessageKind::Request,
                link::RequestOpcode::Abort.into(),
                11,
            ),
        );

        assert_eq!(dequeue_duo(&mut gateway, 0).request_id, 11);
        assert_eq!(dequeue_duo(&mut gateway, 1).request_id, 10);
    }

    #[test]
    fn abort_retries_until_matching_response_arrives() {
        let mut gateway = Gateway::new();
        let abort = uart_frame(
            link::MessageKind::Request,
            link::RequestOpcode::Abort.into(),
            77,
        );
        assert_eq!(push_usb(&mut gateway, &abort), RouteOutcome::Queued);
        assert_eq!(dequeue_duo(&mut gateway, 0).request_id, 77);
        assert!(gateway.abort_pending());
        assert!(gateway
            .dequeue_duo_uart_frame(99, &mut [0; link::MAX_UART_FRAME_LEN])
            .unwrap()
            .is_none());
        assert_eq!(dequeue_duo(&mut gateway, 100).request_id, 77);

        let response = uart_frame(
            link::MessageKind::Response,
            link::ResponseOpcode::CommandAccepted.into(),
            77,
        );
        push_duo(&mut gateway, &response);
        assert!(!gateway.abort_pending());
        assert!(gateway
            .dequeue_duo_uart_frame(1_000, &mut [0; link::MAX_UART_FRAME_LEN])
            .unwrap()
            .is_none());
    }

    #[test]
    fn duplicate_abort_is_not_queued_twice() {
        let mut gateway = Gateway::new();
        let abort = uart_frame(
            link::MessageKind::Request,
            link::RequestOpcode::Abort.into(),
            91,
        );

        assert_eq!(push_usb(&mut gateway, &abort), RouteOutcome::Queued);
        assert_eq!(
            push_usb(&mut gateway, &abort),
            RouteOutcome::DuplicateAbortIgnored
        );
        assert_eq!(dequeue_duo(&mut gateway, 0).request_id, 91);
    }

    #[test]
    fn malformed_usb_frame_does_not_poison_the_next_packet() {
        let mut gateway = Gateway::new();
        assert!(gateway.push_usb_byte(2).is_none());
        assert!(gateway.push_usb_byte(0xff).is_none());
        assert!(gateway.push_usb_byte(0).unwrap().is_err());

        let valid = uart_frame(link::MessageKind::Request, 1, 123);
        assert_eq!(push_usb(&mut gateway, &valid), RouteOutcome::Queued);
        assert_eq!(dequeue_duo(&mut gateway, 0).request_id, 123);
    }

    #[test]
    fn duo_response_can_be_drained_as_usb_or_ble_packet() {
        let response = uart_frame(link::MessageKind::Response, 0x1002, 45);

        let mut usb_gateway = Gateway::new();
        push_duo(&mut usb_gateway, &response);
        let mut usb = [0; link::MAX_UART_FRAME_LEN];
        let usb_len = usb_gateway.dequeue_usb_frame(&mut usb).unwrap().unwrap();
        assert_eq!(&usb[..usb_len], response.as_slice());

        let mut ble_gateway = Gateway::new();
        push_duo(&mut ble_gateway, &response);
        let mut raw = [0; link::MAX_PACKET_LEN];
        let raw_len = ble_gateway.dequeue_ble_packet(&mut raw).unwrap().unwrap();
        let packet = link::decode_packet(&raw[..raw_len]).unwrap();
        assert_eq!(packet.request_id, 45);
        assert_eq!(packet.kind, link::MessageKind::Response);
    }

    #[test]
    fn normal_queue_has_a_hard_capacity() {
        let mut gateway = Gateway::new();
        for request_id in 1..=NORMAL_QUEUE_CAPACITY as u32 {
            assert_eq!(
                push_usb(
                    &mut gateway,
                    &uart_frame(link::MessageKind::Request, 1, request_id)
                ),
                RouteOutcome::Queued
            );
        }
        let overflow = uart_frame(link::MessageKind::Request, 1, 99);
        let result = overflow
            .iter()
            .find_map(|&byte| gateway.push_usb_byte(byte))
            .unwrap();
        assert_eq!(result, Err(GatewayError::QueueFull(QueueKind::NormalToDuo)));
    }
}
