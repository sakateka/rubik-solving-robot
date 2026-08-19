#![no_std]

//! Allocation-free routing core for the ESP32-C6 network/USB ↔ Duo UART gateway.

use rubik_link_protocol as link;

pub const NORMAL_QUEUE_CAPACITY: usize = 4;
pub const URGENT_QUEUE_CAPACITY: usize = 2;
pub const UPSTREAM_QUEUE_CAPACITY: usize = 4;
pub const REQUEST_ROUTE_CAPACITY: usize = NORMAL_QUEUE_CAPACITY + URGENT_QUEUE_CAPACITY;
pub const ABORT_RETRY_INTERVAL_MS: u64 = 100;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Upstream {
    Usb,
    Network,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum QueueKind {
    NormalToDuo,
    UrgentToDuo,
    ToUsb,
    ToNetwork,
    RequestRoutes,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GatewayError {
    FrameTooLong,
    Wire(link::WireError),
    QueueFull(QueueKind),
    UpstreamPacketMustBeRequest,
    UpstreamRequestIdMustBeNonZero,
    DuoPacketMustBeResponseOrEvent,
    DuplicateRequestId(u32),
    UnmatchedResponseRequestId(u32),
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

    fn remove_request_id(&mut self, request_id: u32) -> bool {
        let original_len = self.len;
        let mut removed = false;
        for _ in 0..original_len {
            let packet = self.pop().expect("queue length was captured");
            if !removed && packet.request_id() == request_id {
                removed = true;
            } else {
                self.push(packet).expect("removal preserves queue capacity");
            }
        }
        removed
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

#[derive(Clone, Copy)]
struct RequestRoute {
    request_id: u32,
    upstream: Upstream,
}

struct RequestRoutes {
    slots: [Option<RequestRoute>; REQUEST_ROUTE_CAPACITY],
}

impl RequestRoutes {
    const fn new() -> Self {
        Self {
            slots: [None; REQUEST_ROUTE_CAPACITY],
        }
    }

    fn insert(&mut self, route: RequestRoute) -> Result<(), GatewayError> {
        if self
            .slots
            .iter()
            .flatten()
            .any(|entry| entry.request_id == route.request_id)
        {
            return Err(GatewayError::DuplicateRequestId(route.request_id));
        }
        let slot = self
            .slots
            .iter_mut()
            .find(|entry| entry.is_none())
            .ok_or(GatewayError::QueueFull(QueueKind::RequestRoutes))?;
        *slot = Some(route);
        Ok(())
    }

    fn remove(&mut self, request_id: u32) -> Option<Upstream> {
        let slot = self
            .slots
            .iter_mut()
            .find(|entry| entry.is_some_and(|route| route.request_id == request_id))?;
        slot.take().map(|route| route.upstream)
    }

    fn remove_for(&mut self, request_id: u32, upstream: Upstream) -> bool {
        let Some(slot) = self.slots.iter_mut().find(|entry| {
            entry.is_some_and(|route| route.request_id == request_id && route.upstream == upstream)
        }) else {
            return false;
        };
        *slot = None;
        true
    }
}

/// Protocol-aware forwarding core. Hardware drivers feed bytes/packets into
/// it and drain complete frames without exposing UART, HTTP, or Wi-Fi types.
pub struct Gateway {
    usb_decoder: StreamDecoder,
    duo_decoder: StreamDecoder,
    urgent_to_duo: PacketQueue<URGENT_QUEUE_CAPACITY>,
    normal_to_duo: PacketQueue<NORMAL_QUEUE_CAPACITY>,
    to_usb: PacketQueue<UPSTREAM_QUEUE_CAPACITY>,
    to_network: PacketQueue<UPSTREAM_QUEUE_CAPACITY>,
    request_routes: RequestRoutes,
    event_upstream: Option<Upstream>,
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
            to_usb: PacketQueue::new(),
            to_network: PacketQueue::new(),
            request_routes: RequestRoutes::new(),
            event_upstream: None,
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
        Some(self.route_upstream_request(packet, Upstream::Usb))
    }

    /// Queues one transport-neutral packet received from HTTP/WebSocket.
    pub fn push_network_packet(&mut self, bytes: &[u8]) -> Result<RouteOutcome, GatewayError> {
        self.route_upstream_request(FixedPacket::from_bytes(bytes)?, Upstream::Network)
    }

    /// Compatibility alias for an alternative packet-oriented radio transport.
    pub fn push_ble_packet(&mut self, bytes: &[u8]) -> Result<RouteOutcome, GatewayError> {
        self.push_network_packet(bytes)
    }

    /// Feeds bytes returned by Duo. Responses return only to their requesting
    /// upstream; events are also published to the network observer.
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
        let Some(packet) = self.to_usb.pop() else {
            return Ok(None);
        };
        self.frame_packet(packet, output).map(Some)
    }

    /// Produces the next transport-neutral packet for HTTP/WebSocket. COBS is
    /// UART/USB specific and is intentionally absent here.
    pub fn dequeue_network_packet(
        &mut self,
        output: &mut [u8],
    ) -> Result<Option<usize>, GatewayError> {
        let Some(packet) = self.to_network.peek() else {
            return Ok(None);
        };
        if output.len() < packet.len {
            return Err(GatewayError::Wire(link::WireError::OutputTooSmall));
        }
        let packet = self.to_network.pop().expect("queue was non-empty");
        output[..packet.len].copy_from_slice(&packet.bytes[..packet.len]);
        Ok(Some(packet.len))
    }

    /// Compatibility alias for an alternative packet-oriented radio transport.
    pub fn dequeue_ble_packet(&mut self, output: &mut [u8]) -> Result<Option<usize>, GatewayError> {
        self.dequeue_network_packet(output)
    }

    /// Forgets a timed-out network request so disconnected Duo hardware cannot
    /// permanently exhaust the bounded request-route table.
    pub fn cancel_network_request(&mut self, request_id: u32) -> bool {
        if !self
            .request_routes
            .remove_for(request_id, Upstream::Network)
        {
            return false;
        }
        self.normal_to_duo.remove_request_id(request_id);
        self.urgent_to_duo.remove_request_id(request_id);
        if self
            .pending_abort
            .is_some_and(|pending| pending.packet.request_id() == request_id)
        {
            self.pending_abort = None;
        }
        true
    }

    pub const fn abort_pending(&self) -> bool {
        self.pending_abort.is_some()
    }

    fn route_upstream_request(
        &mut self,
        packet: FixedPacket,
        upstream: Upstream,
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
            self.request_routes.insert(RequestRoute {
                request_id: decoded.request_id,
                upstream,
            })?;
            self.urgent_to_duo.push(packet).map_err(|()| {
                self.request_routes.remove(decoded.request_id);
                GatewayError::QueueFull(QueueKind::UrgentToDuo)
            })?;
        } else {
            self.request_routes.insert(RequestRoute {
                request_id: decoded.request_id,
                upstream,
            })?;
            self.normal_to_duo.push(packet).map_err(|()| {
                self.request_routes.remove(decoded.request_id);
                GatewayError::QueueFull(QueueKind::NormalToDuo)
            })?;
        }
        Ok(RouteOutcome::Queued)
    }

    fn route_duo_packet(&mut self, packet: FixedPacket) -> Result<(), GatewayError> {
        let decoded = packet.packet();
        let kind = decoded.kind;
        let opcode = decoded.opcode;
        let request_id = decoded.request_id;
        match kind {
            link::MessageKind::Request => Err(GatewayError::DuoPacketMustBeResponseOrEvent),
            link::MessageKind::Response => {
                if self
                    .pending_abort
                    .is_some_and(|pending| pending.packet.request_id() == request_id)
                {
                    self.pending_abort = None;
                }
                let upstream = self
                    .request_routes
                    .remove(request_id)
                    .ok_or(GatewayError::UnmatchedResponseRequestId(request_id))?;
                if opcode == u16::from(link::ResponseOpcode::CommandAccepted) {
                    self.event_upstream = Some(upstream);
                }
                self.queue_upstream(upstream, packet)
            }
            link::MessageKind::Event => {
                if self.event_upstream == Some(Upstream::Usb) {
                    self.queue_upstream(Upstream::Usb, packet)?;
                }
                self.queue_upstream(Upstream::Network, packet)?;
                if matches!(
                    link::EventOpcode::try_from(opcode),
                    Ok(link::EventOpcode::OperationCompleted
                        | link::EventOpcode::Aborted
                        | link::EventOpcode::OperationFailed
                        | link::EventOpcode::Fault)
                ) {
                    self.event_upstream = None;
                }
                Ok(())
            }
        }
    }

    fn queue_upstream(
        &mut self,
        upstream: Upstream,
        packet: FixedPacket,
    ) -> Result<(), GatewayError> {
        let (queue, kind) = match upstream {
            Upstream::Usb => (&mut self.to_usb, QueueKind::ToUsb),
            Upstream::Network => (&mut self.to_network, QueueKind::ToNetwork),
        };
        queue
            .push(packet)
            .map_err(|()| GatewayError::QueueFull(kind))
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

    fn packet_bytes(kind: link::MessageKind, opcode: u16, request_id: u32) -> Vec<u8> {
        let packet = link::Packet {
            kind,
            opcode,
            request_id,
            payload: &[],
        };
        let mut bytes = [0; link::MAX_PACKET_LEN];
        let len = link::encode_packet(packet, &mut bytes).unwrap();
        bytes[..len].to_vec()
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

    fn dequeue_network(gateway: &mut Gateway) -> link::Packet<'static> {
        let mut bytes = [0; link::MAX_PACKET_LEN];
        let len = gateway.dequeue_network_packet(&mut bytes).unwrap().unwrap();
        let packet = link::decode_packet(&bytes[..len]).unwrap();
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
    fn response_returns_only_to_the_requesting_upstream() {
        let response = uart_frame(link::MessageKind::Response, 0x1002, 45);

        let mut usb_gateway = Gateway::new();
        push_usb(
            &mut usb_gateway,
            &uart_frame(link::MessageKind::Request, 1, 45),
        );
        dequeue_duo(&mut usb_gateway, 0);
        push_duo(&mut usb_gateway, &response);
        let mut usb = [0; link::MAX_UART_FRAME_LEN];
        let usb_len = usb_gateway.dequeue_usb_frame(&mut usb).unwrap().unwrap();
        assert_eq!(&usb[..usb_len], response.as_slice());
        assert!(usb_gateway
            .dequeue_network_packet(&mut [0; link::MAX_PACKET_LEN])
            .unwrap()
            .is_none());

        let mut network_gateway = Gateway::new();
        network_gateway
            .push_network_packet(&packet_bytes(link::MessageKind::Request, 1, 45))
            .unwrap();
        dequeue_duo(&mut network_gateway, 0);
        push_duo(&mut network_gateway, &response);
        let mut raw = [0; link::MAX_PACKET_LEN];
        let raw_len = network_gateway
            .dequeue_network_packet(&mut raw)
            .unwrap()
            .unwrap();
        let packet = link::decode_packet(&raw[..raw_len]).unwrap();
        assert_eq!(packet.request_id, 45);
        assert_eq!(packet.kind, link::MessageKind::Response);
        assert!(network_gateway
            .dequeue_usb_frame(&mut [0; link::MAX_UART_FRAME_LEN])
            .unwrap()
            .is_none());
    }

    #[test]
    fn simultaneous_usb_and_network_responses_do_not_cross() {
        let mut gateway = Gateway::new();
        push_usb(
            &mut gateway,
            &uart_frame(link::MessageKind::Request, 1, 100),
        );
        gateway
            .push_network_packet(&packet_bytes(link::MessageKind::Request, 1, 200))
            .unwrap();
        dequeue_duo(&mut gateway, 0);
        dequeue_duo(&mut gateway, 0);

        push_duo(
            &mut gateway,
            &uart_frame(link::MessageKind::Response, 0x1002, 200),
        );
        push_duo(
            &mut gateway,
            &uart_frame(link::MessageKind::Response, 0x1002, 100),
        );

        let mut raw = [0; link::MAX_PACKET_LEN];
        let len = gateway.dequeue_network_packet(&mut raw).unwrap().unwrap();
        assert_eq!(link::decode_packet(&raw[..len]).unwrap().request_id, 200);

        let mut frame = [0; link::MAX_UART_FRAME_LEN];
        let len = gateway.dequeue_usb_frame(&mut frame).unwrap().unwrap();
        let mut decoded = [0; link::MAX_PACKET_LEN];
        assert_eq!(
            link::parse_uart_frame(&frame[..len], &mut decoded)
                .unwrap()
                .request_id,
            100
        );
    }

    #[test]
    fn timed_out_network_request_releases_its_route_slot() {
        let mut gateway = Gateway::new();
        for request_id in 1..=REQUEST_ROUTE_CAPACITY as u32 {
            gateway
                .push_network_packet(&packet_bytes(link::MessageKind::Request, 1, request_id))
                .unwrap();
            dequeue_duo(&mut gateway, 0);
        }

        assert_eq!(
            gateway.push_network_packet(&packet_bytes(link::MessageKind::Request, 1, 100)),
            Err(GatewayError::QueueFull(QueueKind::RequestRoutes))
        );
        assert!(gateway.cancel_network_request(1));
        assert_eq!(
            gateway.push_network_packet(&packet_bytes(link::MessageKind::Request, 1, 100)),
            Ok(RouteOutcome::Queued)
        );

        let late_response = uart_frame(link::MessageKind::Response, 0x1002, 1);
        let result = late_response
            .iter()
            .filter_map(|&byte| gateway.push_duo_byte(byte))
            .last()
            .unwrap();
        assert_eq!(result, Err(GatewayError::UnmatchedResponseRequestId(1)));
    }

    #[test]
    fn unsolicited_duo_event_is_published_to_network_observer() {
        let mut gateway = Gateway::new();
        push_duo(
            &mut gateway,
            &uart_frame(
                link::MessageKind::Event,
                link::EventOpcode::StandStateChanged.into(),
                0,
            ),
        );

        let event = dequeue_network(&mut gateway);
        assert_eq!(event.kind, link::MessageKind::Event);
        assert_eq!(
            event.opcode,
            u16::from(link::EventOpcode::StandStateChanged)
        );
    }

    #[test]
    fn usb_operation_event_reaches_usb_owner_and_network_observer() {
        let mut gateway = Gateway::new();
        push_usb(
            &mut gateway,
            &uart_frame(link::MessageKind::Request, 0x0010, 42),
        );
        let _ = dequeue_duo(&mut gateway, 0);
        push_duo(
            &mut gateway,
            &uart_frame(
                link::MessageKind::Response,
                link::ResponseOpcode::CommandAccepted.into(),
                42,
            ),
        );
        let mut accepted_frame = [0; link::MAX_UART_FRAME_LEN];
        gateway
            .dequeue_usb_frame(&mut accepted_frame)
            .unwrap()
            .unwrap();
        push_duo(
            &mut gateway,
            &uart_frame(
                link::MessageKind::Event,
                link::EventOpcode::RobotStateChanged.into(),
                0,
            ),
        );

        let mut usb_frame = [0; link::MAX_UART_FRAME_LEN];
        let usb_len = gateway.dequeue_usb_frame(&mut usb_frame).unwrap().unwrap();
        let mut decoded = [0; link::MAX_PACKET_LEN];
        let usb_event = link::parse_uart_frame(&usb_frame[..usb_len], &mut decoded).unwrap();
        assert_eq!(usb_event.kind, link::MessageKind::Event);
        assert_eq!(dequeue_network(&mut gateway).kind, link::MessageKind::Event);
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
