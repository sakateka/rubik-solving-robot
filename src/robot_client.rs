//! Transport-independent client state machine for the robot control protocol.

use crate::robot_link::{FrameEncodeError, StreamDecodeError, UartFrameEncoder, UartStreamDecoder};
use rubik_link_protocol as link;
use std::{error::Error, fmt};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClientCommand {
    GetStatus,
    Grip,
    StartScan(link::StartScanCommand),
    Solve(link::SolveCommand),
    Execute(link::ExecuteCommand),
    ScanSolveExecute(link::ScanSolveExecuteCommand),
    Open(link::OpenCommand),
    RecoverToOpen,
    ExecuteMoves(link::ExecuteMovesCommand),
    Abort,
}

impl ClientCommand {
    pub const fn opcode(self) -> link::RequestOpcode {
        match self {
            Self::GetStatus => link::RequestOpcode::GetStatus,
            Self::Grip => link::RequestOpcode::Grip,
            Self::StartScan(_) => link::RequestOpcode::StartScan,
            Self::Solve(_) => link::RequestOpcode::Solve,
            Self::Execute(_) => link::RequestOpcode::Execute,
            Self::ScanSolveExecute(_) => link::RequestOpcode::ScanSolveExecute,
            Self::Open(_) => link::RequestOpcode::Open,
            Self::RecoverToOpen => link::RequestOpcode::RecoverToOpen,
            Self::ExecuteMoves(_) => link::RequestOpcode::ExecuteMoves,
            Self::Abort => link::RequestOpcode::Abort,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ClientResponse {
    Accepted {
        request_id: u32,
        payload: link::CommandAccepted,
    },
    Rejected {
        request_id: u32,
        payload: link::CommandRejected,
    },
    Status {
        request_id: u32,
        snapshot: Box<link::StatusSnapshot>,
    },
}

impl ClientResponse {
    pub const fn request_id(&self) -> u32 {
        match self {
            Self::Accepted { request_id, .. }
            | Self::Rejected { request_id, .. }
            | Self::Status { request_id, .. } => *request_id,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ClientEvent {
    RobotStateChanged(link::RobotStateChanged),
    StandStateChanged(link::StandStateChanged),
    FaceScanned(link::FaceScanned),
    PlanChanged(Box<link::PlanChanged>),
    ActionStarted(link::ActionProgress),
    ActionCompleted(link::ActionProgress),
    OperationCompleted(link::OperationCompleted),
    Aborted(link::Aborted),
    CubeSessionChanged(link::CubeSessionChanged),
    OperationFailed(link::OperationFailed),
    Fault(link::FaultEvent),
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ClientMessage {
    Response(ClientResponse),
    Event(ClientEvent),
}

#[derive(Debug)]
pub enum ClientError {
    Encode(FrameEncodeError),
    Stream(StreamDecodeError),
    Payload(link::PayloadError),
    UnexpectedMessageKind(link::MessageKind),
    InvalidRequestId {
        kind: link::MessageKind,
        request_id: u32,
    },
    UnknownOpcode {
        kind: link::MessageKind,
        opcode: u16,
    },
    InvalidSchema(link::SchemaError),
}

impl fmt::Display for ClientError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Encode(error) => write!(f, "{error}"),
            Self::Stream(error) => write!(f, "invalid UART frame: {error:?}"),
            Self::Payload(error) => write!(f, "invalid protocol payload: {error}"),
            Self::UnexpectedMessageKind(kind) => write!(f, "unexpected inbound {kind:?} packet"),
            Self::InvalidRequestId { kind, request_id } => {
                write!(f, "invalid request ID {request_id} for {kind:?} packet")
            }
            Self::UnknownOpcode { kind, opcode } => {
                write!(f, "unknown {kind:?} opcode 0x{opcode:04x}")
            }
            Self::InvalidSchema(error) => write!(f, "invalid protocol state: {error:?}"),
        }
    }
}

impl Error for ClientError {}

impl From<FrameEncodeError> for ClientError {
    fn from(value: FrameEncodeError) -> Self {
        Self::Encode(value)
    }
}

impl From<link::PayloadError> for ClientError {
    fn from(value: link::PayloadError) -> Self {
        Self::Payload(value)
    }
}

pub struct EncodedRequest<'a> {
    pub request_id: u32,
    pub frame: &'a [u8],
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct ClientState {
    pub snapshot: Option<link::StatusSnapshot>,
}

/// Pure protocol state machine. UART, BLE, files and threads stay outside it.
pub struct RobotClient {
    encoder: UartFrameEncoder,
    decoder: UartStreamDecoder,
    next_request_id: u32,
    state: ClientState,
}

impl Default for RobotClient {
    fn default() -> Self {
        Self::with_initial_request_id(1)
    }
}

impl RobotClient {
    pub fn with_initial_request_id(request_id: u32) -> Self {
        Self {
            encoder: UartFrameEncoder::default(),
            decoder: UartStreamDecoder::default(),
            next_request_id: request_id.max(1),
            state: ClientState::default(),
        }
    }

    pub const fn state(&self) -> &ClientState {
        &self.state
    }

    pub fn encode_command(
        &mut self,
        command: ClientCommand,
    ) -> Result<EncodedRequest<'_>, ClientError> {
        let request_id = self.next_request_id;
        self.next_request_id = self.next_request_id.wrapping_add(1).max(1);
        let opcode = u16::from(command.opcode());
        let kind = link::MessageKind::Request;
        let frame = match command {
            ClientCommand::GetStatus
            | ClientCommand::Grip
            | ClientCommand::RecoverToOpen
            | ClientCommand::Abort => self.encoder.encode_empty(kind, opcode, request_id)?,
            ClientCommand::StartScan(payload) => {
                self.encoder.encode(kind, opcode, request_id, &payload)?
            }
            ClientCommand::Solve(payload) => {
                self.encoder.encode(kind, opcode, request_id, &payload)?
            }
            ClientCommand::Execute(payload) => {
                self.encoder.encode(kind, opcode, request_id, &payload)?
            }
            ClientCommand::ScanSolveExecute(payload) => {
                self.encoder.encode(kind, opcode, request_id, &payload)?
            }
            ClientCommand::Open(payload) => {
                self.encoder.encode(kind, opcode, request_id, &payload)?
            }
            ClientCommand::ExecuteMoves(payload) => {
                payload.validate().map_err(ClientError::InvalidSchema)?;
                self.encoder.encode(kind, opcode, request_id, &payload)?
            }
        };
        Ok(EncodedRequest { request_id, frame })
    }

    pub fn push_byte(&mut self, byte: u8) -> Option<Result<ClientMessage, ClientError>> {
        let packet = match self.decoder.push(byte)? {
            Ok(packet) => packet,
            Err(error) => return Some(Err(ClientError::Stream(error))),
        };
        let message = decode_message(&packet).and_then(|message| {
            self.apply(&message)?;
            Ok(message)
        });
        Some(message)
    }

    fn apply(&mut self, message: &ClientMessage) -> Result<(), ClientError> {
        if let ClientMessage::Response(ClientResponse::Status { snapshot, .. }) = message {
            snapshot.validate().map_err(ClientError::InvalidSchema)?;
            self.state.snapshot = Some(**snapshot);
            return Ok(());
        }

        let Some(snapshot) = self.state.snapshot.as_mut() else {
            return Ok(());
        };
        let ClientMessage::Event(event) = message else {
            return Ok(());
        };
        match event {
            ClientEvent::RobotStateChanged(event) => {
                snapshot.controller = event.controller;
                snapshot.active_operation = event.active_operation;
            }
            ClientEvent::StandStateChanged(event) => snapshot.stand = event.stand,
            ClientEvent::CubeSessionChanged(event) => snapshot.cube_session = event.session,
            ClientEvent::FaceScanned(event) => {
                snapshot.scan.current_face = Some(event.face);
                snapshot.scan.camera_face = Some(event.recognized);
                snapshot.scan.scanned_faces = event.scanned_faces;
                snapshot.scan.faces[event.face as usize] = Some(event.recognized);
            }
            ClientEvent::PlanChanged(event) => {
                if usize::from(event.action_count) > link::MAX_PLAN_PREVIEW_ACTIONS {
                    return Err(ClientError::InvalidSchema(
                        link::SchemaError::TooManyPreviewActions(event.action_count),
                    ));
                }
                snapshot.plan = event.actions;
                snapshot.plan_count = event.action_count;
            }
            ClientEvent::OperationCompleted(_) => {
                snapshot.active_operation = None;
                snapshot.controller = link::ControllerState::Ready;
            }
            ClientEvent::Aborted(_) => {
                snapshot.active_operation = None;
                snapshot.controller = link::ControllerState::Aborted;
                snapshot.cube_session = None;
            }
            ClientEvent::Fault(event) => {
                snapshot.active_operation = None;
                snapshot.controller = link::ControllerState::Faulted;
                snapshot.fault = Some(event.fault);
            }
            ClientEvent::ActionStarted(_)
            | ClientEvent::ActionCompleted(_)
            | ClientEvent::OperationFailed(_) => {}
        }
        Ok(())
    }
}

fn decode_message(
    packet: &crate::robot_link::ReceivedPacket,
) -> Result<ClientMessage, ClientError> {
    match packet.kind {
        link::MessageKind::Request => Err(ClientError::UnexpectedMessageKind(packet.kind)),
        link::MessageKind::Response => decode_response(packet).map(ClientMessage::Response),
        link::MessageKind::Event => decode_event(packet).map(ClientMessage::Event),
    }
}

fn decode_response(
    packet: &crate::robot_link::ReceivedPacket,
) -> Result<ClientResponse, ClientError> {
    if packet.request_id == 0 {
        return Err(ClientError::InvalidRequestId {
            kind: packet.kind,
            request_id: packet.request_id,
        });
    }
    let opcode = link::ResponseOpcode::try_from(packet.opcode).map_err(|opcode| {
        ClientError::UnknownOpcode {
            kind: packet.kind,
            opcode,
        }
    })?;
    match opcode {
        link::ResponseOpcode::CommandAccepted => Ok(ClientResponse::Accepted {
            request_id: packet.request_id,
            payload: link::decode_payload(packet.payload())?,
        }),
        link::ResponseOpcode::CommandRejected => Ok(ClientResponse::Rejected {
            request_id: packet.request_id,
            payload: link::decode_payload(packet.payload())?,
        }),
        link::ResponseOpcode::StatusSnapshot => {
            let snapshot: link::StatusSnapshot = link::decode_payload(packet.payload())?;
            snapshot.validate().map_err(ClientError::InvalidSchema)?;
            Ok(ClientResponse::Status {
                request_id: packet.request_id,
                snapshot: Box::new(snapshot),
            })
        }
    }
}

fn decode_event(packet: &crate::robot_link::ReceivedPacket) -> Result<ClientEvent, ClientError> {
    if packet.request_id != 0 {
        return Err(ClientError::InvalidRequestId {
            kind: packet.kind,
            request_id: packet.request_id,
        });
    }
    let opcode = link::EventOpcode::try_from(packet.opcode).map_err(|opcode| {
        ClientError::UnknownOpcode {
            kind: packet.kind,
            opcode,
        }
    })?;
    let payload = packet.payload();
    Ok(match opcode {
        link::EventOpcode::RobotStateChanged => {
            ClientEvent::RobotStateChanged(link::decode_payload(payload)?)
        }
        link::EventOpcode::StandStateChanged => {
            ClientEvent::StandStateChanged(link::decode_payload(payload)?)
        }
        link::EventOpcode::FaceScanned => ClientEvent::FaceScanned(link::decode_payload(payload)?),
        link::EventOpcode::PlanChanged => {
            let event: link::PlanChanged = link::decode_payload(payload)?;
            if usize::from(event.action_count) > link::MAX_PLAN_PREVIEW_ACTIONS {
                return Err(ClientError::InvalidSchema(
                    link::SchemaError::TooManyPreviewActions(event.action_count),
                ));
            }
            ClientEvent::PlanChanged(Box::new(event))
        }
        link::EventOpcode::ActionStarted => {
            ClientEvent::ActionStarted(link::decode_payload(payload)?)
        }
        link::EventOpcode::ActionCompleted => {
            ClientEvent::ActionCompleted(link::decode_payload(payload)?)
        }
        link::EventOpcode::OperationCompleted => {
            ClientEvent::OperationCompleted(link::decode_payload(payload)?)
        }
        link::EventOpcode::Aborted => ClientEvent::Aborted(link::decode_payload(payload)?),
        link::EventOpcode::CubeSessionChanged => {
            ClientEvent::CubeSessionChanged(link::decode_payload(payload)?)
        }
        link::EventOpcode::OperationFailed => {
            ClientEvent::OperationFailed(link::decode_payload(payload)?)
        }
        link::EventOpcode::Fault => ClientEvent::Fault(link::decode_payload(payload)?),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allocates_request_ids_and_encodes_empty_payload() {
        let mut client = RobotClient::default();
        let first = client.encode_command(ClientCommand::GetStatus).unwrap();
        let first_bytes = first.frame.to_vec();
        assert_eq!(first.request_id, 1);

        let second = client.encode_command(ClientCommand::Abort).unwrap();
        assert_eq!(second.request_id, 2);

        let mut decoder = UartStreamDecoder::default();
        let packet = first_bytes
            .into_iter()
            .find_map(|byte| decoder.push(byte))
            .unwrap()
            .unwrap();
        assert_eq!(packet.opcode, u16::from(link::RequestOpcode::GetStatus));
        assert!(packet.payload().is_empty());
    }

    #[test]
    fn accepts_a_process_specific_initial_request_id() {
        let mut client = RobotClient::with_initial_request_id(0x1234_5678);

        let request = client.encode_command(ClientCommand::Abort).unwrap();

        assert_eq!(request.request_id, 0x1234_5678);
    }

    #[cfg(feature = "pca9685")]
    mod end_to_end {
        use super::*;
        use crate::{
            pca9685::PwmOutput,
            robot_service::{RobotService, ServiceMessage},
            stand::StandCalibration,
        };
        use anyhow::Result;
        use std::time::{Duration, Instant};

        #[derive(Default)]
        struct MockOutput;

        impl PwmOutput for MockOutput {
            fn set_channels(&mut self, _channels: &[(u8, u16)]) -> Result<()> {
                Ok(())
            }

            fn disable_channels(&mut self, _channels: &[u8]) -> Result<()> {
                Ok(())
            }

            fn all_off(&mut self) -> Result<()> {
                Ok(())
            }
        }

        fn decode_one(frame: &[u8]) -> crate::robot_link::ReceivedPacket {
            let mut decoder = UartStreamDecoder::default();
            frame
                .iter()
                .find_map(|&byte| decoder.push(byte))
                .unwrap()
                .unwrap()
        }

        fn deliver(
            client: &mut RobotClient,
            encoder: &mut UartFrameEncoder,
            messages: &[ServiceMessage],
        ) -> Vec<ClientMessage> {
            let mut received = Vec::new();
            for message in messages {
                let frame = message.encode_uart(encoder).unwrap().to_vec();
                for byte in frame {
                    if let Some(result) = client.push_byte(byte) {
                        received.push(result.unwrap());
                    }
                }
            }
            received
        }

        fn request(
            client: &mut RobotClient,
            service: &mut RobotService<MockOutput>,
            server_encoder: &mut UartFrameEncoder,
            command: ClientCommand,
            now: Instant,
        ) -> Vec<ClientMessage> {
            let frame = client.encode_command(command).unwrap().frame.to_vec();
            let messages = service.handle_packet(&decode_one(&frame), now);
            deliver(client, server_encoder, &messages)
        }

        #[test]
        fn status_and_recovery_round_trip_as_real_uart_frames() {
            let base = Instant::now();
            let mut client = RobotClient::default();
            let mut service = RobotService::new(MockOutput, StandCalibration::default());
            let mut server_encoder = UartFrameEncoder::default();

            let status = request(
                &mut client,
                &mut service,
                &mut server_encoder,
                ClientCommand::GetStatus,
                base,
            );
            assert!(matches!(
                status.as_slice(),
                [ClientMessage::Response(ClientResponse::Status {
                    request_id: 1,
                    ..
                })]
            ));
            assert_eq!(
                client.state().snapshot.unwrap().stand.pose.kind,
                link::StandPoseKind::Unknown
            );

            let accepted = request(
                &mut client,
                &mut service,
                &mut server_encoder,
                ClientCommand::RecoverToOpen,
                base,
            );
            assert!(matches!(
                &accepted[0],
                ClientMessage::Response(ClientResponse::Accepted {
                    request_id: 2,
                    payload: link::CommandAccepted {
                        operation_id: Some(1)
                    }
                })
            ));

            for elapsed in [0, 1_200, 2_400, 3_400] {
                let messages = service.tick(base + Duration::from_millis(elapsed));
                deliver(&mut client, &mut server_encoder, &messages);
            }

            let snapshot = client.state().snapshot.unwrap();
            assert_eq!(snapshot.controller, link::ControllerState::Ready);
            assert_eq!(snapshot.stand.pose.kind, link::StandPoseKind::Open);
            assert!(snapshot.active_operation.is_none());
        }
    }
}
