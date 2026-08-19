#![no_std]
#![no_main]
#![recursion_limit = "256"]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::{
    net::Ipv4Addr,
    sync::atomic::{AtomicU32, Ordering},
};

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex, signal::Signal,
};
use embassy_time::{Duration, Timer, with_timeout};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    ram,
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart},
    usb_serial_jtag::UsbSerialJtag,
};
use esp_radio::wifi::{
    AuthenticationMethod, Config as WifiConfig, ControllerConfig, Interface, WifiController,
    ap::AccessPointConfig,
};
use picoserve::{
    response::{
        File, IntoResponse, Json, StatusCode, WebSocketUpgrade,
        ws::{Message, SocketRx, SocketTx, WebSocketCallback},
    },
    routing::{Router, get, get_service, post},
};
use rubik_link_gateway::Gateway;
use rubik_link_protocol::{self as link, CRC_LEN, HEADER_LEN, MAX_PACKET_LEN, MAX_UART_FRAME_LEN};
use serde::Serialize;
use static_cell::StaticCell;

mod app_config {
    include!(concat!(env!("OUT_DIR"), "/app_config.rs"));
}

esp_bootloader_esp_idf::esp_app_desc!();

const HTTP_RPC_TIMEOUT: Duration = Duration::from_secs(3);
const HTTP_REQUEST_ID_START: u32 = 0x8000_0001;
const HTTP_REQUEST_PAYLOAD_CAPACITY: usize = 64;
static NEXT_HTTP_REQUEST_ID: AtomicU32 = AtomicU32::new(HTTP_REQUEST_ID_START);
static NEXT_HTTP_EVENT_SEQUENCE: AtomicU32 = AtomicU32::new(1);
static HTTP_REQUESTS: Channel<CriticalSectionRawMutex, RequestPacket, 1> = Channel::new();
static HTTP_RESPONSES: Channel<CriticalSectionRawMutex, OwnedPacket, 1> = Channel::new();
static HTTP_CANCELLATIONS: Channel<CriticalSectionRawMutex, u32, 1> = Channel::new();
static HTTP_EVENT: Signal<CriticalSectionRawMutex, ApiEvent> = Signal::new();
static HTTP_RPC_LOCK: Mutex<CriticalSectionRawMutex, ()> = Mutex::new(());

struct RequestPacket {
    bytes: [u8; HEADER_LEN + HTTP_REQUEST_PAYLOAD_CAPACITY + CRC_LEN],
    len: usize,
}

impl RequestPacket {
    fn empty(opcode: link::RequestOpcode, request_id: u32) -> Result<Self, ()> {
        Self::from_payload(opcode, request_id, &[])
    }

    fn with_payload<T: Serialize>(
        opcode: link::RequestOpcode,
        request_id: u32,
        value: &T,
    ) -> Result<Self, ()> {
        let mut payload = [0; HTTP_REQUEST_PAYLOAD_CAPACITY];
        let payload = link::encode_payload(value, &mut payload).map_err(|_| ())?;
        Self::from_payload(opcode, request_id, payload)
    }

    fn from_payload(
        opcode: link::RequestOpcode,
        request_id: u32,
        payload: &[u8],
    ) -> Result<Self, ()> {
        let mut packet = Self {
            bytes: [0; HEADER_LEN + HTTP_REQUEST_PAYLOAD_CAPACITY + CRC_LEN],
            len: 0,
        };
        packet.len = link::encode_packet(
            link::Packet {
                kind: link::MessageKind::Request,
                opcode: opcode.into(),
                request_id,
                payload,
            },
            &mut packet.bytes,
        )
        .map_err(|_| ())?;
        Ok(packet)
    }

    fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

struct OwnedPacket {
    bytes: [u8; MAX_PACKET_LEN],
    len: usize,
}

#[derive(Clone, Copy, Serialize)]
struct ApiEvent {
    sequence: u32,
    event: &'static str,
}

mod execute_moves_json {
    #![allow(
        clippy::large_stack_frames,
        reason = "the JSON command owns one bounded 96-byte Singmaster string"
    )]

    use serde::Deserialize;

    #[derive(Deserialize)]
    pub(super) struct Request {
        pub(super) session_id: u32,
        pub(super) sequence: heapless::String<96>,
    }
}

struct EventWebSocket;

impl WebSocketCallback for EventWebSocket {
    async fn run<R: picoserve::io::Read, W: picoserve::io::Write<Error = R::Error>>(
        self,
        mut rx: SocketRx<R>,
        mut tx: SocketTx<W>,
    ) -> Result<(), W::Error> {
        tx.send_json(ApiEvent {
            sequence: 0,
            event: "connected",
        })
        .await?;

        let mut receive_buffer = [0; 64];
        loop {
            match rx
                .next_message(&mut receive_buffer, HTTP_EVENT.wait())
                .await?
            {
                picoserve::futures::Either::Second(event) => tx.send_json(event).await?,
                picoserve::futures::Either::First(Ok(Message::Ping(data))) => {
                    tx.send_pong(data).await?
                }
                picoserve::futures::Either::First(Ok(Message::Close(_))) => {
                    return tx.close(None).await;
                }
                picoserve::futures::Either::First(Ok(_)) => {}
                picoserve::futures::Either::First(Err(error)) => {
                    return tx
                        .close(Some((error.code(), "invalid websocket message")))
                        .await;
                }
            }
        }
    }
}

impl OwnedPacket {
    fn as_slice(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

#[derive(Serialize)]
#[serde(tag = "result", rename_all = "snake_case")]
enum ApiCommandBody {
    Accepted {
        request_id: u32,
        operation_id: Option<u32>,
    },
    Error {
        error: &'static str,
        reason: Option<link::RejectionReason>,
        controller: Option<link::ControllerState>,
    },
}

struct ApiCommandResponse {
    status: StatusCode,
    body: ApiCommandBody,
}

impl IntoResponse for ApiCommandResponse {
    async fn write_to<R, W>(
        self,
        connection: picoserve::response::Connection<'_, R>,
        response_writer: W,
    ) -> Result<picoserve::ResponseSent, W::Error>
    where
        R: picoserve::io::Read,
        W: picoserve::response::ResponseWriter<Error = R::Error>,
    {
        Json(self.body)
            .into_response()
            .with_status_code(self.status)
            .write_to(connection, response_writer)
            .await
    }
}

enum HttpRpcError {
    Timeout,
    InvalidPacket,
}

macro_rules! make_static {
    ($type:ty, $value:expr) => {{
        static CELL: StaticCell<$type> = StaticCell::new();
        CELL.uninit().write($value)
    }};
}

#[allow(
    clippy::large_stack_frames,
    reason = "esp-rtos stores the async main future in static executor task storage"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 40 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupts = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupts.software_interrupt0);

    let ap_config = AccessPointConfig::default()
        .with_ssid(app_config::WIFI_SSID)
        .with_password(app_config::WIFI_PASSWORD.into())
        .with_auth_method(AuthenticationMethod::Wpa2Personal)
        .with_channel(app_config::WIFI_CHANNEL)
        .with_max_connections(4);
    let (wifi_controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(WifiConfig::AccessPoint(ap_config)),
    )
    .expect("initialize Wi-Fi access point");

    let gateway_address = Ipv4Addr::from(app_config::HTTP_ADDRESS);
    let network_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(gateway_address, 24),
        gateway: Some(gateway_address),
        dns_servers: Default::default(),
    });
    let rng = Rng::new();
    let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
    let (network, network_runner) = embassy_net::new(
        interfaces.access_point,
        network_config,
        make_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    spawner.spawn(wifi_connection_task(wifi_controller).expect("allocate Wi-Fi connection task"));
    spawner.spawn(network_task(network_runner).expect("allocate network task"));
    spawner.spawn(dhcp::task(network, gateway_address).expect("allocate DHCP task"));

    let uart = Uart::new(
        peripherals.UART1,
        UartConfig::default().with_baudrate(115_200),
    )
    .expect("UART1 115200")
    .with_rx(peripherals.GPIO17)
    .with_tx(peripherals.GPIO16);
    let usb = UsbSerialJtag::new(peripherals.USB_DEVICE);

    network.wait_config_up().await;
    join(http_server(network), gateway_loop(uart, usb)).await;
    unreachable!()
}

#[allow(
    clippy::large_stack_frames,
    reason = "the HTTP future is stored in the esp-rtos main task, not on a thread stack"
)]
async fn http_server(network: Stack<'static>) -> ! {
    let app = Router::new()
        .route(
            "/",
            get_service(File::html(include_str!("../../web/index.html"))),
        )
        .route(
            "/api/health",
            get_service(File::with_content_type(
                "application/json",
                br#"{"status":"ok","service":"rubik-robot"}"#,
            )),
        )
        .route("/api/status", get(api_status))
        .route("/api/events", get(api_events))
        .route("/api/recover", post(api_recover))
        .route("/api/grip", post(api_grip))
        .route("/api/scan", post(api_scan))
        .route("/api/solve", post(api_solve))
        .route("/api/execute", post(api_execute))
        .route("/api/execute-moves", post(api_execute_moves))
        .route("/api/scan-solve-execute", post(api_scan_solve_execute))
        .route("/api/open", post(api_open))
        .route("/api/abort", post(api_abort));
    let server_config = picoserve::Config::const_default().close_connection_after_response();
    let mut tcp_rx_buffer_0 = [0; 2048];
    let mut tcp_tx_buffer_0 = [0; 4096];
    let mut http_buffer_0 = [0; 2048];
    let mut tcp_rx_buffer_1 = [0; 2048];
    let mut tcp_tx_buffer_1 = [0; 4096];
    let mut http_buffer_1 = [0; 2048];

    let (shutdown, _) = join(
        picoserve::Server::new(&app, &server_config, &mut http_buffer_0).listen_and_serve(
            0,
            network,
            app_config::HTTP_PORT,
            &mut tcp_rx_buffer_0,
            &mut tcp_tx_buffer_0,
        ),
        picoserve::Server::new(&app, &server_config, &mut http_buffer_1).listen_and_serve(
            1,
            network,
            app_config::HTTP_PORT,
            &mut tcp_rx_buffer_1,
            &mut tcp_tx_buffer_1,
        ),
    )
    .await;
    shutdown.into_never()
}

async fn api_events(websocket: WebSocketUpgrade) -> impl IntoResponse {
    websocket.on_upgrade(EventWebSocket)
}

#[allow(
    clippy::large_stack_frames,
    reason = "the HTTP request future is stored in picoserve task state"
)]
async fn api_status() -> Result<Json<link::StatusSnapshot>, (StatusCode, &'static str)> {
    let request_id = next_http_request_id();
    let request =
        RequestPacket::empty(link::RequestOpcode::GetStatus, request_id).map_err(|_| {
            (
                StatusCode::INTERNAL_SERVER_ERROR,
                "failed to encode status request",
            )
        })?;
    let response = send_http_request(request_id, request)
        .await
        .map_err(status_rpc_error)?;

    let packet = link::decode_packet(response.as_slice())
        .map_err(|_| (StatusCode::BAD_GATEWAY, "Duo returned an invalid packet"))?;
    if packet.kind != link::MessageKind::Response
        || packet.opcode != u16::from(link::ResponseOpcode::StatusSnapshot)
    {
        return Err((StatusCode::BAD_GATEWAY, "unexpected response from Duo"));
    }
    let snapshot: link::StatusSnapshot = link::decode_payload(packet.payload)
        .map_err(|_| (StatusCode::BAD_GATEWAY, "invalid status payload from Duo"))?;
    snapshot
        .validate()
        .map_err(|_| (StatusCode::BAD_GATEWAY, "invalid robot status from Duo"))?;
    Ok(Json(snapshot))
}

async fn api_recover() -> ApiCommandResponse {
    send_empty_command(link::RequestOpcode::RecoverToOpen).await
}

async fn api_grip() -> ApiCommandResponse {
    send_empty_command(link::RequestOpcode::Grip).await
}

async fn api_scan(Json(command): Json<link::StartScanCommand>) -> ApiCommandResponse {
    send_command(link::RequestOpcode::StartScan, &command).await
}

async fn api_solve(Json(command): Json<link::SolveCommand>) -> ApiCommandResponse {
    send_command(link::RequestOpcode::Solve, &command).await
}

#[allow(
    clippy::large_stack_frames,
    reason = "picoserve stores the HTTP handler future in static task storage"
)]
async fn api_execute(Json(command): Json<link::ExecuteCommand>) -> ApiCommandResponse {
    send_command(link::RequestOpcode::Execute, &command).await
}

#[allow(
    clippy::large_stack_frames,
    reason = "picoserve stores this HTTP handler future in static task storage"
)]
async fn api_execute_moves(Json(command): Json<execute_moves_json::Request>) -> ApiCommandResponse {
    let request_id = next_http_request_id();
    let request = match encode_execute_moves_request(request_id, command) {
        Ok(request) => request,
        Err(response) => return response,
    };
    send_command_request(request_id, request).await
}

fn encode_execute_moves_request(
    request_id: u32,
    command: execute_moves_json::Request,
) -> Result<RequestPacket, ApiCommandResponse> {
    let (moves, move_count) = match link::parse_singmaster(&command.sequence) {
        Ok(parsed) => parsed,
        Err(link::SingmasterError::Empty) => {
            return Err(api_command_error(
                StatusCode::BAD_REQUEST,
                "move sequence is empty",
            ));
        }
        Err(link::SingmasterError::TooManyMoves) => {
            return Err(api_command_error(
                StatusCode::BAD_REQUEST,
                "move sequence exceeds 32 moves",
            ));
        }
        Err(link::SingmasterError::InvalidToken { .. }) => {
            return Err(api_command_error(
                StatusCode::BAD_REQUEST,
                "invalid Singmaster move",
            ));
        }
    };
    RequestPacket::with_payload(
        link::RequestOpcode::ExecuteMoves,
        request_id,
        &link::ExecuteMovesCommand {
            session_id: command.session_id,
            moves,
            move_count,
        },
    )
    .map_err(|()| api_command_error(StatusCode::INTERNAL_SERVER_ERROR, "encode request"))
}

async fn api_scan_solve_execute(
    Json(command): Json<link::ScanSolveExecuteCommand>,
) -> ApiCommandResponse {
    send_command(link::RequestOpcode::ScanSolveExecute, &command).await
}

async fn api_open(Json(command): Json<link::OpenCommand>) -> ApiCommandResponse {
    send_command(link::RequestOpcode::Open, &command).await
}

async fn api_abort() -> ApiCommandResponse {
    send_empty_command(link::RequestOpcode::Abort).await
}

fn next_http_request_id() -> u32 {
    NEXT_HTTP_REQUEST_ID.fetch_add(1, Ordering::Relaxed) | 0x8000_0000
}

#[allow(
    clippy::large_stack_frames,
    reason = "picoserve stores the HTTP handler future in static task storage"
)]
async fn send_empty_command(opcode: link::RequestOpcode) -> ApiCommandResponse {
    let request_id = next_http_request_id();
    let request = match RequestPacket::empty(opcode, request_id) {
        Ok(request) => request,
        Err(()) => {
            return api_command_error(StatusCode::INTERNAL_SERVER_ERROR, "encode request");
        }
    };
    send_command_request(request_id, request).await
}

#[allow(
    clippy::large_stack_frames,
    reason = "picoserve stores the HTTP handler future in static task storage"
)]
async fn send_command<T: Serialize>(
    opcode: link::RequestOpcode,
    payload: &T,
) -> ApiCommandResponse {
    let request_id = next_http_request_id();
    let request = match RequestPacket::with_payload(opcode, request_id, payload) {
        Ok(request) => request,
        Err(()) => {
            return api_command_error(StatusCode::INTERNAL_SERVER_ERROR, "encode request");
        }
    };
    send_command_request(request_id, request).await
}

#[allow(
    clippy::large_stack_frames,
    reason = "picoserve stores the HTTP handler future in static task storage"
)]
async fn send_command_request(request_id: u32, request: RequestPacket) -> ApiCommandResponse {
    let response = match send_http_request(request_id, request).await {
        Ok(response) => response,
        Err(error) => return command_rpc_error(error),
    };
    let packet = match link::decode_packet(response.as_slice()) {
        Ok(packet) => packet,
        Err(_) => return api_command_error(StatusCode::BAD_GATEWAY, "invalid Duo packet"),
    };
    if packet.kind != link::MessageKind::Response {
        return api_command_error(StatusCode::BAD_GATEWAY, "unexpected Duo message");
    }

    match link::ResponseOpcode::try_from(packet.opcode) {
        Ok(link::ResponseOpcode::CommandAccepted) => {
            let accepted: link::CommandAccepted = match link::decode_payload(packet.payload) {
                Ok(accepted) => accepted,
                Err(_) => {
                    return api_command_error(
                        StatusCode::BAD_GATEWAY,
                        "invalid acceptance payload",
                    );
                }
            };
            ApiCommandResponse {
                status: StatusCode::ACCEPTED,
                body: ApiCommandBody::Accepted {
                    request_id,
                    operation_id: accepted.operation_id,
                },
            }
        }
        Ok(link::ResponseOpcode::CommandRejected) => {
            let rejected: link::CommandRejected = match link::decode_payload(packet.payload) {
                Ok(rejected) => rejected,
                Err(_) => {
                    return api_command_error(StatusCode::BAD_GATEWAY, "invalid rejection payload");
                }
            };
            ApiCommandResponse {
                status: StatusCode::CONFLICT,
                body: ApiCommandBody::Error {
                    error: "command rejected",
                    reason: Some(rejected.reason),
                    controller: Some(rejected.controller),
                },
            }
        }
        _ => api_command_error(StatusCode::BAD_GATEWAY, "unexpected Duo response"),
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "picoserve stores the HTTP handler future in static task storage"
)]
async fn send_http_request(
    request_id: u32,
    request: RequestPacket,
) -> Result<OwnedPacket, HttpRpcError> {
    let _rpc_guard = HTTP_RPC_LOCK.lock().await;
    HTTP_REQUESTS.send(request).await;
    let response = with_timeout(HTTP_RPC_TIMEOUT, async {
        loop {
            let response = HTTP_RESPONSES.receive().await;
            let packet = link::decode_packet(response.as_slice())
                .map_err(|_| HttpRpcError::InvalidPacket)?;
            if packet.request_id == request_id {
                break Ok(response);
            }
        }
    })
    .await;
    match response {
        Ok(response) => response,
        Err(_) => {
            HTTP_CANCELLATIONS.send(request_id).await;
            Err(HttpRpcError::Timeout)
        }
    }
}

fn status_rpc_error(error: HttpRpcError) -> (StatusCode, &'static str) {
    match error {
        HttpRpcError::Timeout => (StatusCode::GATEWAY_TIMEOUT, "Duo did not answer in time"),
        HttpRpcError::InvalidPacket => (StatusCode::BAD_GATEWAY, "Duo returned an invalid packet"),
    }
}

fn command_rpc_error(error: HttpRpcError) -> ApiCommandResponse {
    match error {
        HttpRpcError::Timeout => api_command_error(StatusCode::GATEWAY_TIMEOUT, "Duo timeout"),
        HttpRpcError::InvalidPacket => {
            api_command_error(StatusCode::BAD_GATEWAY, "invalid Duo packet")
        }
    }
}

fn api_command_error(status: StatusCode, error: &'static str) -> ApiCommandResponse {
    ApiCommandResponse {
        status,
        body: ApiCommandBody::Error {
            error,
            reason: None,
            controller: None,
        },
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the allocation-free gateway lives inside static esp-rtos task storage"
)]
async fn gateway_loop(
    mut uart: Uart<'static, esp_hal::Blocking>,
    mut usb: UsbSerialJtag<'static, esp_hal::Blocking>,
) -> ! {
    let mut gateway = Gateway::new();
    let mut io_buffer = [0u8; 64];
    let mut frame_buffer = [0u8; MAX_UART_FRAME_LEN];

    loop {
        if uart.read_ready()
            && let Ok(count) = uart.read(&mut io_buffer)
        {
            for &byte in &io_buffer[..count] {
                let _ = gateway.push_duo_byte(byte);
            }
        }

        let mut count = 0;
        while count < io_buffer.len() {
            match usb.read_byte() {
                Ok(byte) => {
                    io_buffer[count] = byte;
                    count += 1;
                }
                Err(_) => break,
            }
        }
        for &byte in &io_buffer[..count] {
            let _ = gateway.push_usb_byte(byte);
        }

        while let Ok(request_id) = HTTP_CANCELLATIONS.try_receive() {
            gateway.cancel_network_request(request_id);
        }
        while let Ok(packet) = HTTP_REQUESTS.try_receive() {
            let _ = gateway.push_network_packet(packet.as_slice());
        }

        let now_ms = esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_millis();
        while let Ok(Some(frame_len)) = gateway.dequeue_duo_uart_frame(now_ms, &mut frame_buffer) {
            let mut sent = 0;
            while sent < frame_len {
                match uart.write(&frame_buffer[sent..frame_len]) {
                    Ok(0) | Err(_) => break,
                    Ok(count) => sent += count,
                }
            }
            if sent != frame_len {
                break;
            }
        }

        while let Ok(Some(frame_len)) = gateway.dequeue_usb_frame(&mut frame_buffer) {
            if usb.write(&frame_buffer[..frame_len]).is_err() || usb.flush_tx().is_err() {
                break;
            }
        }

        while !HTTP_RESPONSES.is_full() {
            let Ok(Some(packet_len)) = gateway.dequeue_network_packet(&mut frame_buffer) else {
                break;
            };

            let Ok(decoded) = link::decode_packet(&frame_buffer[..packet_len]) else {
                continue;
            };
            if decoded.kind == link::MessageKind::Event {
                if let Some(event) = api_event(decoded.opcode) {
                    HTTP_EVENT.signal(event);
                }
                continue;
            }
            let mut packet = OwnedPacket {
                bytes: [0; MAX_PACKET_LEN],
                len: packet_len,
            };
            packet.bytes[..packet_len].copy_from_slice(&frame_buffer[..packet_len]);
            let _ = HTTP_RESPONSES.try_send(packet);
        }

        Timer::after_millis(1).await;
    }
}

fn api_event(opcode: u16) -> Option<ApiEvent> {
    let event = match link::EventOpcode::try_from(opcode).ok()? {
        link::EventOpcode::RobotStateChanged => "robot_state_changed",
        link::EventOpcode::StandStateChanged => "stand_state_changed",
        link::EventOpcode::FaceScanned => "face_scanned",
        link::EventOpcode::PlanChanged => "plan_changed",
        link::EventOpcode::ActionStarted => "action_started",
        link::EventOpcode::ActionCompleted => "action_completed",
        link::EventOpcode::OperationCompleted => "operation_completed",
        link::EventOpcode::Aborted => "aborted",
        link::EventOpcode::CubeSessionChanged => "cube_session_changed",
        link::EventOpcode::OperationFailed => "operation_failed",
        link::EventOpcode::Fault => "fault",
    };
    Some(ApiEvent {
        sequence: NEXT_HTTP_EVENT_SEQUENCE.fetch_add(1, Ordering::Relaxed),
        event,
    })
}

#[embassy_executor::task]
async fn network_task(mut runner: Runner<'static, Interface<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn wifi_connection_task(controller: WifiController<'static>) -> ! {
    loop {
        let _ = controller
            .wait_for_access_point_connected_event_async()
            .await;
        Timer::after_secs(1).await;
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the Embassy macro places this async state machine in static task storage"
)]
mod dhcp {
    use super::*;

    #[embassy_executor::task]
    pub async fn task(network: Stack<'static>, gateway_address: Ipv4Addr) -> ! {
        use core::net::SocketAddrV4;

        use edge_dhcp::{
            io::{self, DEFAULT_SERVER_PORT},
            server::{Server, ServerOptions},
        };
        use edge_nal::UdpBind;
        use edge_nal_embassy::{Udp, UdpBuffers};

        let packet_buffer = make_static!([u8; 1500], [0u8; 1500]);
        let gateway_buffer = make_static!([Ipv4Addr; 1], [Ipv4Addr::UNSPECIFIED]);
        let buffers = make_static!(
            UdpBuffers<3, 1024, 1024, 10>,
            UdpBuffers::<3, 1024, 1024, 10>::new()
        );
        let socket = Udp::new(network, buffers);
        let mut socket = socket
            .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
                Ipv4Addr::UNSPECIFIED,
                DEFAULT_SERVER_PORT,
            )))
            .await
            .expect("bind DHCP server");

        loop {
            let _ = io::server::run(
                &mut Server::<_, 64>::new_with_et(gateway_address),
                &ServerOptions::new(gateway_address, Some(&mut *gateway_buffer)),
                &mut socket,
                &mut *packet_buffer,
            )
            .await;
            Timer::after(Duration::from_millis(500)).await;
        }
    }
}
