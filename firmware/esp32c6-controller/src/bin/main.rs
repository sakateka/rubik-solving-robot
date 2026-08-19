#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use core::net::Ipv4Addr;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Timer};
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
    response::File,
    routing::{Router, get_service},
};
use rubik_link_gateway::Gateway;
use rubik_link_protocol::MAX_UART_FRAME_LEN;
use static_cell::StaticCell;

mod app_config {
    include!(concat!(env!("OUT_DIR"), "/app_config.rs"));
}

esp_bootloader_esp_idf::esp_app_desc!();

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
        );
    let server_config = picoserve::Config::const_default().keep_connection_alive();
    let mut tcp_rx_buffer = [0; 2048];
    let mut tcp_tx_buffer = [0; 4096];
    let mut http_buffer = [0; 2048];

    picoserve::Server::new(&app, &server_config, &mut http_buffer)
        .listen_and_serve(
            0,
            network,
            app_config::HTTP_PORT,
            &mut tcp_rx_buffer,
            &mut tcp_tx_buffer,
        )
        .await
        .into_never()
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

        Timer::after_millis(1).await;
    }
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
