#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::Instant;
use esp_hal::uart::{Config, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;
use rubik_link_gateway::Gateway;
use rubik_link_protocol::MAX_UART_FRAME_LEN;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32c6 -o unstable-hal -o log -o esp-backtrace

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // XIAO ESP32-C6: D7/GPIO17 = UART1 RX (Duo TX), D6/GPIO16 = UART1 TX (Duo RX).
    let mut uart = Uart::new(peripherals.UART1, Config::default().with_baudrate(115_200))
        .expect("UART1 115200")
        .with_rx(peripherals.GPIO17)
        .with_tx(peripherals.GPIO16);
    let mut usb = UsbSerialJtag::new(peripherals.USB_DEVICE);

    static GATEWAY: StaticCell<Gateway> = StaticCell::new();
    let gateway = GATEWAY.init_with(Gateway::new);
    let mut io_buf = [0u8; 64];
    let mut frame_buf = [0u8; MAX_UART_FRAME_LEN];
    loop {
        if uart.read_ready()
            && let Ok(count) = uart.read(&mut io_buf)
        {
            for &byte in &io_buf[..count] {
                let _ = gateway.push_duo_byte(byte);
            }
        }

        let mut count = 0;
        while count < io_buf.len() {
            match usb.read_byte() {
                Ok(b) => {
                    io_buf[count] = b;
                    count += 1;
                }
                Err(_) => break,
            }
        }
        for &byte in &io_buf[..count] {
            let _ = gateway.push_usb_byte(byte);
        }

        let now_ms = Instant::now().duration_since_epoch().as_millis();
        while let Ok(Some(frame_len)) = gateway.dequeue_duo_uart_frame(now_ms, &mut frame_buf) {
            let mut sent = 0;
            while sent < frame_len {
                match uart.write(&frame_buf[sent..frame_len]) {
                    Ok(0) | Err(_) => break,
                    Ok(k) => sent += k,
                }
            }
            if sent != frame_len {
                break;
            }
        }

        while let Ok(Some(frame_len)) = gateway.dequeue_usb_frame(&mut frame_buf) {
            if usb.write(&frame_buf[..frame_len]).is_err() {
                break;
            }
            if usb.flush_tx().is_err() {
                break;
            }
        }
    }
}
