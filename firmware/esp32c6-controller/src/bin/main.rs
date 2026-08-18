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
use esp_hal::uart::{Config, Uart};
use esp_hal::usb_serial_jtag::UsbSerialJtag;

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

    usb.write(b"rubik-c6 ready\r\n").ok();
    usb.flush_tx().ok();

    let mut buf = [0u8; 64];
    loop {
        if uart.read_ready() {
            if let Ok(n) = uart.read(&mut buf) {
                usb.write(&buf[..n]).ok();
                usb.flush_tx().ok();
            }
        }

        let mut n = 0;
        while n < buf.len() {
            match usb.read_byte() {
                Ok(b) => {
                    buf[n] = b;
                    n += 1;
                }
                Err(_) => break,
            }
        }
        if n > 0 {
            let mut sent = 0;
            while sent < n {
                match uart.write(&buf[sent..n]) {
                    Ok(0) | Err(_) => break,
                    Ok(k) => sent += k,
                }
            }
        }
    }
}
