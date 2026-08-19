# ESP32-C6 robot link gateway

This firmware runs on the Seeed XIAO ESP32-C6 between the development host and
the Milk-V Duo:

```text
rubik-robotctl
    ↕ USB Serial/JTAG, COBS frames
ESP32-C6 protocol gateway
    ↕ UART1, 115200 baud, COBS frames
Milk-V Duo rubik-robotd[-sim]
```

The current milestone provides a protocol-aware USB ↔ UART gateway. It validates
COBS framing, packet length, protocol version and CRC before forwarding. Normal
requests use a bounded FIFO. `Abort` has a separate priority queue and is retried
every 100 ms until Duo returns a response with the same request ID.

No text is written to USB after boot. USB Serial/JTAG is the binary protocol
stream, so status messages such as `rubik-c6 ready` would corrupt framing.

## Wiring

| Duo | ESP32-C6 | Direction |
|---|---|---|
| GP0 / UART1_TX | D7 / GPIO17 / RX | Duo → C6 |
| GP1 / UART1_RX | D6 / GPIO16 / TX | C6 → Duo |
| GND | GND | common reference |

## Build and flash

From this directory on the host connected to C6:

```sh
cargo check
cargo run --release
```

The Cargo runner invokes `espflash` for `/dev/ttyACM0` and exits after flashing.
It intentionally does not start a serial monitor: `rubik-robotctl` must own that
device, and the stream contains binary frames rather than logs.

Equivalent explicit flashing command:

```sh
espflash flash --chip esp32c6 --port /dev/ttyACM0 target/riscv32imac-unknown-none-elf/release/esp32c6-controller
```

## Quiet end-to-end check

Run `rubik-robotd-sim` on Duo, then execute on the C6 development host:

```sh
rubik-robotctl status
rubik-robotctl --confirm-stand-motion recover
rubik-robotctl --confirm-stand-motion grip
rubik-robotctl abort
```

The simulator does not open I²C and cannot move servos, while all link framing,
request caching, deadlines and protocol events remain real.

## Next milestone

The gateway core already accepts and emits transport-neutral BLE packets through
`push_ble_packet` and `dequeue_ble_packet`. The next firmware step is to connect
those methods to BLE GATT characteristics and add ATT fragmentation. USB remains
the development transport.
