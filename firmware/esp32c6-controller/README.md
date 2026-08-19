# ESP32-C6 robot network controller

This firmware runs on the Seeed XIAO ESP32-C6 between browser/development
clients and the Milk-V Duo:

```text
Browser                     rubik-robotctl
    ↕ Wi-Fi, HTTP/WebSocket     ↕ USB Serial/JTAG, COBS frames
ESP32-C6 network controller and protocol gateway
    ↕ UART1, 115200 baud, COBS frames
Milk-V Duo rubik-robotd[-sim]
```

The firmware provides two upstream transports:

- a WPA2 Wi-Fi access point with DHCP and an embedded HTTP UI;
- USB Serial/JTAG for the existing `rubik-robotctl` development client.

The protocol-aware gateway validates COBS framing, packet length, protocol
version and CRC before forwarding. Normal requests use a bounded FIFO. `Abort`
has a separate priority queue and is retried every 100 ms until Duo returns a
response with the same request ID.

No text is written to USB after boot. USB Serial/JTAG is the binary protocol
stream, so status messages such as `rubik-c6 ready` would corrupt framing.

## Wiring

| Duo | ESP32-C6 | Direction |
|---|---|---|
| GP0 / UART1_TX | D7 / GPIO17 / RX | Duo → C6 |
| GP1 / UART1_RX | D6 / GPIO16 / TX | C6 → Duo |
| GND | GND | common reference |

## Build and flash

### Local configuration

Committed defaults live in `config/default.toml`. They intentionally use the
bring-up password `ChangeMe`. Create an ignored partial override for the actual
robot:

```sh
cp config/default.toml config/local.toml
```

Then edit `config/local.toml`. Every field is optional in the local file, so a
minimal override is sufficient:

```toml
[wifi]
password = "replace-with-a-private-password"
```

The effective configuration is generated at compile time. WPA2 passwords must
contain 8 to 63 bytes. Changing the configuration currently requires rebuilding
and flashing the firmware.

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

## Wi-Fi smoke test

Connect to the configured SSID and open:

```text
http://192.168.4.1/
```

The page polls `GET /api/status` and renders the complete JSON snapshot returned
by the Duo daemon. The default SSID is `Rubik Robot`; the default password is
`ChangeMe`.

The same endpoint can be checked without the UI:

```sh
curl http://192.168.4.1/api/status
```

This is a real request across the complete path
HTTP → C6 gateway → UART → Duo daemon. If Duo does not answer within three
seconds, C6 returns HTTP `504 Gateway Timeout`. `GET /api/health` only verifies
the C6 HTTP server and does not contact Duo.

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

Add command endpoints and a WebSocket event stream. Robot decisions and state
remain on the Duo; C6 only adapts HTTP/WebSocket messages to the existing link
protocol.
