# ESP32-C6 Rust firmware handoff

This note is for an agent working on the other host, where the ESP32-C6 is
physically attached. It contains the established hardware facts and the first
firmware milestone. Do not replace the C6 firmware with Arduino or ESP-IDF C:
the project uses Rust on both the Milk-V Duo and the C6.

## Board and wiring

The board is a Seeed Studio XIAO ESP32-C6. Its USB device is already detected
by the development host as Espressif USB JTAG/serial debug unit at
`/dev/ttyACM0`.

The UART wiring is crossed:

| Milk-V Duo | XIAO ESP32-C6 | Meaning |
|---|---|---|
| `GP0` / `UART1_TX` | `D7` / `GPIO17` / `RX` | Duo → C6 |
| `GP1` / `UART1_RX` | `D6` / `GPIO16` / `TX` | C6 → Duo |
| `GND` | `GND` | common reference |

The C6 is powered by its own USB connection during development. Do not add a
power wire between Duo and C6 for this stage.

On Duo, `GP0` and `GP1` were already observed in the `UART1_TX` and
`UART1_RX` pinmux functions. The expected Linux node is `/dev/ttyS1`; verify
it before testing with `ls -l /dev/ttyS*`.

## Goal of this milestone

Create a small, standalone Rust firmware crate at
`firmware/esp32c6-controller/`. It must use `esp-hal` in `no_std` mode and
provide a transparent 115200 baud UART bridge:

```text
USB Serial/JTAG on development host <-> C6 UART1 (D7 RX, D6 TX) <-> Duo UART1
```

The bridge is deliberately the first step. Do not implement BLE, Wi-Fi,
servo commands, or a final command protocol yet. Its purpose is to establish
the physical link and a Rust firmware build/flash workflow.

## Rust toolchain setup on the development host

Install the ESP Rust tools in the user's Cargo bin directory:

```sh
cargo install espup espflash cargo-generate
```

Install the Espressif Rust toolchain and load it in the current shell:

```sh
espup install
```

```sh
. "$HOME/export-esp.sh"
```

Use the current `esp-generate` scaffold; `esp-template` is no longer updated.
The established crate uses `esp-hal` 1.1 in `no_std` mode, without Embassy.
Do not copy old `esp-hal` APIs from random examples: use the dependency
versions and generated configuration from the current scaffold, then consult
the matching `esp-hal` documentation when wiring UART and USB Serial/JTAG.

The firmware should be built and flashed with `espflash`, for example from the
crate directory:

```sh
espflash flash --monitor /dev/ttyACM0
```

If `/dev/ttyACM0` access is denied, add the user to `dialout` and start a new
login session:

```sh
sudo usermod -aG dialout "$USER"
```

If automatic flashing cannot reset the board, hold `BOOT`, press and release
`RESET`, then release `BOOT` before retrying the flash command.

## Required observable behaviour

At boot, write a clear `rubik-c6 ready` line to the USB monitor. Then forward
all received bytes in both directions. Keep the implementation blocking and
small; no async executor is needed for this first hardware probe. This
milestone has now been physically validated: `hello from duo` was observed in
the C6 USB monitor and text entered through C6 was observed by
`cat /dev/ttyS1` on Duo.

Verify both directions after flashing:

1. On Duo, configure the UART:

   ```sh
   stty -F /dev/ttyS1 115200 raw -echo -ixon -ixoff
   ```

2. Duo → C6: run the following on Duo and observe the string in the C6 USB
   monitor:

   ```sh
   printf 'hello from duo\n' > /dev/ttyS1
   ```

3. C6 → Duo: leave this running on Duo and type a line in the C6 USB monitor:

   ```sh
   cat /dev/ttyS1
   ```

The UART byte stream itself has no framing yet. Once this is confirmed, the
next project step is a small framed Rust protocol (`status`, `scan`, `solve`,
`abort`) between C6 and Duo.

## Scope and documentation

- Keep the C6 crate independent from the Linux/Duo root Cargo package; it is
  a nested Cargo project.
- Do not modify hardware calibration or stand choreography while bringing up
  the link.
- After a physical two-way bridge test succeeds, append the exact flash and
  validation commands to `PROJECT_NOTES.md`.
- Do not create a git commit unless the user explicitly asks for one.

References: [XIAO ESP32-C6 pin map](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/), [ESP-RS template](https://github.com/esp-rs/esp-template).
