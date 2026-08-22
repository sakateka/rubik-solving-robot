# Rubik Robot

An end-to-end software stack for an autonomous Rubik's Cube robot built around
the Milk-V Duo 256M (SG2002). The project scans all six faces with a GC2083
camera and the CV181X TPU, validates the cube state, solves it with min2phase,
and executes the solution on an eight-servo gripper stand.

The repository also contains a server-authoritative 3D simulator. It runs the
same robot service and motion planner without hardware, visualizes the cube and
grippers in a browser, and checks motion sequencing before changes reach the
physical mechanism.

## What is implemented

- GC2083 camera capture through the vendor VI/ISP/VPSS stack.
- YOLO sticker detection on the CV181X TPU and conversion of six scans into a
  validated `URFDLB` cube state.
- min2phase solving and parsing of Singmaster moves.
- Collision-aware planning for four rail servos and four gripper servos, with
  optimized execution across compatible moves.
- A stateful robot daemon, framed serial protocol, host CLI, and ESP32-C6
  gateway/firmware components.
- A browser-based 3D operator UI with scan/solve/execute, manual moves,
  scramble loading, animation controls, and per-tab server-side sessions.
- Rust unit tests and Playwright safety/animation tests, including all 18 face
  turns and repeated scan/solve workflows.

The simulator is an important pre-deployment check, but it does not replace
inspection and cautious validation on the real stand: the current servos have
no position or force feedback.

## Run the simulator

Install a current Rust toolchain, clone the repository, and run:

```bash
cargo run --features pca9685 --bin rubik-robotd-sim -- \
  --addr 127.0.0.1:8022
```

Open <http://127.0.0.1:8022>. Each browser tab gets an independent robot,
cube, solution, and animation-speed session, so manual work is isolated from
other tabs and automated tests. Reloading a tab restores that tab's server-side
snapshot. After rebuilding and restarting the server, connected tabs reload
the embedded UI automatically.

The normal simulator workflow is:

```text
Recover open -> Grip -> Load or Run moves -> Auto scan / solve / execute
```

`Grip` deliberately does not perform recovery automatically. A cube may be
lying between the grippers, so rotating or closing from an unknown mechanical
state would be unsafe.

## Test locally

Run the Rust workspace tests:

```bash
cargo test --features pca9685 --workspace
```

The Playwright suite starts its own simulator on port `18123`; it never reuses
the operator server on port `8022`:

```bash
cd tests/ui
npm install
npx playwright install chromium
npm test
```

## Inspect a move plan without hardware

The planner CLI compares the conservative and optimized mechanical plans and
prints their actions, servo targets, and estimated duration:

```bash
cargo run --bin rubik-move-plan -- "R U R' U' F2" --open-after
```

## Connect to the robot over Wi-Fi

The ESP32-C6 controller creates its own WPA2 access point with DHCP, so the
robot does not need a home network or an Internet connection. With the Duo
daemon and C6 firmware running:

1. Connect a phone or laptop to the configured robot SSID.
2. Open <http://192.168.4.1/>.
3. Use the embedded operator UI to inspect status and run robot operations.

The default bring-up configuration uses SSID `Rubik Robot` and password
`ChangeMe`. **Replace that password before normal use** by creating
`firmware/esp32c6-controller/config/local.toml` as described in the
[ESP32-C6 firmware README](firmware/esp32c6-controller/README.md). Wi-Fi
settings are compiled into the firmware, so changing them requires rebuilding
and flashing the C6.

The browser talks HTTP and WebSocket to the C6. The C6 forwards protocol
requests over UART to `rubik-robotd` on the Milk-V Duo; the Duo remains the
authoritative owner of robot state, safety checks, scanning, solving, and
motion planning. The same interface is also available as an HTTP API; see the
[control protocol](docs/ROBOT_CONTROL_PROTOCOL.md#wi-fi-and-http-transport).

## Hardware build

The Milk-V Duo Buildroot SDK is a pinned Git submodule. Clone it together with
the project:

```bash
git clone --recurse-submodules <repository-url>
```

For an existing checkout:

```bash
git submodule update --init --recursive
```

Do not update the SDK casually. Its headers, runtime libraries, musl ABI, and
cross-toolchain must remain compatible with the firmware on the Duo.

After building the SDK prerequisites described in
[`docs/PROJECT_NOTES.md`](docs/PROJECT_NOTES.md), cross-compile the complete
Duo binary set with:

```bash
./scripts/build-duo.sh
```

The production daemon requires an explicit acknowledgement before it can move
the stand:

```bash
rubik-robotd --confirm-stand-motion
```

The physical operator button uses two wires between Duo256M `GP21` (Linux GPIO
`506`) and GND:

```text
GP21 ---- button ---- GND
```

At startup the daemon enables and reads back GP21's internal weak pull-up, then
opens the active-low input with 50 ms debounce. No external resistor is needed.
A press while open runs `Grip` followed by automatic scan/solve/execute. A
press from any other idle pose runs collision-safe recovery to open. During an
active operation it performs priority `Abort` followed by recovery; recovery
is not started if Abort fails and the controller enters `Faulted`. Use
`--no-button` only for diagnostics. A custom `--button-gpio` must provide its
own electrically stable released level.

Use the calibration in [`config/stand.toml`](config/stand.toml) as the starting
point for the physical stand. Read the recovery, grip, abort, and session
invariants in
[`docs/ROBOT_OPERATION_WORKFLOWS.md`](docs/ROBOT_OPERATION_WORKFLOWS.md)
before enabling servo power.

## Architecture

```text
GC2083 -> VI/ISP/VPSS -> YOLO on CV181X TPU -> six canonical faces
                                                     |
                                                     v
                           validated cube state -> min2phase solution
                                                     |
                                                     v
                robot service -> motion planner -> PCA9685 -> 8 servos
                       |
                       +-> serial protocol -> ESP32-C6 / host client
                       +-> simulated stand -> HTTP/SSE -> browser 3D UI
```

The Duo owns authoritative robot state and motion planning. Operations are
bound to cube-session, scan-revision, solution, and operation IDs so stale or
duplicated commands cannot execute against a different physical state. On
startup the mechanism is `Unknown`; collision-safe recovery opens both rail
pairs before rotating grippers to their safe perpendicular pose.

## Repository guide

- `src/cube.rs` — face orientation, cube validation, and solver integration.
- `src/move_planner.rs` — optimized collision-aware mechanical plans.
- `src/robot_service.rs` — authoritative command and operation state machine.
- `src/stand*.rs`, `src/pca9685.rs` — stand execution and servo control.
- `src/camera.rs`, `src/tpu.rs`, `src/vision_scanner.rs` — camera-to-face
  vision pipeline.
- `src/sim_server.rs`, `web/` — isolated native simulator and embedded 3D UI.
- `crates/rubik-link-*`, `firmware/` — transport protocol, gateway, and
  ESP32-C6 controller.
- `tests/ui/` — Playwright workflow, isolation, safety, and animation tests.

## Documentation

- [`docs/PROJECT_NOTES.md`](docs/PROJECT_NOTES.md) — engineering journal,
  reproducible camera/TPU commands, deployment notes, and current simulator
  behavior.
- [`docs/ROBOT_OPERATION_WORKFLOWS.md`](docs/ROBOT_OPERATION_WORKFLOWS.md) —
  safety and lifecycle contract for physical operations.
- [`docs/ROBOT_CONTROL_PROTOCOL.md`](docs/ROBOT_CONTROL_PROTOCOL.md) — daemon
  responsibilities, commands, events, and the Wi-Fi HTTP/WebSocket transport.
- [`docs/planner-optimization.md`](docs/planner-optimization.md) — planner
  optimization design and measured Cube20 results.
- [`docs/wasm-simulator.md`](docs/wasm-simulator.md) — plan for a Web Worker /
  WASM simulator deployable to GitHub Pages.
- [`models/README.md`](models/README.md) — released PyTorch, ONNX, and CV181X
  BF16 model artifacts.
- [`docs/CONTRIBUTING.md`](docs/CONTRIBUTING.md) — contribution and FFI safety
  rules.
