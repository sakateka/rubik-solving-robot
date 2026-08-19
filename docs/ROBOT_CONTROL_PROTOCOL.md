# Robot control architecture and link protocol

This document defines communication between the browser UI, the ESP32-C6
network controller, and the Milk-V Duo robot controller.

The behavioural command sequences and cube-session rules are specified in
`ROBOT_OPERATION_WORKFLOWS.md`.

## Responsibility boundaries

```text
Browser
        │ HTTP + WebSocket over Wi-Fi
        ▼
ESP32-C6
  WPA2 access point, web server and protocol gateway
        │ UART1, 115200 baud
        ▼
Milk-V Duo
  robot daemon
        ├── stand runtime / PCA9685
        ├── camera and TPU inference
        ├── facelet construction
        └── min2phase and move execution
```

The Milk-V Duo is the single source of truth. It owns the commanded mechanical
state, cube scan, solver input and output, and the mechanical action queue.

The ESP32-C6 owns Wi-Fi connectivity, embedded HTML/CSS/JavaScript assets, the
HTTP/WebSocket API, UART framing and link status. It does not decide how to
move the cube. `Abort` is the only command that receives transport priority:
C6 places it ahead of queued traffic and retries it until Duo acknowledges it.

The browser presents state and sends semantic commands. It must never send
PCA9685 channel values or servo pulse widths.

## Duo robot daemon

The production process will be a long-running daemon, provisionally named
`rubik-robotd`. Existing interactive binaries remain useful as diagnostics,
but a blocking `scan` or `execute` function cannot provide immediate abort,
live status, or action progress.

The daemon therefore executes one bounded mechanical action at a time:

```text
receive UART frames
handle Abort before normal commands
advance the active operation by one step
publish resulting state changes and events
repeat
```

Servo delays are represented as deadlines. The daemon must continue handling
UART while it waits for a rail or gripper movement to finish; it must not sleep
inside the complete scan or solve-execute flow.

### Orthogonal state model

Robot state is not one linear enum. For example, after a normal scan the stand
is open while a valid facelet remains available. A status snapshot therefore
contains independent components:

```text
controller       Booting | Ready | Busy | Aborted | Faulted
stand            commanded state of every rail and gripper
scan             None | InProgress | Valid | Invalid
solution         None | Solving | Ready | Executing | Completed
active_operation optional operation ID, kind, and action progress
```

This avoids a cross-product such as `OpenWithValidScanAndPreparedSolution`.
Command admission is decided using all relevant components. For example,
`Grip` requires a known open stand and a ready controller; `Execute` requires a
prepared solution and a mechanically valid starting pose.

### Commanded mechanical state

The servos have no position feedback. Consequently, the protocol reports
**commanded state**, not measured physical state.

Each rail and gripper is represented as one of:

```text
Unknown
Stable(value)
Moving(from?, target)
```

Rail values are `Open` and `Grip`. Gripper values are `FrameParallel`,
`FramePerpendicular`, and `FrameParallelReversed`.

If an operation is aborted during movement, the moving axes become `Unknown`.
Normal operation cannot continue from that state. A separate recovery command
must move the inspected stand to a known open state.

## Public commands

The initial command set is:

| Command | Meaning |
|---|---|
| `GetStatus` | Return a complete current snapshot. |
| `Grip` | Capture a cube from a known open stand. |
| `StartScan` | Scan all six faces and validate the facelet. |
| `Solve` | Solve the last validated facelet without moving the stand. |
| `Execute` | Execute the already prepared solver sequence. |
| `ScanSolveExecute` | Scan, solve, and execute without opening between scan and execution; open after completion. |
| `Open` | Perform the normal orientation-preserving release sequence. |
| `Abort` | Immediately cancel the operation and disable every PWM channel. |
| `RecoverToOpen` | Re-establish a known open state after operator inspection of an abort or fault. |
| `ExecuteMoves` | Execute a bounded manual Singmaster sequence and invalidate previous scan/solution data. |

`StartScan`, `Solve`, `Execute`, `ScanSolveExecute`, `Open`, and `ExecuteMoves`
carry the cube session they expect. `Solve` additionally carries a scan
revision; `Execute` carries the session, scan revision, and solution ID. This
prevents a delayed client request from moving a different cube.

Daemon `StartScan` returns to canonical grip and keeps the cube held. It does
not inherit the open-after-scan behaviour of the diagnostic CLI.

Long-running commands have two distinct results:

```text
CommandAccepted(request_id, operation_id)
OperationCompleted(operation_id)
```

An invalid command receives `CommandRejected(request_id, reason)`. Acceptance
only means that the operation was admitted; it does not mean it has completed.

## Status and events

A full status snapshot contains:

- controller state and active operation ID;
- optional cube session ID; its presence means the robot has commanded custody
  of one uninterrupted physical cube;
- logical stand pose (`Unknown`, `Open`, canonical grip, scan pose, move pose,
  or transitional);
- all commanded rail and gripper positions;
- whether PWM outputs are currently enabled;
- camera-facing logical face, when known;
- latest recognized 3×3 physical color matrix;
- mask and matrices of already scanned faces;
- color counts and facelet validation status;
- logical solver moves;
- current mechanical action and a bounded preview of queued actions;
- current fault, if any.

Incremental events avoid repeatedly transmitting the entire snapshot:

- `RobotStateChanged`;
- `StandStateChanged`;
- `FaceScanned`;
- `PlanChanged`;
- `ActionStarted` and `ActionCompleted`;
- `OperationCompleted`;
- `CubeSessionChanged`;
- recoverable `OperationFailed`;
- `Aborted`;
- `Fault`.

`GetStatus` remains available after reconnect and restores the complete client
view even if notifications were lost. The C6 web adapter uses events as
invalidation signals rather than maintaining a second robot-state reducer: it
coalesces event bursts, retains the latest notification and fetches one complete
snapshot from Duo.

Responses remain point-to-point and return only to the upstream that issued the
request. Events are observable by the web adapter regardless of whether the
operation was started from HTTP or development USB; a USB operation owner also
receives its event stream through `rubik-robotctl`.

Sticker confidence is transmitted as an integer from 0 to 255. This is enough
for the web UI to mark uncertain recognition without exposing model-specific
floating-point details.

Mechanical actions are semantic, for example `SetRail`, `SetGripper`, `Wait`,
`CaptureFace`, and `RecognizeFace`. Raw PWM values are diagnostic information
and are not part of the normal HTTP API.

## Transport-neutral packet

All multi-byte integers use little-endian byte order. Protocol version 1 uses:

```text
offset  size  field
0       1     protocol version
1       1     message kind: request, response, or event
2       2     explicit opcode
4       4     request ID; zero for unsolicited events
8       2     payload length
10      N     payload
10+N    2     CRC-16/CCITT-FALSE over header and payload
```

The payload is limited to 1024 bytes in version 1. Payload structures use
`postcard` 1.1, whose wire format is stable and designed for `no_std`. C-like
wire enums use explicit integer representations through `serde_repr`; their
encoded values do not depend on Rust declaration order. Struct field order is
part of the schema and is protected by golden encoding tests.

Version 1 reserves these top-level opcode ranges:

| Kind | Range | Current opcodes |
|---|---:|---|
| request | `0x0000..0x0fff` | `GetStatus=0x0001`, normal operations `0x0010..0x0017`, `Abort=0x00ff` |
| response | `0x1000..0x1fff` | accepted, rejected, status snapshot |
| event | `0x2000..0x2fff` | state, stand, face, plan, action, completion, abort, fault |

The solution buffer is bounded at 32 logical moves and the status snapshot
contains at most 16 preview actions. Count fields are validated after
deserialization; CRC-valid input is not automatically schema-valid.

Request IDs are allocated by the upstream adapter. Development USB requests
from `rubik-robotctl` use `0x00000001..0x7fffffff`; C6 HTTP requests use
`0x80000000..0xffffffff`. The disjoint ranges let both transports have requests
in flight without an ID collision. Duo keeps a bounded cache of recent request
results. Receiving the same ID again returns the previous result and must not
start a second scan or execute a move twice.

## UART framing: COBS

COBS means **Consistent Overhead Byte Stuffing**. It transforms arbitrary bytes
into a representation that contains no zero bytes. A zero byte can therefore
be used as an unambiguous frame delimiter:

```text
COBS(packet) 0x00 COBS(packet) 0x00 ...
```

The receiver accumulates bytes until `0x00`, decodes one COBS frame, validates
its CRC and lengths, then dispatches it. A truncated or corrupted frame is
discarded. The next zero delimiter restores framing without relying on UART
timing gaps.

The CRC detects corruption of an otherwise structurally valid frame. COBS
provides framing and resynchronization; it does not provide integrity.

## Wi-Fi and HTTP transport

C6 starts a WPA2-protected access point and serves the UI directly. The first
deployment uses a static gateway address and DHCP, so the robot remains
reachable without a home network or Internet connection:

```text
SSID: configured at build time
URL:  http://192.168.4.1/
```

HTTP maps semantic operations onto the same transport-neutral requests used by
the USB development client. A WebSocket carries state and progress events to
the browser. The initial API surface is:

| Method | Path | Meaning |
|---|---|---|
| `GET` | `/api/status` | Complete current snapshot. |
| `POST` | `/api/recover` | Recover to a known open stand. |
| `POST` | `/api/grip` | Grip a cube. |
| `POST` | `/api/scan` | Scan all faces. |
| `POST` | `/api/solve` | Solve the validated facelet. |
| `POST` | `/api/execute` | Execute the prepared solution. |
| `POST` | `/api/scan-solve-execute` | Run the complete flow. |
| `POST` | `/api/open` | Release the cube preserving orientation. |
| `POST` | `/api/abort` | Priority software stop. |
| `GET` | `/api/events` | WebSocket event stream. |

The WebSocket emits compact JSON notifications such as
`{"sequence":17,"event":"face_scanned"}`. `sequence` is local to the current
C6 boot and is diagnostic; it is not a robot operation or protocol request ID.
The browser refreshes `/api/status` after each notification and after every
WebSocket reconnect.

`recover`, `grip`, and `abort` have an empty request body. Session-bound
commands use the same field names as the UART protocol payloads:

```json
{"session_id": 1}
{"session_id": 1, "scan_revision": 2}
{"session_id": 1, "scan_revision": 2, "solution_id": 3}
```

An admitted command returns HTTP `202` with its protocol request and operation
IDs. A command rejected by Duo returns HTTP `409`; malformed JSON is HTTP `400`,
and an unavailable Duo is HTTP `504`. C6 does not reinterpret Duo admission
rules.

The firmware configuration is a compile-time overlay. A committed default
contains non-secret bring-up values; an ignored `config/local.toml` overrides
the SSID, WPA2 password, channel, address, or port for a particular robot.
Configuration validation rejects invalid WPA2 password lengths and network
values during the build.

USB Serial/JTAG remains a binary development transport. It does not share text
logs with protocol frames.

## Abort contract

Abort is handled before all normal traffic:

1. C6 places `Abort` at the head of its UART transmit queue.
2. C6 retries it until acknowledgement or Duo link failure.
3. Duo checks for abort before advancing any mechanical action.
4. Duo immediately disables all PCA9685 outputs.
5. The active operation and pending action queue are cancelled.
6. Any axis that was moving becomes `Unknown`.
7. Duo enters `Aborted` and publishes the result.

This is an immediate software stop. A servo already in motion still has its
own physical stopping time after PWM is removed.
