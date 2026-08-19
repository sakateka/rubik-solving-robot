# Robot control architecture and link protocol

This document defines communication between the Android application, the
ESP32-C6 radio controller, and the Milk-V Duo robot controller. It describes
the intended production architecture; the current C6 USB-to-UART bridge is a
bring-up tool, not the final application protocol.

The behavioural command sequences and cube-session rules are specified in
`ROBOT_OPERATION_WORKFLOWS.md`.

## Responsibility boundaries

```text
Android application
        │ BLE GATT
        ▼
ESP32-C6
  BLE ↔ UART gateway
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

The ESP32-C6 owns BLE connectivity, UART framing and forwarding, and link
status. It does not decide how to move the cube. `Abort` is the only command
that receives transport priority: C6 places it ahead of queued traffic and
retries it until Duo acknowledges it.

The Android application presents the state and sends semantic commands. It
must never send PCA9685 channel values or servo pulse widths.

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
view even if notifications were lost.

Sticker confidence is transmitted as an integer from 0 to 255. This is enough
for the mobile UI to mark uncertain recognition without exposing model-specific
floating-point details.

Mechanical actions are semantic, for example `SetRail`, `SetGripper`, `Wait`,
`CaptureFace`, and `RecognizeFace`. Raw PWM values are diagnostic information
and are not part of the normal mobile API.

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

Request IDs are allocated by the phone. Duo keeps a bounded cache of recent
request results. Receiving the same ID again returns the previous result and
must not start a second scan or execute a move twice.

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

## BLE transport

C6 exposes one custom GATT service with at least two characteristics:

- command: phone writes packets to C6;
- event: C6 sends notifications to the phone.

The transport-neutral packet is reused, but COBS and the terminating zero are
UART-specific. BLE messages larger than the negotiated ATT payload require a
small fragmentation envelope containing packet ID, fragment index, and
fragment count.

Business payloads pass through C6 unchanged. C6 only needs to understand the
wire envelope sufficiently to prioritize `Abort`, correlate delivery, and
report UART/BLE link status.

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
