# Robot operation workflows

This document defines the behavioural contract of the Rubik robot: how a
physical cube enters and leaves the stand, which operations may follow one
another, and when scan or solution data becomes stale. Wire layout and opcode
numbers are defined separately in `ROBOT_CONTROL_PROTOCOL.md`.

## Core invariants

1. Milk-V Duo is the only component that owns robot state and motion planning.
2. Every normal operation starts and ends in a known logical stand pose.
3. A successful `Grip` creates a new cube session. Scan and solution data are
   valid only inside that session.
4. `Open`, `Abort`, restart, or loss of mechanical position ends or invalidates
   the session.
5. A solution may execute only against the same session and scan revision from
   which it was calculated.
6. Any manually requested cube move invalidates the saved scan and solution.
7. Recognition failure is recoverable: return to canonical grip and keep the
   cube held. A hardware or motion failure is fatal and requires recovery.
8. `Abort` and `GetStatus` remain admissible while another operation is active.

The protocol reports commanded mechanical state. The current servos provide no
measured position feedback.

## Logical stand poses

Fine-grained rail and gripper positions remain available in `StandState`, but
command admission uses a higher-level pose:

- `Unknown`: physical position cannot be trusted.
- `Open`: all rails are open and grippers are in the safe perpendicular pose.
- `CanonicalGrip`: the cube is held with the operator-selected Front facing the
  camera and all grippers perpendicular.
- `ScanPose`: a known face is exposed to the camera during scanning.
- `MovePose`: a temporary known pose used to execute a logical cube move.
- `Transitional`: a bounded mechanical action is currently changing pose.

No normal scan, solve, or move command is accepted from `Unknown`.

## Startup and recovery

After process or board startup, outputs are off and stand pose is `Unknown`.
Software must not assume that a previous process left the mechanism open.

`RecoverToOpen` performs the collision-safe sequence:

1. Do not rotate any gripper.
2. Command left/right rails open and wait for their configured duration.
3. Command top/bottom rails open and wait.
4. With all rails open, rotate all grippers to `FramePerpendicular`.
5. Disable PWM outputs and publish pose `Open`.

Recovery discards any cube session, scan, and solution.

## Cube session

A cube session identifies one uninterrupted period during which the robot has
custody of one physical cube:

```text
Grip succeeds                    session 42 begins
Scan succeeds                    scan revision 1 belongs to session 42
Solve succeeds                   solution 7 belongs to scan revision 1
Execute or ExecuteMoves changes  only the cube in session 42
Open succeeds                    session 42 ends
```

Session, scan revision, and solution IDs prevent delayed or duplicated client
commands from operating on stale physical state. Mutating commands carry the
IDs they expect. Duo rejects a command if they do not match its current state.

## Place and grip

Precondition: controller `Ready`, stand pose `Open`, no cube session.

The operator places a cube with the chosen Front facing the camera and sends
`Grip`. Duo closes the rails using the already validated safe sequence. A new
session ID is published only after the operation completes in
`CanonicalGrip`.

Without force or position sensors, custody `Held` means that the commanded
grip sequence completed; it is not an independent physical measurement.

## Manual cube moves

`ExecuteMoves` accepts a bounded Singmaster sequence for the current session.
The executor starts in `CanonicalGrip`, performs each logical move through the
validated mechanical primitives, and returns to `CanonicalGrip` after the
complete sequence. Between compatible opposite-face moves, the planner may
keep one axis pair in `MovePose` and defer its shared regrip. Temporary
whole-cube reorientation for a contiguous `F/B` block is never exposed as a
logical change of cube orientation.

After the first successful manual move, existing scan and solution data is
invalidated. The initial implementation does not attempt to transform a saved
facelet mathematically.

## Scan only

`StartScan` requires the current session and `CanonicalGrip`. It performs the
validated canonical face sequence, captures training artifacts for every face,
and returns the cube to its original Front orientation.

Unlike the existing diagnostic scan command, the daemon does **not** open the
stand after `StartScan`. Successful completion leaves:

```text
controller = Ready
stand pose = CanonicalGrip
session = unchanged, custody Held
scan = Valid(new revision)
solution = None
```

If recognition or facelet validation fails while mechanical state remains
known, Duo returns to `CanonicalGrip`, preserves the artifacts and partial scan
status, and reports a recoverable operation failure. The operator may retry the
full scan or open the stand.

## Solve and execute

`Solve` requires a valid scan revision in the current held session. It invokes
min2phase without moving the stand and publishes a solution ID tied to that
session and scan revision.

`Execute` requires all three IDs to match current state:

```text
session ID
scan revision
solution ID
```

Execution publishes logical moves and a bounded preview of semantic mechanical
actions. `MoveCompleted` advances logical progress, while compatible moves may
share a non-canonical mechanical holding mode. After the final move, the
planner restores canonical grip, performs the normal orientation-preserving
open sequence and ends the session.

The normal mobile workflow can present `Solve` and `Execute` as one “Solve
cube” action while retaining separate protocol commands for diagnostics.

## Automatic scan, solve, and execute

`ScanSolveExecute` requires a held session in `CanonicalGrip` and runs:

```text
scan → validate → solve → execute → restore Front → open
```

There is no release between scan and execution. A recoverable scan or solver
failure leaves the cube held in `CanonicalGrip`; a fatal mechanical failure
disables outputs and requires recovery.

## Physical operator button

The active-low button has two wires between Milk-V Duo256M `GP21` (Linux GPIO
`506`) and GND. Before opening the GPIO, `rubik-robotd` verifies that the pad is
still muxed as `XGPIOA[26]`, enables its internal weak pull-up through the
vendor-defined SG2002 pad register, and verifies the register read-back. No
external resistor is needed for this pin. The daemon polls it with 50 ms
debounce. A button held while the daemon starts is ignored until it has been
released and pressed again, so startup alone cannot move the stand.

One debounced press selects a workflow from authoritative Duo state:

| Current state | Button action |
|---|---|
| No operation, pose `Open` | `Grip`; after that exact operation succeeds, `ScanSolveExecute` for the new cube session. |
| No operation, any other known pose or `Unknown` | `RecoverToOpen`. |
| Any active operation | Priority `Abort`; after the controller reaches `Aborted`, `RecoverToOpen`. |

The follow-up is bound to the operation and cube-session IDs created by the
service. A concurrent operation, rejected command, failed grip, or fault
cancels it rather than guessing what happened. In particular, if disabling PWM
during `Abort` fails and the controller enters `Faulted`, the button does not
start recovery motion automatically. Pressing the button again while a
button-started grip or automatic workflow is still active follows the same
`Abort` then recovery path.

## Normal open

`Open` is session-bound. Duo first restores the original Front and canonical
perpendicular grip, then opens left/right rails, waits, opens top/bottom rails,
waits, and disables outputs. Successful opening ends the session and clears
scan and solution data.

`Open` is not a substitute for recovery from unknown state.

## Abort and failure handling

`Abort` is independent of session IDs and has transport and dispatcher
priority. Duo immediately disables all PWM outputs, cancels the active action
queue, marks moving axes and logical stand pose `Unknown`, invalidates the cube
session, and admits only status or recovery commands.

Recoverable operation failures include recognition and invalid facelet data
when the stand can be returned to canonical grip. Fatal failures include I2C or
motion failures that make mechanical state uncertain.

## Reconnection

Loss of Wi-Fi or the browser connection does not abort an autonomous Duo
operation. The client may reconnect and issue `GetStatus`; operation ID,
current action, scan progress and plan preview reconstruct the UI. Only an
explicit `Abort` stops the robot.

## Deferred options

Two useful extensions are intentionally outside the initial implementation:

- an independent hardware emergency-stop path, such as PCA9685 OE or servo
  power control;
- an optional verification scan after execution and before release.

They are recorded here so the architecture does not preclude them, but they
are not current milestones.
