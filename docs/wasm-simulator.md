# Browser/WASM mode for the robot simulator

## Goal

Add a second simulator runtime that runs entirely in a dedicated browser Web
Worker and can therefore be deployed as static files on GitHub Pages.

The existing native simulator remains supported:

```text
Native mode:  sim.html <- HTTP + SSE -> axum adapter -> SimCore
Pages mode:   sim.html <- postMessage -> Web Worker/WASM -> SimCore
```

Both modes must use the same Rust simulation core. The Pages version is not a
separate JavaScript reimplementation of robot movement, cube state, collision
rules, or solution replay.

## Non-goals

- Do not compile `axum`, `tokio`, TCP listeners, or SSE support into WASM.
- Do not replace the native simulator used before deployment to the mechanism.
- Do not enable WASM threads or depend on `SharedArrayBuffer`. A dedicated
  Worker is sufficient and avoids requiring COOP/COEP response headers.
- Do not make Three.js responsible for authoritative cube state. Rendering
  remains a projection of state produced by `SimCore`.
- Do not duplicate planner or safety logic in JavaScript.

## Current code and required boundaries

### What is already reusable

The following modules contain logic that should remain shared:

- `src/robot_service.rs`: protocol operations, motion plans, scan/solve/execute
  workflows, statuses, and events.
- `src/move_planner.rs`: physical plans for Singmaster moves.
- `src/cube.rs`: cube notation and solver integration.
- `crates/rubik-link-protocol`: wire command/status/event types.
- The server-side cube, visual replay, presentation, interpolation, and safety
  code currently embedded in `src/sim_server.rs`.

### What is native-only today

`src/sim_server.rs` currently combines several responsibilities:

1. Authoritative simulation state (`SimState`, `ServerCube`, `VisualReplay`,
   `CubePresentation`).
2. One simulated robot daemon per browser session.
3. UART frame encoding/decoding used to exercise the real protocol service.
4. OS threads and synchronous channels for the daemon and event pump.
5. HTTP command parsing, session lookup, JSON responses, SSE, and static asset
   serving through `axum`.

Only items 1 and 3 belong in the cross-platform core. Items 2, 4, and 5 are
native transport/runtime concerns.

`RobotService::begin_solver` also calls `std::thread::spawn` and returns the
result through `std::sync::mpsc`. That path cannot be used unchanged in a
single-threaded WASM Worker.

### Browser coupling today

`web/sim.html` directly assumes the native HTTP transport:

- `fetch('/command')` for commands;
- `fetch('/api/status')` and `fetch('/api/cube')` for snapshots;
- `EventSource('/events')` for status, cube, event, and collision updates;
- absolute `/three.js` and `/gripper.stl` asset paths.

These calls need to move behind a small transport interface. Rendering and UI
handlers should not know whether the implementation uses HTTP or a Worker.

## Proposed crate/module layout

Prefer a small crate with minimal dependencies so the WASM build does not pull
in hardware, ML, camera, HTTP, or multi-threaded runtime dependencies:

```text
crates/rubik-sim-core/
  Cargo.toml
  src/
    lib.rs
    command.rs
    core.rs
    cube.rs
    replay.rs
    safety.rs
    snapshot.rs

crates/rubik-sim-wasm/
  Cargo.toml
  src/lib.rs

src/sim_server.rs              # native axum/session adapter
web/
  sim.html
  sim-transport.js
  sim-worker.js
```

If moving code into a crate causes too much churn initially, `sim_core.rs` can
first live in the root crate behind a platform-neutral module. A separate crate
is still the preferred final state because the root package currently depends
on Candle, image processing, Tokio, Axum, and hardware-oriented features.

## `SimCore` API

The core should be deterministic and driven by an explicit virtual clock. It
must not call `Instant::now()`, sleep, spawn threads, or open sockets.

An approximate Rust API:

```rust
pub struct SimCore {
    service: RobotService<SimPwmOutput>,
    simulation: SimState,
    clock: Duration,
    speed: u32,
    outbox: VecDeque<SimMessage>,
}

impl SimCore {
    pub fn new(calibration: StandCalibration) -> Self;
    pub fn restore(snapshot: PersistedSnapshot) -> Result<Self, RestoreError>;

    pub fn command(&mut self, command: SimCommand) -> CommandReply;
    pub fn advance(&mut self, wall_delta: Duration);

    pub fn status(&self) -> StatusEnvelope;
    pub fn cube(&self) -> CubeSnapshot;
    pub fn drain_messages(&mut self) -> Vec<SimMessage>;
    pub fn checkpoint(&self) -> Option<PersistedSnapshot>;
}
```

`advance` multiplies `wall_delta` by the selected animation speed and advances
the robot service, interpolated rail/gripper state, cube replay, collision
checks, and event generation. The native adapter can call it from a timer;
the Worker can call it from `setInterval` or a self-scheduling timer.

### Keep exercising the protocol

The simulator currently passes HTTP commands through `UartFrameEncoder`,
`UartStreamDecoder`, and `RobotService::handle_packet`. This is useful: the
simulator checks the same request codec and rejection behavior as the daemon.

Preserve that path inside `SimCore::command`:

```text
SimCommand
  -> encode rubik-link request
  -> decode frame
  -> RobotService::handle_packet
  -> collect protocol response/events/status
  -> JSON/JS-friendly CommandReply + SimMessage
```

`load_scramble` and `set_animation_speed` remain simulator-only commands and
can be handled directly by `SimCore`, as they are today in the HTTP adapter.

## Solver strategy

The Worker itself already runs off the browser UI thread, so the first WASM
version does not need another thread for `min2phase`.

Introduce a solver boundary instead of calling `std::thread::spawn` directly
from `RobotService`:

```rust
pub trait SolverBackend {
    type Job;

    fn start(&mut self, facelets: String, max_moves: usize) -> Self::Job;
    fn poll(&mut self, job: &mut Self::Job) -> Option<Result<Vec<CubeMove>, String>>;
}
```

Implementations:

- `ThreadedSolver`: current native behavior using an OS thread and channel.
- `InlineSolver`: performs the solve in the Web Worker and returns a ready
  result on the next core tick.

The UI remains responsive while `InlineSolver` blocks because it executes in
the dedicated Worker. Status messages cannot be emitted during that blocking
call, but solving is normally short enough for an MVP. If solver latency later
becomes visible, move solving to a second dedicated Worker rather than enabling
WASM shared-memory threads.

Before implementing the full port, add `wasm32-unknown-unknown` and verify that
the pinned `min2phase` revision and its `rand`/`lazy_static` dependencies compile
for that target. This is a phase-zero go/no-go spike, not an assumption.

## Worker bindings and message protocol

Use `wasm-bindgen` for a narrow boundary. Avoid exposing every Rust struct as a
class. Commands and pushed messages can cross as serialized JS values using
`serde-wasm-bindgen`.

Request envelope:

```json
{
  "type": "request",
  "request_id": 17,
  "method": "command",
  "payload": { "command": "moves", "session_id": 1, "sequence": "R U" }
}
```

Response envelope:

```json
{
  "type": "response",
  "request_id": 17,
  "ok": true,
  "payload": { "operation_id": 4 }
}
```

Push messages retain the current SSE shape so `handleServerMessage` can be
reused unchanged:

```json
{ "type": "status", "status": {}, "visual": {}, "safety": {} }
{ "type": "cube", "cube": {} }
{ "type": "event", "opcode": 4, "payload": {} }
{ "type": "collision", "rule": "...", "active": true }
```

Suggested exported WASM surface:

```rust
#[wasm_bindgen]
pub struct WasmSimulator { core: SimCore }

#[wasm_bindgen]
impl WasmSimulator {
    #[wasm_bindgen(constructor)]
    pub fn new(calibration: JsValue, restored: Option<JsValue>) -> Result<Self, JsValue>;
    pub fn command(&mut self, command: JsValue) -> Result<JsValue, JsValue>;
    pub fn advance(&mut self, elapsed_ms: f64) -> Result<JsValue, JsValue>;
    pub fn status(&self) -> Result<JsValue, JsValue>;
    pub fn checkpoint(&self) -> Result<JsValue, JsValue>;
}
```

`sim-worker.js` owns one `WasmSimulator`, advances it, drains messages, and
forwards those messages with `postMessage`.

## Browser transport abstraction

Add a transport selected at startup:

```javascript
class SimTransport {
  async start(onMessage) {}
  async command(body) {}
  async status() {}
  async cube() {}
  close() {}
}

class HttpSimTransport extends SimTransport {}
class WorkerSimTransport extends SimTransport {}
```

Selection can initially be explicit:

```text
?runtime=server   -> HTTP/SSE
?runtime=wasm     -> dedicated Worker
```

For the Pages build, `wasm` is the default because no server endpoints exist.
For `rubik-robotd-sim`, `server` remains the default. Do not silently fall back
from server to WASM after a command failure; that would create a second robot
state under an existing UI without the operator noticing.

Refactor these existing functions rather than rewriting the scene:

- `command(body)` calls `transport.command(body)`.
- `restoreCubeState()` calls `transport.cube()`.
- `connectEvents()` becomes `transport.start(handleServerMessage)`.
- `handleServerMessage`, `renderStatus`, animation queues, scene code, and
  safety rendering stay transport-independent.

## Tab isolation and reload persistence

### Tab isolation

A dedicated Worker per page naturally isolates live state between tabs. Keep
the existing tab/session identifier for logging, test selectors, and persisted
checkpoint keys, but do not share a Worker or mutable core between tabs.

Do not use a `SharedWorker`: it makes accidental cross-tab state sharing easier
and provides no benefit for this simulator.

### Reload behavior

A dedicated Worker is destroyed on reload. Pages mode therefore needs explicit
checkpoint persistence if it is expected to match native reload behavior.

Persist only authoritative core snapshots, never Three.js objects, animation
fractions, sticker DOM state, or UI-derived facelets.

Recommended policy:

1. `SimCore::checkpoint()` returns a versioned snapshot only at an idle, safe
   boundary (`Ready`, no active operation, no moving axis).
2. The Worker sends the checkpoint to the page after each stable state change.
3. The page stores it in IndexedDB under the tab/session key.
4. A reload supplies the last valid snapshot when constructing the Worker.
5. A schema version mismatch or failed validation resets to cold-start unknown
   state and requires explicit `Recover -> Open`.
6. Reload during motion restores the last stable checkpoint, reports the
   interrupted operation, and never guesses intermediate rail/gripper poses.

This deliberately preserves safety over perfect animation continuation.

The minimum MVP may reset on every reload, but the UI must say so clearly. It
must not partially restore colors while resetting robot pose.

## Static assets and GitHub Pages

GitHub project Pages are normally served below `/<repository>/`, so absolute
paths such as `/three.js` and `/gripper.stl` will point at the domain root and
fail.

Use URLs relative to the current module/document:

```javascript
const workerUrl = new URL('./sim-worker.js', import.meta.url);
const gripperUrl = new URL('./rcr_gripper-v5.stl', import.meta.url);
```

The Three.js import map must also be generated with the Pages base path or
replaced with a relative module import. The native Axum adapter can serve the
same static directory instead of embedding route-specific absolute URLs, or it
can continue embedding assets while injecting an appropriate base URL.

GitHub Pages is static hosting and supports publishing the result of a custom
GitHub Actions build:

- <https://docs.github.com/en/pages/getting-started-with-github-pages/creating-a-github-pages-site>
- <https://docs.github.com/en/pages/getting-started-with-github-pages/what-is-github-pages>

The build should produce one self-contained artifact directory, for example:

```text
dist/
  index.html
  sim-worker.js
  rubik_sim_wasm.js
  rubik_sim_wasm_bg.wasm
  three.js
  rcr_gripper-v5.stl
```

Use a Pages workflow with these logical steps:

1. Checkout.
2. Install the pinned Rust toolchain and `wasm32-unknown-unknown` target.
3. Install a pinned `wasm-bindgen-cli`, `wasm-pack`, or equivalent build tool.
4. Build `rubik-sim-wasm` in release mode for web/worker usage.
5. Assemble `dist/` and add `.nojekyll`.
6. Run browser tests against a local static server serving `dist/` under a
   non-root base path.
7. Upload the Pages artifact and deploy it.

Do not require WASM threads. Public GitHub Pages does not provide a convenient
per-repository mechanism for custom cross-origin isolation headers, and the
single Worker architecture does not need them.

## Testing strategy

The WASM mode must not rely only on unit tests or a successful build.

### Core transcript tests

Run identical command transcripts against `SimCore` natively and in WASM:

```text
recover -> grip -> load scramble -> scan -> solve -> execute
```

Compare normalized outputs:

- command acceptance/rejection;
- controller, stand pose, scan, and solution states;
- event ordering;
- cube revisions and facelets;
- visual move face, sign, index, and monotonic fraction;
- collision/safety violations;
- final canonical presentation basis.

Wall-clock timestamps may be omitted from transcript equality. Virtual elapsed
time and operation ordering must match.

### Rust tests

- Existing planner and robot service tests remain native.
- Move server-independent `ServerCube`, replay, and safety tests into
  `rubik-sim-core`.
- Add tests for snapshot versioning, validation, and interrupted-operation
  restore policy.
- Add a compile/test job for `wasm32-unknown-unknown`.
- Use `wasm-bindgen-test` for the exported binding where useful.

### Playwright matrix

Reuse the existing fixtures through the transport interface and run the same
behavior suite in two projects:

```text
native-server  -> rubik-robotd-sim + ?runtime=server
static-wasm    -> static file server + ?runtime=wasm
```

At minimum both projects must cover:

- independent tabs;
- cold recovery motion serialization;
- grip geometry and collision checks;
- all 18 face turns;
- signed half-turn animation following the gripper;
- scan orientation and six captured faces;
- solution persistence until the next Scan/Auto;
- two different `Load -> Auto` cycles;
- Cube20 compact scramble input;
- reload from a stable checkpoint;
- no cube snapshot correction after visual replay.

The existing Playwright speed override can remain: most tests run at `x8`,
while tests that assert intermediate frame smoothness run at `x2`.

## Safety requirements

Pages mode is useful for visualization and deterministic plan validation, but
native mode remains the final pre-mechanism gate.

The shared core must ensure that both modes evaluate the same rules:

- no concurrent rail and gripper movement during recovery;
- no adjacent parallel-gripper collision;
- no cube custody loss;
- canonical presentation after scan/reorientation;
- physical gripper axis owns the layer being animated;
- visual progress is monotonic and keyed by operation plus move index;
- server/core cube never needs a late browser-side correction.

Browser timers can be throttled when a tab is hidden. `advance` must therefore
accept bounded deltas or subdivide a large elapsed delta into fixed simulation
ticks so it cannot skip safety transitions:

```rust
while remaining > Duration::ZERO {
    let step = remaining.min(Duration::from_millis(20));
    core_tick(step);
    remaining -= step;
}
```

The Worker may stop sending presentation frames while the page is hidden, but
it must not skip internal mechanical states or collision checks.

## Implementation phases

### Phase 0: compatibility spike

- Add the WASM target locally/CI.
- Create a minimal crate using the pinned `min2phase` dependency.
- Compile and execute one solve in a Worker test page.
- Confirm generated `.wasm` size and solve latency.
- Decide whether `min2phase` can remain unchanged or needs a small portability
  patch/fork.

Exit condition: a solved and a scrambled facelet string can be processed in a
single-threaded Worker without WASM threads.

### Phase 1: extract deterministic `SimCore`

- Move `ServerCube`, `VisualReplay`, `CubePresentation`, safety state, command
  normalization, and simulator-only commands out of the Axum module.
- Replace direct wall-clock reads with explicit `advance(delta)` time.
- Keep UART protocol encode/decode inside the core path.
- Add native transcript tests before changing the browser.
- Keep `rubik-robotd-sim` behavior and existing Playwright suite green.

Exit condition: native Axum is a thin adapter and all current native behavior
is unchanged.

### Phase 2: make solving portable

- Add `SolverBackend`.
- Preserve threaded native solving.
- Add single-Worker inline solving.
- Verify Auto does not consume a stale solution ID across repeated cycles.

Exit condition: core workflow tests pass with both solver backends.

### Phase 3: WASM Worker runtime

- Add `rubik-sim-wasm` bindings.
- Define request/response/push envelopes.
- Implement `sim-worker.js`, virtual ticking, speed changes, and error
  propagation.
- Add a tiny standalone debug page before integrating the full Three.js UI.

Exit condition: commands and status/cube/event updates work without HTTP.

### Phase 4: UI transport split

- Add `HttpSimTransport` and `WorkerSimTransport`.
- Route command, initial cube restore, and push messages through the transport.
- Convert static asset URLs to base-path-safe relative URLs.
- Run existing UI against both transports.

Exit condition: the same `sim.html` works locally in native and static modes.

### Phase 5: persistence and safety parity

- Add versioned stable checkpoints.
- Restore per-tab state after reload.
- Handle reload-during-operation as an explicit interrupted recovery case.
- Compare native/WASM transcripts and complete the Playwright matrix.

Exit condition: no partial/stale color restore and no cross-tab state leakage.

### Phase 6: GitHub Pages deployment

- Add the static release build and Pages workflow.
- Test under a repository subpath, not only `/`.
- Add cache/version handling so HTML and WASM cannot be served from different
  builds.
- Document local static preview and deployment commands.

Exit condition: a fresh browser can open the Pages URL and complete two
different `Load -> Auto` cycles with zero safety violations.

## Estimated effort

- MVP without reload persistence: approximately 1–2 focused days after the
  `min2phase` compatibility spike succeeds.
- Reliable dual-mode implementation with persistence, test parity, and Pages
  CI/deployment: approximately 3–5 focused days.
- Add contingency if `min2phase` needs WASM portability work or generated WASM
  size/initialization time is unacceptable.

## Definition of done

- Native and Pages modes use one Rust simulation core.
- Every browser tab owns an isolated Worker/core in Pages mode.
- Stable reload restores one validated authoritative snapshot or explicitly
  resets to cold-start unknown state; it never mixes cube colors and robot pose.
- All existing safety rules run inside the shared core.
- Native and WASM transcript tests agree.
- The Playwright suite passes in both transport modes.
- Static assets work below the GitHub project Pages base path.
- GitHub Actions builds and deploys a version-consistent static artifact.
- Native simulation remains the required final check before deployment to the
  physical mechanism.
