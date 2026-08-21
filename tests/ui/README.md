# UI tests (Playwright Test)

Browser-level tests for the `rubik-robotd-sim` operator UI. They drive the
real HTTP server through the same endpoints as the web frontend, sample the
in-page probes (`window.__animDebug`, `__sceneDebug`, `__cubies`), and verify
mechanics that are impossible to eyeball: tween smoothness, exact cube
permutations after a sequence, gripper reach, collision-free poses.

## Run

```bash
npm install            # once, in tests/ui/
npm test               # starts the sim itself if it isn't already running
npx playwright show-report   # HTML report with traces/screenshots of failures
```

The runner reuses an already-running simulator (`reuseExistingServer`);
set `SIM_NO_SERVER=1` to forbid auto-start, `SIM_URL=` to point elsewhere.

## What each spec covers

| spec | checks |
|------|--------|
| `flow.spec.js` | full workflow: recover → grip → scan (6/6) → solve → execute → release; readable event log |
| `anim.spec.js` | moves TWEEN in fine steps; layers carry 9 cubies; F' animates whole-cube reorientation; final permutation matches the logical model exactly (integer math) |
| `grip.spec.js` | claws retract when open, press the face flush on Grip, closed grippers never overlap |

## Probes exposed by sim.html

- `window.__animDebug()` — animation queue/active tween state
- `window.__sceneDebug()` — world AABBs of the cube and every gripper
- `window.__cubies()` — quantized cubie positions + orientation bases

Artifacts (traces, failure screenshots, manual PNGs, HTML report) land in
`tests/ui/artifacts/`.
