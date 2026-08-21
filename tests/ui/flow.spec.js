/* Full workflow: recover → grip → scan → solve → execute → release. */
'use strict';
const { test, expect } = require('./lib/sim.js');
const H = require('./lib/cube.js');

test('workflow smoke: scan, solve and execute a cube', async ({ sim }) => {
  await sim.cmd({ command: 'set_animation_speed', multiplier: 2 });
  const open = await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  expect(open.stand.pose.kind).toBe(1);

  const gripped = await sim.runOperation(
    { command: 'grip' },
    s => s.stand.pose.kind === 2 && s.cube_session !== null);
  expect(gripped.cube_session).not.toBeNull();
  await sim.page.screenshot({ path: 'artifacts/flow-grip.png' });

  // A solved scan only checks colour counts. Scramble first so facelet order,
  // scan orientation and solver integration are all exercised.
  await sim.page.fill('#movesInput', 'R D');
  await sim.page.click('#btnMoves');
  await expect.poll(() => sim.idle(), { timeout: 240_000 }).toBe(true);
  await sim.settledPose();
  expect(new Set((await sim.cube()).facelets.slice(0, 9)).size).toBeGreaterThan(1);

  // scan all six faces
  await sim.page.evaluate(() => {
    window.__scanPoseSamples = [];
    window.__scanPoseTimer = setInterval(() => {
      window.__scanPoseSamples.push(window.__animDebug().presentationQuaternion);
    }, 20);
  });
  await sim.cmd({
    command: 'scan',
    session_id: gripped.cube_session.id,
  });
  await expect
    .poll(() => sim.status().then(s => s.scan), { timeout: 180_000 })
    .toEqual(expect.objectContaining({ state: 2, scanned_faces: 0x3f }));
  const scanAnim = await sim.debug();
  const scanPoseSamples = await sim.page.evaluate(() => {
    clearInterval(window.__scanPoseTimer);
    return window.__scanPoseSamples;
  });
  expect(
    Math.max(...scanPoseSamples.map(q => Math.hypot(q[0], q[1], q[2]))),
    'rendered cube visibly rotates during scan',
  ).toBeGreaterThan(0.5);
  expect(scanPoseSamples.some(q => {
    const rotation = Math.hypot(q[0], q[1], q[2]);
    return rotation > 0.08 && rotation < 0.60;
  }), 'scan contains intermediate rendered orientations, not endpoint jumps').toBe(true);
  const angularSteps = scanPoseSamples.slice(1).map((q, index) => {
    const previous = scanPoseSamples[index];
    const dot = Math.min(1, Math.abs(q.reduce(
      (sum, value, component) => sum + value * previous[component], 0)));
    return 2 * Math.acos(dot);
  });
  const largestStep = Math.max(...angularSteps);
  const largestStepIndex = angularSteps.indexOf(largestStep);
  expect(largestStep,
    `including the 180° turn, no frame jumps near ${JSON.stringify(
      scanPoseSamples.slice(Math.max(0, largestStepIndex - 2), largestStepIndex + 4))}`)
    .toBeLessThan(0.35);
  expect(scanAnim.rigidTurnsStarted, 'all rigid scan rotations were observed').toBe(10);
  expect(scanAnim.rigidTurnsCompleted, 'all rigid scan rotations completed').toBe(10);
  expect(scanAnim.presentationQuaternion.map(Math.abs), 'cube returned to canonical pose')
    .toEqual([0, 0, 0, 1]);
  await sim.page.screenshot({ path: 'artifacts/flow-scan.png' });

  // solve from the fresh revision
  const rev = (await sim.status()).scan.revision;
  expect(rev).not.toBeNull();
  await sim.cmd({
    command: 'solve', session_id: gripped.cube_session.id, scan_revision: rev,
  });
  await expect
    .poll(() => sim.status().then(s => s.solution.state))
    .toBe(2); // ready
  const sol = (await sim.status()).solution;
  expect(sol.id).not.toBeNull();
  expect(sol.move_count, 'scrambled scan produces a non-empty solution').toBeGreaterThan(0);

  // execute the solution (empty for a solved cube — still must complete)
  await sim.cmd({
    command: 'execute', session_id: gripped.cube_session.id,
    scan_revision: rev, solution_id: sol.id,
  });
  await expect.poll(() => sim.idle(), { timeout: 240_000 }).toBe(true);
  await sim.settledPose();
  expect((await sim.cube()).facelets).toEqual([
    ...Array(9).fill(0), ...Array(9).fill(2), ...Array(9).fill(4),
    ...Array(9).fill(1), ...Array(9).fill(3), ...Array(9).fill(5),
  ]);
  await sim.page.screenshot({ path: 'artifacts/flow-execute.png' });

  // release the cube; session closes
  await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  expect((await sim.safety()).violations, 'server safety gate remains clean').toEqual([]);
});

test('all 18 face turns stay physically synchronized with their grippers', async ({ sim }) => {
  await sim.cmd({ command: 'set_animation_speed', multiplier: 8 });
  await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  const gripped = await sim.runOperation(
    { command: 'grip' },
    s => s.stand.pose.kind === 2 && s.cube_session !== null);
  const sid = gripped.cube_session.id;

  const before = await sim.debug();
  await sim.page.fill('#movesInput',
    "U U' U2 R R' R2 F F' F2 D D' D2 L L' L2 B B' B2");
  await sim.page.click('#btnMoves');
  await expect.poll(() => sim.idle(), { timeout: 240_000 }).toBe(true);
  await sim.settledPose();

  const log = await sim.page.locator('#log').textContent();
  expect(log).toContain('moves ✓ accepted as operation #');
  expect((await sim.debug()).layerTurnsStarted - before.layerTurnsStarted,
    'each requested move is driven by one matching physical gripper turn').toBe(18);
  expect((await sim.debug()).cubeSnapshotCorrections - before.cubeSnapshotCorrections,
    'rapid telemetry never repairs a layer tween driven by the wrong move').toBe(0);
  expect((await sim.safety()).violations, 'server safety gate remains clean').toEqual([]);
});

test('auto scan-solve-execute animates its computed solution', async ({ sim }) => {
  await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  const gripped = await sim.runOperation(
    { command: 'grip' },
    s => s.stand.pose.kind === 2 && s.cube_session !== null);

  await sim.page.fill('#movesInput', 'R U');
  await sim.page.click('#btnMoves');
  await expect.poll(() => sim.idle(), { timeout: 240_000 }).toBe(true);
  await sim.settledPose();
  const beforeAuto = await sim.debug();
  const layersBeforeAuto = beforeAuto.layerTurnsStarted;

  await sim.runOperation(
    { command: 'auto', session_id: gripped.cube_session.id },
    s => s.stand.pose.kind === 1 && s.cube_session === null);

  expect((await sim.debug()).layerTurnsStarted - layersBeforeAuto,
    'auto execution rendered at least one solution layer').toBeGreaterThan(0);
  expect((await sim.debug()).cubeSnapshotCorrections - beforeAuto.cubeSnapshotCorrections,
    'server snapshots never repair or jump rendered stickers').toBe(0);
  expect(await sim.page.locator('#moves .mv').count(),
    'completed Auto keeps the computed solution visible').toBeGreaterThan(0);
  await expect(sim.page.locator('#solState')).toHaveText('last completed');
  expect((await sim.cube()).facelets).toEqual([
    ...Array(9).fill(0), ...Array(9).fill(2), ...Array(9).fill(4),
    ...Array(9).fill(1), ...Array(9).fill(3), ...Array(9).fill(5),
  ]);
  expect((await sim.safety()).violations, 'server safety gate remains clean').toEqual([]);
});

test('loading and auto-solving two scrambles never replays the first solution', async ({ sim }) => {
  await sim.cmd({ command: 'set_animation_speed', multiplier: 8 });
  await sim.runOperation({ command: 'recover' }, s => s.stand.pose.kind === 1);

  for (const scramble of ['R U', "F D' L"]) {
    const loaded = await sim.cmd({ command: 'load_scramble', sequence: scramble });
    expect(loaded.ok, `load ${scramble}`).toBe(true);
    const gripped = await sim.runOperation(
      { command: 'grip' },
      s => s.stand.pose.kind === 2 && s.cube_session !== null);
    await sim.runOperation(
      { command: 'auto', session_id: gripped.cube_session.id },
      s => s.stand.pose.kind === 1 && s.cube_session === null);
    expect((await sim.cube()).facelets, `${scramble} ends solved`).toEqual([
      ...Array(9).fill(0), ...Array(9).fill(2), ...Array(9).fill(4),
      ...Array(9).fill(1), ...Array(9).fill(3), ...Array(9).fill(5),
    ]);
  }

  expect((await sim.safety()).violations,
    'second Auto never consumes the first Auto solution').toEqual([]);
});
