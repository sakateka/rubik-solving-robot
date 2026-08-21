/* Animation correctness: moves must TWEEN (fine steps, full 9-cubie layers),
 * F'/B must trigger a whole-cube reorientation, presentation must return to
 * canonical, and the final permutation must match the logical model exactly. */
'use strict';
const { test, expect } = require('./lib/sim.js');
const CUBE = require('./lib/cube.js');

test('R U F\' D animates smoothly and lands on the exact permutation', async ({ sim }) => {
  await sim.cmd({ command: 'set_animation_speed', multiplier: 2 });
  await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  await sim.runOperation(
    { command: 'grip' },
    s => s.stand.pose.kind === 2 && s.cube_session !== null);

  const seq = "R U F' D";
  await sim.page.fill('#movesInput', seq);
  await sim.page.click('#btnMoves');

  // wait until the first tween actually starts, then sample to the very end
  expect(await sim.animStarted(), 'animation activity begins after moves').toBe(true);
  const samples = [];
  for (;;) {
    const d = await sim.debug();
    if (d.active) samples.push(d);
    const idle = await sim.idle();
    if (idle && !d.active && d.queue === 0) break;
    await sim.page.waitForTimeout(60);
  }
  await expect
    .poll(async () => {
      const d = await sim.debug();
      return d.active === null && d.queue === 0;
    })
    .toBe(true); // visual replay fully drained

  const active = samples.filter(s => s.active);
  const layerFrames = active.filter(s => s.active.kind === 'layer');
  const wholeFrames = active.filter(s => s.active.kind === 'wholeCube');

  test.info().annotations.push({
    type: 'anim frames',
    description: `layer=${layerFrames.length} whole=${wholeFrames.length}`,
  });

  expect(layerFrames.length, 'layer turns are tweened, not snapped').toBeGreaterThanOrEqual(12);
  expect(Math.max(...layerFrames.map(s => s.active.layerSize), 0),
    'each turn carries the full 9-cubie layer').toBe(9);
  const ts = layerFrames.map(s => s.active.t);
  const positiveSteps = [];
  for (let i = 1; i < ts.length; i++) {
    const d = ts[i] - ts[i - 1];
    if (d > 0) positiveSteps.push(d);
  }
  expect(Math.max(0, ...positiveSteps),
    'tween advances in increments below 150ms').toBeLessThan(0.15);
  expect(wholeFrames.length, "F' triggers an animated reorientation").toBeGreaterThan(0);

  const fin = await sim.debug();
  expect(Math.abs(fin.presentationYaw), 'presentation back to canonical').toBeLessThan(0.05);
  expect(fin.cubeSnapshotCorrections,
    'server snapshot does not repaint a divergent browser cube').toBe(0);

  const expected = CUBE.freshCube();
  for (const mv of CUBE.parseMoves(seq)) CUBE.applyMove(expected, mv.face, mv.turns);
  const diff = CUBE.diffWithScene(expected, await sim.cubies());
  expect(diff, 'final cube matches R U F\' D').toBe('ok');

  const persistedFacelets = await sim.page.evaluate(() => window.__facelets());
  await sim.page.reload();
  await sim.page.waitForFunction(() => typeof window.__facelets === 'function');
  expect(await sim.page.evaluate(() => window.__facelets()),
    'server cube snapshot survives reload').toEqual(persistedFacelets);
  expect((await sim.safety()).violations, 'server safety gate remains clean').toEqual([]);
});

test('a half-turn follows the physical gripper direction', async ({ sim }) => {
  await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  await sim.runOperation(
    { command: 'grip' },
    s => s.stand.pose.kind === 2 && s.cube_session !== null);

  await sim.page.fill('#movesInput', 'R2');
  await sim.page.click('#btnMoves');
  expect(await sim.animStarted()).toBe(true);
  const renderedTurns = [];
  while (!(await sim.idle())) {
    const active = (await sim.debug()).active;
    if (active?.kind === 'layer') renderedTurns.push(active.turns);
    await sim.page.waitForTimeout(15);
  }

  expect(renderedTurns, 'the half-turn was visibly animated').not.toEqual([]);
  expect(new Set(renderedTurns), 'the layer uses the claw\'s signed −180° path')
    .toEqual(new Set([-2]));
  expect((await sim.safety()).violations).toEqual([]);
});
