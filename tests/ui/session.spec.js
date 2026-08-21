/* Server-side isolation, reload persistence, and cold-start safety. */
'use strict';
const { test, expect } = require('./lib/sim.js');

const url = (path, session) => `${path}?session=${session}`;
const status = async (request, session) =>
  (await (await request.get(url('/api/status', session))).json()).status;
const command = (request, session, body) =>
  request.post(url('/command', session), { data: body });

test('two browser sessions own independent robot runtimes', async ({ request }) => {
  const first = 'isolation-first';
  const second = 'isolation-second';
  await status(request, second); // create the untouched control session

  const response = await command(request, first, { command: 'recover' });
  expect((await response.json()).response.operation_id).toBe(1);
  await expect.poll(async () => {
    const s = await status(request, first);
    return `${s.controller}:${s.stand.pose.kind}`;
  }).toBe('1:1');

  const untouched = await status(request, second);
  expect(untouched.stand.pose.kind).toBe(0);
  expect(untouched.active_operation).toBeNull();
  expect(untouched.cube_session).toBeNull();
});

test('cold recovery never moves rails and grippers concurrently', async ({ request }) => {
  const session = 'cold-recovery-safety';
  await command(request, session, { command: 'recover' });
  const end = Date.now() + 30_000;
  for (;;) {
    const envelope = await (await request.get(url('/api/status', session))).json();
    const s = envelope.status;
    const railMoving = s.stand.rails.some(axis => axis.motion === 2);
    const gripperMoving = s.stand.grippers.some(axis => axis.motion === 2);
    expect(railMoving && gripperMoving, 'cold recovery motion is serialized').toBe(false);
    expect(envelope.safety.concurrent_rail_gripper_motion).toBe(false);
    if (s.controller === 1 && s.stand.pose.kind === 1) break;
    if (Date.now() > end) throw new Error('cold recovery did not complete');
    await new Promise(resolve => setTimeout(resolve, 20));
  }
});

test('cube snapshot survives reload in the same tab session', async ({ page }) => {
  const session = 'reload-persistence';
  await page.goto(`/?session=${session}`);
  await page.waitForFunction(() => typeof window.__facelets === 'function');
  const facelets = await page.evaluate(() => window.__facelets());
  await page.reload();
  await page.waitForFunction(() => typeof window.__facelets === 'function');
  expect(await page.evaluate(() => window.__facelets())).toEqual(facelets);
});

test('Load applies a scramble instantly without mechanical motion', async ({ sim }) => {
  const beforeStatus = await sim.status();
  const beforeAnim = await sim.debug();
  await sim.page.fill('#movesInput', 'B2L1B2R3F3U3B3L1D3F3L1U1L2B2L3D2B2D2R2B2');
  await sim.page.click('#btnLoadScramble');

  await expect.poll(() => sim.cube().then(cube => cube.revision)).toBe(20);
  const cube = await sim.cube();
  await expect.poll(() => sim.page.evaluate(() => window.__facelets())).toEqual(cube.facelets);
  const afterStatus = await sim.status();
  const afterAnim = await sim.debug();

  expect(afterStatus.stand).toEqual(beforeStatus.stand);
  expect(afterStatus.active_operation).toBeNull();
  expect(afterAnim.layerTurnsStarted).toBe(beforeAnim.layerTurnsStarted);
  expect(afterAnim.rigidTurnsStarted).toBe(beforeAnim.rigidTurnsStarted);
});

test('animation speed buttons are per-session and toggle back to normal', async ({ sim }) => {
  await sim.page.click('#speedControls [data-speed="4"]');
  await expect(sim.page.locator('#speedControls [data-speed="4"]')).toHaveClass(/active/);
  expect((await sim.cmd({ command: 'set_animation_speed', multiplier: 8 })).response)
    .toEqual({ animation_speed: 8 });
  await expect(sim.page.locator('#speedControls [data-speed="8"]')).toHaveClass(/active/);

  await sim.page.click('#speedControls [data-speed="8"]');
  await expect(sim.page.locator('#speedControls .active')).toHaveCount(0);
});
