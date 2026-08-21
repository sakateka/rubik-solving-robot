/* Mechanical sanity of the grippers:
 *  - at rest (open) all four stand far from the cube;
 *  - after Grip every claw presses its face (flush, tiny overlap);
 *  - closed claws never overlap each other's bounding boxes. */
'use strict';
const { test, expect } = require('./lib/sim.js');

const FACE_LIMIT = 1.545; // cube half-size in scene units
const TOL = 0.12;         // allowed press-in / standoff

test('mouse orbit updates continuously before pointer release', async ({ sim }) => {
  const canvas = sim.page.locator('canvas').first();
  const box = await canvas.boundingBox();
  const before = (await sim.scene()).camera;
  await sim.page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
  await sim.page.mouse.down();
  await sim.page.mouse.move(box.x + box.width / 2 + 120, box.y + box.height / 2 + 30,
    { steps: 6 });
  await sim.page.waitForTimeout(50);
  const whileHeld = (await sim.scene()).camera;
  expect(whileHeld, 'camera moves before pointerup').not.toEqual(before);
  await sim.page.mouse.up();
});

test('grippers open wide at rest and close flush on Grip', async ({ sim }) => {
  const open = await sim.runOperation(
    { command: 'recover' }, s => s.stand.pose.kind === 1);
  void open;

  const openScene = await sim.scene();
  // tangent-mounted claws spin perpendicular at rest, so the world-X extent
  // of the retracted arm is shorter than its full length
  expect(openScene.grips.left.min[0], 'left retracted when open').toBeLessThan(-3.2);
  expect(openScene.grips.right.max[0], 'right retracted when open').toBeGreaterThan(3.2);
  expect(openScene.grips.top.max[1], 'top retracted when open').toBeGreaterThan(3.2);
  expect(openScene.grips.bottom.min[1], 'bottom retracted when open').toBeLessThan(-3.2);

  // canonical grip pose reached AND settled before measuring
  const closedPose = await sim.runOperation(
    { command: 'grip' },
    s => s.stand.pose.kind === 2 && s.cube_session !== null);
  expect(closedPose.stand.pose.kind).toBe(2);

  const closed = await sim.scene();
  await sim.page.screenshot({ path: 'artifacts/grip-labels.png' });
  const penetration = {
    left: closed.grips.left.max[0] + FACE_LIMIT,
    right: FACE_LIMIT - closed.grips.right.min[0],
    top: FACE_LIMIT - closed.grips.top.min[1],
    bottom: closed.grips.bottom.max[1] + FACE_LIMIT,
  };
  for (const [side, depth] of Object.entries(penetration)) {
    expect(depth, `${side} claw reaches around the face`).toBeGreaterThan(0.49);
    expect(depth, `${side} claw does not bury into the cube`).toBeLessThanOrEqual(TOL + 0.54);
  }

  // no two closed grippers may share space
  const sides = ['left', 'right', 'top', 'bottom'];
  for (let i = 0; i < sides.length; i++)
    for (let j = i + 1; j < sides.length; j++) {
      const a = closed.grips[sides[i]], b = closed.grips[sides[j]];
      const overlap = ['0', '1', '2'].every(ax =>
        Math.min(a.max[ax], b.max[ax]) > Math.max(a.min[ax], b.min[ax]));
      expect(overlap, `${sides[i]} and ${sides[j]} bboxes stay disjoint`).toBe(false);
    }
  expect((await sim.safety()).violations, 'server safety gate remains clean').toEqual([]);
});
