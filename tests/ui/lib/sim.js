/* Shared fixtures for the simulator UI specs. */
const base = require('@playwright/test');
const crypto = require('node:crypto');
const sessionUrl = (path, session) => `${path}?session=${session}`;

async function getStatus(request, session) {
  const res = await request.get(sessionUrl('/api/status', session));
  return (await res.json()).status;
}

async function getSafety(request, session) {
  const res = await request.get(sessionUrl('/api/status', session));
  return (await res.json()).safety;
}

const test = base.test.extend({
  sessionId: async ({}, use, testInfo) => {
    const id = crypto.createHash('sha256').update(testInfo.testId).digest('hex').slice(0, 16);
    await use(`playwright-${id}`);
  },
  /** Navigated page with the UI fully booted (probes exposed). */
  uiPage: async ({ page, sessionId }, use) => {
    await page.goto(`/?session=${sessionId}`);
    await page.waitForFunction(() =>
      typeof window.__animDebug === 'function' && typeof window.__cubies === 'function');
    await use(page);
  },

  /** High-level simulator client: commands, status, scene probes. */
  sim: async ({ uiPage: page, request, sessionId }, use) => {
    const cmd = async (body) => {
      const res = await request.post(sessionUrl('/command', sessionId), { data: body });
      return res.json();
    };
    // Mechanical integration tests care about ordering and final state, not
    // wall-clock servo timing. Each test owns a server session, so accelerate
    // it without affecting the developer's browser session.
    await cmd({ command: 'set_animation_speed', multiplier: 8 });
    await use({
      page,
      cmd,
      status: () => getStatus(request, sessionId),
      safety: () => getSafety(request, sessionId),
      cube: async () => {
        const res = await request.get(sessionUrl('/api/cube', sessionId));
        return res.json();
      },
      /** Run a mechanical operation and wait for this exact operation id to
       * start and settle. This avoids accepting a stale Ready/Open snapshot. */
      runOperation: async (body, finalState) => {
        const reply = await cmd(body);
        const operationId = reply.response?.operation_id;
        if (!reply.ok || operationId == null)
          throw new Error(`command was not accepted: ${JSON.stringify(reply)}`);
        const end = Date.now() + 120_000;
        let started = operationId == null;
        while (Date.now() < end) {
          const s = await getStatus(request, sessionId);
          if (s.active_operation?.id === operationId) started = true;
          if (started && s.controller === 1 && s.active_operation === null &&
              (!finalState || finalState(s))) {
            await new Promise(r => setTimeout(r, 350));
            return getStatus(request, sessionId);
          }
          await new Promise(r => setTimeout(r, 100));
        }
        throw new Error(`operation #${operationId} never settled`);
      },
      /** Truly idle: controller Ready AND no operation in flight.
       * (A freshly booted stand also reports no operation, so callers that
       * need a specific end state should additionally poll the pose.) */
      idle: () =>
        getStatus(request, sessionId).then(
          s => s.controller === 1 && s.active_operation === null),
      poseKind: () => getStatus(request, sessionId).then(s => s.stand.pose.kind),
      settledPose: async () => {
        // wait until idle, then give the final SSE frame a moment to land
        const end = Date.now() + 120_000;
        while (Date.now() < end) {
          const s = await getStatus(request, sessionId);
          if (s.controller === 1 && s.active_operation === null) {
            await new Promise(r => setTimeout(r, 350));
            return s;
          }
          await new Promise(r => setTimeout(r, 150));
        }
        throw new Error('stand never went idle');
      },
      debug: () => page.evaluate(() => window.__animDebug()),
      /** Resolves once a tween/queue activity appears (operation started). */
      animStarted: async () => {
        const end = Date.now() + 30_000;
        while (Date.now() < end) {
          const d = await page.evaluate(() => window.__animDebug());
          if (d.active || d.queue > 0) return true;
          await new Promise(r => setTimeout(r, 80));
        }
        return false;
      },
      scene: () => page.evaluate(() => window.__sceneDebug()),
      cubies: () => page.evaluate(() => window.__cubies()),
    });
  },
});

module.exports = { test, expect: base.expect };
