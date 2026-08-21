const { defineConfig } = require('@playwright/test');

// Never share the operator's default simulator (8123). A test run owns this
// dedicated process and therefore cannot move the stand in a manual tab.
const PORT = Number(process.env.SIM_TEST_PORT ?? 18123);

module.exports = defineConfig({
  testDir: '.',
  testMatch: /.*\.spec\.js/,
  // one stand — tests mutate shared physical state
  workers: 1,
  fullyParallel: false,
  timeout: 300_000,
  expect: { timeout: 45_000 },
  retries: process.env.CI ? 1 : 0,
  reporter: [['list'], ['html', { open: 'never', outputFolder: 'artifacts/html' }]],
  outputDir: 'artifacts/results',
  use: {
    baseURL: process.env.SIM_URL ?? `http://127.0.0.1:${PORT}`,
    viewport: { width: 1365, height: 900 },
    screenshot: 'only-on-failure',
    trace: 'retain-on-failure',
    actionTimeout: 20_000,
  },
  webServer: {
    command:
      `cargo run --bin rubik-robotd-sim --features pca9685 --manifest-path ../../Cargo.toml -- --addr 127.0.0.1:${PORT}`,
    url: `http://127.0.0.1:${PORT}/api/status`,
    // never inherit a stand left in an arbitrary state by a previous run
    reuseExistingServer: false,
    timeout: 180_000,
  },
});
