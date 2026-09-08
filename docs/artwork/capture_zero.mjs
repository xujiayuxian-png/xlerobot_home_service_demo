import { chromium } from '../../.xlerobot/readme-art/node_modules/playwright/index.mjs';
import { mkdir } from 'node:fs/promises';
import { dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const browser = await chromium.launch({
  executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome',
  headless: true,
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
});
try {
  const page = await browser.newPage({ viewport: { width: 1400, height: 850 }, deviceScaleFactor: 1 });
  page.on('pageerror', error => console.error(error));
  await page.goto(`${process.argv[2]}?view=zero`);
  await page.waitForFunction(() => window.renderReady === true, { timeout: 120000 });
  const info = await page.evaluate(() => window.renderInfo);
  if (Object.keys(info.pose).length < 14 || Object.values(info.pose).some(value => value !== 0)) {
    throw new Error('The zero reference must set every movable URDF joint to exactly zero.');
  }
  console.log('zero', info);
  const path = fileURLToPath(new URL('../../ros2_ws/src/xlerobot_hmi/web/public/calibration-zero.png', import.meta.url));
  await mkdir(dirname(path), { recursive: true });
  await page.screenshot({ path });
  console.log(path);
} finally {
  await browser.close();
}
