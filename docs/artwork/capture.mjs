import { chromium } from '../../.xlerobot/readme-art/node_modules/playwright/index.mjs';
import { fileURLToPath } from 'node:url';

const browser = await chromium.launch({
  executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome',
  headless: true,
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'],
});
try {
  const page = await browser.newPage({ viewport: { width: 1600, height: 900 }, deviceScaleFactor: 1 });
  page.on('pageerror', error => console.error(error));
  for (const view of ['hero', 'detail']) {
    await page.goto(`${process.argv[2]}?view=${view}`);
    await page.waitForFunction(() => window.renderReady === true, { timeout: 120000 });
    console.log(view, await page.evaluate(() => window.renderInfo));
    const path = fileURLToPath(new URL(`../images/robot-${view}.png`, import.meta.url));
    await page.screenshot({ path });
    console.log(path);
  }
} finally {
  await browser.close();
}
