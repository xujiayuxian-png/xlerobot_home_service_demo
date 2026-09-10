// Documentation-only fixtures: no ROS, local configuration or hardware access.
import { chromium } from '../../.xlerobot/readme-art/node_modules/playwright/index.mjs';
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { resolve, extname, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const language = process.argv[2] || 'zh';
if (!['zh', 'en'].includes(language)) throw new Error('Usage: node docs/artwork/capture_ui.mjs [zh|en]');
const english = language === 'en', suffix = english ? '-en' : '';
const root = fileURLToPath(new URL('../../', import.meta.url));
const dist = resolve(root, 'ros2_ws/src/xlerobot_hmi/web/dist');
const output = resolve(root, 'docs/images');
let workspace = 'operator';
const mapping = { slam: 'OFFLINE', map: null, pose: null, scan: null, path: null };
const health = { readiness: 'BLOCKED', execute_task_available: false, voice_state: 'DISABLED',
  drive_stop_latched: true, diagnostics: {}, requirements: {} };
function bootstrap() {
  return { release: 'UI preview', unit: 'demo-01', site: 'home-demo',
    available_workspaces: [workspace], mapping_phase: 'build', calibration_workflow: '',
    default_dataset_id: 'my-grasps', engineering_tools_enabled: true,
    named_places: [], active_task: null, mapping, perception: null, voice_transcript: '',
    collection: null, drive_stop_latched: true,
    collection_storage: { root: '.xlerobot/artifacts', config_file: 'config/local.yaml' } };
}
const server = createServer(async (req, res) => {
  try {
    if (req.method !== 'GET') { res.writeHead(405).end(); return; }
    const path = new URL(req.url, 'http://localhost').pathname;
    if (path.startsWith('/api/v1/cameras/')) {
      res.setHeader('Content-Type', 'image/svg+xml');
      res.end(`<svg xmlns="http://www.w3.org/2000/svg" width="640" height="260"><rect width="640" height="260" fill="#07151b"/><text x="320" y="135" text-anchor="middle" fill="#7894a1" font-family="sans-serif" font-size="18">${english ? 'OFFLINE PREVIEW · NO CAMERA CONNECTED' : '离线预览 · 未连接相机'}</text></svg>`);
      return;
    }
    if (path.startsWith('/api/')) {
      const data = path === '/api/v1/bootstrap' ? bootstrap()
        : path === '/api/v1/health' ? health
          : path === '/api/v1/tasks' ? { tasks: [] }
            : path.endsWith('/episodes') ? { episodes: [] }
              : path === '/api/v1/sites' ? { site_id: 'home-demo', active_version: '',
                draft: { map_saved: false, map_name: '', map_saved_at: '', places: [], ready: false } }
                : null;
      if (!data) { res.writeHead(404).end(); return; }
      res.setHeader('Content-Type', 'application/json'); res.end(JSON.stringify(data)); return;
    }
    const file = resolve(dist, `.${path === '/' ? '/index.html' : path}`);
    if (!file.startsWith(dist + sep)) { res.writeHead(403).end(); return; }
    let data = await readFile(file);
    // No live state stream is needed for immutable artwork fixtures.
    if (file.endsWith('index.html')) data = data.toString().replace('</head>',
      '<script>window.EventSource=class {close(){}};</script></head>');
    res.setHeader('Content-Type', { '.html': 'text/html', '.js': 'text/javascript',
      '.css': 'text/css', '.png': 'image/png' }[extname(file)] || 'application/octet-stream');
    res.end(data);
  } catch { res.writeHead(404).end(); }
});
await new Promise(done => server.listen(0, '127.0.0.1', done));
let browser;
try {
  browser = await chromium.launch({ executablePath: process.env.CHROME_BIN || '/usr/bin/google-chrome', headless: true });
  await mkdir(output, { recursive: true });
  for (const [name, mode, selector] of [['demo', 'operator', '.operator-workspace'],
    ['mapping', 'mapping', '.mapping-grid'], ['collection', 'collection', '.collection-card']]) {
    workspace = mode;
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    const errors = [];
    page.on('pageerror', error => errors.push(String(error)));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    await page.locator(selector).waitFor();
    if (english) await page.getByRole('button', { name: 'Switch to English', exact: true }).click();
    if (name === 'collection' && english) {
      // Explicitly edit illustrative form data; language switching never changes real data.
      await page.getByPlaceholder('Object label', { exact: true }).fill('shuttlecock');
      await page.getByLabel('Language instruction', { exact: true }).fill('Grasp the shuttlecock');
    }
    if (name === 'mapping') await page.getByText(english ? 'No map draft yet' : '尚无地图草稿', { exact: false }).waitFor();
    await page.evaluate(async () => {
      await document.fonts.ready;
      await Promise.all([...document.images].map(img => img.decode().catch(() => {})));
    });
    await page.locator('.shell').evaluate((element, english) => {
      const banner = document.createElement('p');
      banner.textContent = english ? 'DOCUMENTATION PREVIEW · NO ROBOT OR CAMERA CONNECTED · NOT A HARDWARE TEST'
        : '文档预览 · 未连接机器人或相机 · 不是实机测试';
      banner.style.cssText = 'padding:16px;background:#26434c;border:1px solid #63aef7;border-radius:8px;color:#fff;font-weight:700';
      element.prepend(banner);
    }, english);
    if (english) {
      const untranslated = await page.locator('.shell').evaluate(element => {
        const copy = element.cloneNode(true);
        copy.querySelectorAll('.language-toggle').forEach(node => node.remove());
        const text = copy.textContent + [...copy.querySelectorAll('input')].map(node => node.value).join(' ')
          + [...copy.querySelectorAll('[alt],[title],[placeholder],[aria-label]')].map(node =>
            ['alt', 'title', 'placeholder', 'aria-label'].map(attr => node.getAttribute(attr) || '').join(' ')).join(' ');
        return /\p{Script=Han}/u.test(text);
      });
      if (untranslated) throw new Error(`Untranslated screenshot: ${name}`);
    }
    if (errors.length) throw new Error(errors.join('\n'));
    await page.screenshot({ path: resolve(output, `${name}-ui${suffix}.png`), fullPage: true });
    await page.close();
    console.log(`Captured ${name} (${language})`);
  }
} finally {
  await browser?.close();
  await new Promise(done => server.close(done));
}
