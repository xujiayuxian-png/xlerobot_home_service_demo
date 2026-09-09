// Documentation-only fixtures. Serve built assets on loopback; never contact ROS.
import { chromium } from '../../.xlerobot/readme-art/node_modules/playwright/index.mjs';
import { createServer } from 'node:http';
import { readFile, mkdir } from 'node:fs/promises';
import { resolve, extname, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = fileURLToPath(new URL('../../', import.meta.url));
const dist = resolve(root, 'ros2_ws/src/xlerobot_hmi/web/dist');
const output = resolve(root, 'docs/images');
// Display the public head sweep, not repeated zero poses that would be unobservable.
const headPoseYaml = await readFile(resolve(root,
  'ros2_ws/src/xlerobot_calibration_tools/config/head_camera_poses.yaml'), 'utf8');
const headPoses = [...headPoseYaml.matchAll(/\{pan:\s*([\d.-]+), tilt:\s*([\d.-]+)\}/g)]
  .map(([, pan, tilt]) => [Number(pan), Number(tilt)]);
if (headPoses.length !== 25) throw new Error('Update artwork for the changed head pose format.');
let stage = 'head_camera';
const stages = Object.fromEntries(['servo', 'leader', 'head_camera', 'right_handeye', 'hover'].map(key =>
  [key, { present: true, quality_passed: true, freshness: 'current', missing: [] }]));
function snapshot() {
  return { unit: 'docs-demo', active: 'example-v1', runtime_matches_active: true,
    hardware_enabled: false, draft_complete: true, stages, versions: ['example-v1'],
    session: { stage: stage === 'hover' ? null : stage, running: stage !== 'hover',
      generation: stage, error: '', log_path: null } };
}
function visual() {
  const handeye = stage === 'right_handeye', count = handeye ? 26 : 25;
  const metrics = { translation_rmse_mm: 2, translation_p95_mm: 3,
    rotation_rmse_deg: 1, reprojection_rmse_px: .3 };
  return { available: true, state_fresh: true, state_age_s: .1, action_ready: true,
    request_inflight: false, unit_id: 'docs-demo', running: false, phase: 'COMPLETED',
    message: '文档示例：展示采样完成后的界面，不代表实测精度。', pose_index: count - 1,
    pose_count: count, sample_count: count, target_sample_count: handeye ? 26 : 12,
    pose_states: Array(count).fill('captured'), pose_pan: handeye ? Array(count).fill(0) : headPoses.map(p => p[0]),
    pose_tilt: handeye ? Array(count).fill(.8) : headPoses.map(p => p[1]),
    pose_roles: Array.from({ length: count }, (_, i) => i < 20 ? 'fit' : 'validation'),
    result_uri: '', quality_passed: true,
    metrics: handeye ? Object.fromEntries(['fit', 'validation'].flatMap(group =>
      Object.entries(metrics).map(([key, value]) => [`${group}.${key}`, value]))) : metrics,
    target: { accepted: false, fresh: false, tag_count: 0, detail: '离线截图，无相机连接。' } };
}
const arrivals = ['center', 'right', 'left'].map((target, i) => {
  const x = [0, 30, -30][i], dx = [2, 4, 3][i], dy = [-3, -4, -5][i];
  return { target, report_id: `example-${target}`, target_xyz_mm: [x, 0, -200],
    observed_xyz_mm: [x + dx, dy, -180], planar_error_mm: Math.hypot(dx, dy),
    height_mm: 180, height_shortfall_mm: 20, max_joint_error_deg: 1 };
});
const hover = { stale: false, report: { id: 'documentation-example', status: 'COMPLETED',
  message: '示例数据 · 仅展示结果布局，不代表机器人实测精度。', source: 'documentation_fixture',
  updated_at: 'documentation preview', independent_arrivals: 3, frames: 60, arrivals,
  summary: { mean_planar_error_mm: arrivals.reduce((sum, row) => sum + row.planar_error_mm, 0) / 3,
    mean_height_shortfall_mm: 20 },
  suggestion: { frame: 'calibration_board', add_to_target_xyz_m: [-.003, .004, -.02],
    raise_target_mm: 20, between_pose_error_std_mm: [.816, .816, 0],
    advice: '示例参数不可用于机器人；实际建议由本机测量生成，应用前必须重新规划和验证。' } } };
const server = createServer(async (req, res) => {
  try {
    if (req.method !== 'GET') { res.writeHead(405).end(); return; }
    const url = new URL(req.url, 'http://localhost');
    if (url.pathname.includes('/api/') || url.pathname.startsWith('/workbench-api/')) {
      res.setHeader('Content-Type', 'application/json');
      const data = url.pathname === '/workbench-api/status' ? snapshot()
        : url.pathname === '/workbench-api/hover-result' ? hover
          : url.pathname.endsWith('/status') ? visual() : null;
      if (!data) { res.writeHead(404).end(); return; }
      res.end(JSON.stringify(data)); return;
    }
    const file = resolve(dist, `.${url.pathname === '/' ? '/index.html' : url.pathname}`);
    if (!file.startsWith(dist + sep)) { res.writeHead(403).end(); return; }
    let data = await readFile(file);
    if (file.endsWith('index.html')) data = data.toString().replace('<head>',
      '<head><meta name="xlerobot-workbench" content="v1">');
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
  for (const [name, label] of [['head_camera', '头部相机'], ['right_handeye', '手眼标定'], ['hover', '悬停精度']]) {
    stage = name;
    const page = await browser.newPage({ viewport: { width: 1440, height: 1100 } });
    const errors = [];
    page.on('pageerror', error => errors.push(String(error)));
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    await page.getByRole('button', { name: label, exact: true }).click();
    const panel = page.locator(name === 'hover' ? '.hover-report' : '.head-calibration');
    await panel.waitFor();
    await page.getByText(name === 'hover' ? '建议补偿 · 仅作为下一轮验证初值' : '重新标定', { exact: true }).waitFor();
    await panel.evaluate(element => {
      const banner = document.createElement('p');
      banner.textContent = '文档预览 / DOCUMENTATION PREVIEW · 示例数据 · 未连接机器人 · 非实测精度';
      banner.style.cssText = 'padding:16px;background:#26434c;border:1px solid #63aef7;border-radius:8px;color:#fff;font-weight:700';
      element.prepend(banner);
      const camera = element.querySelector('.head-camera-preview');
      if (camera) camera.innerHTML = '<div style="height:260px;display:grid;place-items:center;background:#08171e;color:#9cb2b7">相机画面区域 · 离线预览不连接相机</div>';
    });
    await page.evaluate(() => document.fonts.ready);
    if (errors.length) throw new Error(errors.join('\n'));
    await panel.screenshot({ path: resolve(output, `calibration-${name}.png`) });
    await page.close();
    console.log(`Captured ${name}`);
  }
} finally {
  await browser?.close();
  await new Promise(done => server.close(done));
}
