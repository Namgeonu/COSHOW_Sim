// Development-only browser verification. No Node packages are used by the dashboard.
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import {createRequire} from 'node:module';
const require = createRequire(import.meta.url);
const modules = process.env.DASHBOARD_QA_NODE_MODULES;
const {chromium} = require(modules ? path.join(modules, 'playwright') : 'playwright');
const sharp = require(modules ? path.join(modules, 'sharp') : 'sharp');
const base = process.env.DASHBOARD_QA_URL || 'http://127.0.0.1:8088';
const route = process.env.DASHBOARD_QA_PAGE || '/visitor.html';
const images = 'dashboard/REPORTS/img';
await fs.mkdir(images, {recursive: true});
const browser = await chromium.launch({executablePath: process.env.CHROMIUM_EXECUTABLE || undefined});
const context = await browser.newContext({viewport: {width: 1920, height: 1080}, deviceScaleFactor: 1});
const external = [], requests = new Set(), errors = [], warnings = [];
await context.route('**/*', async request => {
  const url = request.request().url();
  if (new URL(url).origin !== new URL(base).origin) {
    external.push(url); await request.abort();
  } else { requests.add(new URL(url).pathname); await request.continue(); }
});
const page = await context.newPage();
page.on('pageerror', error => errors.push(error.message));
page.on('console', message => { if (message.type() === 'warning') warnings.push(message.text()); });
let hello, state;
page.on('websocket', socket => socket.on('framereceived', ({payload}) => {
  if (typeof payload !== 'string') return;
  const value = JSON.parse(payload);
  if (value.type === 'hello') hello = value;
  if (value.type === 'state') state = value;
}));
const wait = async (predicate, label, timeout = 12000) => {
  const end = Date.now() + timeout;
  while (!predicate()) {
    assert.ok(Date.now() < end, `Timed out: ${label}`);
    await new Promise(resolve => setTimeout(resolve, 50));
  }
};

async function geometry(label, expectedRoles = 6, expectedCameras = 4) {
  const result = await page.evaluate(() => {
    const visible = e => e.getBoundingClientRect().width > 0;
    const rect = e => ({...e.getBoundingClientRect().toJSON(), name: e.dataset.region || e.className});
    const regions = [...document.querySelectorAll('[data-region]')].filter(visible).map(rect);
    const targets = [...document.querySelectorAll('h1,h2,h3,p,dt,dd,.phase-pill,.camera-name,.interest')].filter(visible);
    const clipped = targets.filter(e => e.scrollWidth > e.clientWidth + 2 || e.scrollHeight > e.clientHeight + 2).map(e => e.className || e.tagName);
    const outside = [...document.querySelectorAll('[data-region],.robot-panel,.camera-tile')].filter(visible).map(rect)
      .filter(r => r.left < -1 || r.top < -1 || r.right > innerWidth + 1 || r.bottom > innerHeight + 1);
    const canvas = document.querySelector('canvas').getBoundingClientRect();
    const head = document.querySelector('.head');
    const bounds = head.getBoundingClientRect();
    const headerOverflow = [...head.querySelectorAll('*')].map(rect).filter(r =>
      r.left < bounds.left - 1 || r.right > bounds.right + 1 || r.top < bounds.top - 1 || r.bottom > bounds.bottom + 1);
    return {regions, clipped, outside, headerOverflow, viewport: [innerWidth, innerHeight],
      document: [document.documentElement.scrollWidth, document.documentElement.scrollHeight],
      roles: document.querySelectorAll('.robot-panel').length,
      cameras: [...document.querySelectorAll('.camera-tile')].map(rect),
      narrativeFont: getComputedStyle(document.querySelector('#narrative')).fontSize,
      canvasRatio: canvas.width / canvas.height};
  });
  assert.deepEqual(result.clipped, [], label + ' text clipped');
  assert.deepEqual(result.outside, [], label + ' outside viewport');
  assert.deepEqual(result.document, result.viewport, label + ' document scroll');
  assert.equal(result.roles, expectedRoles);
  assert.equal(result.cameras.length, expectedCameras);
  for (const r of result.cameras) assert.ok(Math.abs(r.width / r.height - 324 / 244) < .002);
  assert.ok(Math.abs(result.canvasRatio - 7 / 6) < .002);
  assert.deepEqual(result.headerOverflow, [], label + ' header child outside parent');
  assert.ok(result.regions.find(r => r.name === 'fleet').height <= result.viewport[1] * .22 + 1);
  console.log('LAYOUT', label, JSON.stringify(result));
}

async function contrast() {
  // Remove glyphs only, sample the actual composited backdrop below every ink-2 label.
  const labels = await page.locator('body *').evaluateAll(es => es.filter(e => e.getBoundingClientRect().width > 0 &&
    e.textContent.trim() && getComputedStyle(e).color === 'rgba(244, 246, 251, 0.72)').map((e, i) => {
    e.dataset.contrast = String(i);
    return {text: e.textContent, rect: e.getBoundingClientRect().toJSON()};
  }));
  const style = await page.addStyleTag({content: '[data-contrast]{color:transparent !important}'});
  const pixels = await sharp(await page.screenshot()).removeAlpha().raw().toBuffer({resolveWithObject: true});
  await style.evaluate(element => element.remove());
  const luminance = rgb => rgb.map(v => v / 255).map(v => v <= .04045 ? v / 12.92 : ((v + .055) / 1.055) ** 2.4)
    .reduce((sum, v, i) => sum + v * [.2126, .7152, .0722][i], 0);
  const results = labels.map(label => {
    let min = Infinity, sample = null;
    const r = label.rect;
    for (let y = Math.ceil(r.top); y < Math.floor(r.bottom); y++) {
      for (let x = Math.ceil(r.left); x < Math.floor(r.right); x++) {
        const index = (y * pixels.info.width + x) * pixels.info.channels;
        const bg = [...pixels.data.subarray(index, index + 3)];
        const fg = bg.map((v, i) => [244, 246, 251][i] * .72 + v * .28);
        const ratio = (luminance(fg) + .05) / (luminance(bg) + .05);
        if (ratio < min) {min = ratio; sample = {x, y, bg};}
      }
    }
    return {text: label.text, minimum: +min.toFixed(3), ...sample};
  });
  console.log('CONTRAST every visible ink-2 label; foreground rgba(244,246,251,.72)', JSON.stringify(results));
  assert.ok(results.every(result => result.minimum >= 4.5), 'label contrast >= 4.5');
}

let admin;
const fixtureTimers = new Set();
try {
  await page.goto(base + route);
  await wait(() => hello && state, 'initial protocol');
  assert.equal(hello.mock, true, 'requires --mock server');
  admin = new WebSocket(base.replace('http', 'ws') + '/ws?role=admin');
  await new Promise((resolve, reject) => {admin.onopen = resolve; admin.onerror = reject;});
  assert.ok(['IDLE', 'DONE', 'ABORTED'].includes(state.run.state), 'start a fresh mock or wait for its run to finish');
  if (state.run.state !== 'IDLE') admin.send(JSON.stringify({cmd: 'reset'}));
  await page.waitForSelector('.visitor[data-mode="idle"]');
  await page.evaluate(() => document.fonts.ready);
  await page.waitForFunction(() => [...document.images].every(img => img.complete && img.naturalWidth));
  const assets = await page.evaluate(async () => ({
    font: document.fonts.check('500 30px Pretendard'),
    fontFaces: [...document.fonts].map(f => ({family: f.family, status: f.status})),
    three: (await import('/static/vendor/three.module.js')).REVISION,
  }));
  assert.ok(assets.font && assets.fontFaces.some(f => f.family === 'Pretendard' && f.status === 'loaded'));
  assert.equal(assets.three, '186');
  console.log('ASSETS', JSON.stringify(assets));
  await geometry('FHD idle');
  await page.screenshot({path: `${images}/M3_idle_fhd.png`});
  const intro = await page.locator('.idle-intro p').allTextContents();
  assert.deepEqual(intro, hello.idle_lines);
  console.log('IDLE hello.idle_lines rendered verbatim', JSON.stringify(intro));

  admin.send(JSON.stringify({cmd: 'preflight'}));
  await wait(() => state?.run?.state === 'READY', 'READY');
  admin.send(JSON.stringify({cmd: 'start'}));
  await wait(() => state?.mission?.phase === 'search', 'search', 45000);
  await page.waitForSelector('.visitor[data-phase="search"]');
  console.log('SEARCH protocol', JSON.stringify({run: state.run, mission: state.mission}));
  await geometry('FHD search');
  await page.screenshot({path: `${images}/M3_search_fhd.png`});
  await contrast();
  await page.setViewportSize({width: 3840, height: 2160});
  await page.waitForFunction(() => document.querySelector('canvas').getBoundingClientRect().width > 1000);
  await geometry('4K search');
  await page.screenshot({path: `${images}/M3_search_4k.png`});
  await page.setViewportSize({width: 1920, height: 1080});
  await page.evaluate(() => document.documentElement.dataset.fx = 'low');
  const low = await page.locator('.glass').first().evaluate(e => ({filter: getComputedStyle(e).backdropFilter, bg: getComputedStyle(e).backgroundColor}));
  assert.deepEqual(low, {filter: 'none', bg: 'rgba(14, 21, 48, 0.78)'});
  await page.emulateMedia({reducedMotion: 'reduce'});
  assert.ok(await page.evaluate(() => matchMedia('(prefers-reduced-motion: reduce)').matches));
  console.log('LOW_FX_REDUCED_MOTION', JSON.stringify(low));
  // Explicit browser-only protocol fixture: changed role count, external BT, and WS recovery.
  // The required screenshots above always come from the actual --mock CLI timeline.
  const fixtureHello = structuredClone(hello);
  fixtureHello.drones = fixtureHello.drones.slice(0, 3);
  fixtureHello.lanes = Object.fromEntries(Object.entries(fixtureHello.lanes).filter(([id]) => fixtureHello.drones.includes(id)));
  const fixtureState = structuredClone(state);
  fixtureState.run = {...fixtureState.run, state: 'IDLE', external_bt: true};
  fixtureState.mission = {...fixtureState.mission, phase: 'capture', finder: fixtureHello.drones[1], age: 0};
  fixtureState.mission.cmd[fixtureHello.limos[0]] = {kind: 'nav', goal: [-1, 1.5], t: 30};
  fixtureState.robots[fixtureHello.drones[1]].detections = [1, 2, 3];
  let stream = true;
  let connections = 0;
  await page.routeWebSocket('**/ws?role=visitor', socket => {
    connections++;
    socket.send(JSON.stringify(fixtureHello));
    const interval = setInterval(() => {if (stream) socket.send(JSON.stringify(fixtureState));}, 100);
    fixtureTimers.add(interval);
    socket.onClose(async (code, reason) => {
      clearInterval(interval); fixtureTimers.delete(interval);
      await socket.close({code, reason});
    });
  });
  await page.reload();
  await page.waitForSelector('.visitor[data-phase="capture"]');
  await page.evaluate(() => document.fonts.ready);
  await geometry('browser fixture: 3 drones + 2 LIMOs', 5, 3);
  assert.equal(await page.locator('.robot-panel[data-kind="limo"] .metric-value').nth(1).textContent(), '−1.0, 1.5');
  assert.match(await page.locator('#narrative').textContent(), new RegExp(fixtureHello.display_names[fixtureHello.drones[1]]));
  assert.equal(await page.locator('.interest:not([hidden])').first().textContent(), '3번');
  stream = false;
  await page.waitForSelector('.visitor[data-mode="offline"]', {timeout: 5000});
  assert.equal(await page.locator('.visitor').getAttribute('data-signal'), 'off');
  await wait(() => connections >= 2, 'reconnect', 5000);
  stream = true;
  await page.waitForSelector('.visitor[data-phase="capture"]');
  console.log('BROWSER FIXTURE PASS: 5 role panels / 3 camera tiles; external BT display; interest-only badge; 2s stalled publisher hides phase; reconnect recovered', {connections});
  assert.deepEqual(external, []);
  assert.deepEqual(errors, []);
  assert.deepEqual(warnings, []);
  console.log('NETWORK external attempts=0; all non-data requests', JSON.stringify([...requests].sort()));
  console.log('BROWSER pageerrors=0 warnings=0', browser.version());
  console.log('PASS M3 actual --mock idle/search, FHD/4K, local assets, contrast, low effects; separate dynamic/reconnect browser fixture');
} finally {
  for (const interval of fixtureTimers) clearInterval(interval);
  admin?.close();
  await browser.close();
}
