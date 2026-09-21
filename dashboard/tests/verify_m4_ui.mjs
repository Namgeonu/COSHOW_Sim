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
const browser = await chromium.launch({executablePath: process.env.CHROMIUM_EXECUTABLE || undefined,headless:process.env.M4_HEADED!=='1',args:['--disable-background-timer-throttling','--disable-renderer-backgrounding']});
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
    const canvas = document.querySelector('#field-canvas').getBoundingClientRect();
    const stage = document.querySelector('.field-stage').getBoundingClientRect();
    const head = document.querySelector('.head');
    const bounds = head.getBoundingClientRect();
    const headerOverflow = [...head.querySelectorAll('*')].map(rect).filter(r =>
      r.left < bounds.left - 1 || r.right > bounds.right + 1 || r.top < bounds.top - 1 || r.bottom > bounds.bottom + 1);
    return {regions, clipped, outside, headerOverflow, viewport: [innerWidth, innerHeight],
      document: [document.documentElement.scrollWidth, document.documentElement.scrollHeight],
      roles: document.querySelectorAll('.robot-panel').length,
      cameras: [...document.querySelectorAll('.camera-tile')].map(rect),
      narrativeFont: getComputedStyle(document.querySelector('#narrative')).fontSize,
      canvasRatio: canvas.width / canvas.height,stageRatio:stage.width/stage.height};
  });
  assert.deepEqual(result.clipped, [], label + ' text clipped');
  assert.deepEqual(result.outside, [], label + ' outside viewport');
  assert.deepEqual(result.document, result.viewport, label + ' document scroll');
  assert.equal(result.roles, expectedRoles);
  assert.equal(result.cameras.length, expectedCameras);
  for (const r of result.cameras) assert.ok(Math.abs(r.width / r.height - 324 / 244) < .002);
  assert.ok(Math.abs(result.canvasRatio - result.stageRatio) < .002);
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

let admin;const fixtureTimers=new Set();
try {
  await page.goto(base+route);
  await wait(()=>hello&&state,'initial protocol');
  assert.equal(hello.mock,true);
  await page.waitForFunction(()=>window.dashboardDiagnostics?.().twin?.frames>10);
  admin=new WebSocket(base.replace('http','ws')+'/ws?role=admin');
  admin.onmessage=event=>{if(typeof event.data==='string'){const value=JSON.parse(event.data);if(value.type==='state')state=value;}};
  await new Promise((resolve,reject)=>{admin.onopen=resolve;admin.onerror=reject;});
  const command=cmd=>admin.send(JSON.stringify({cmd}));
  if(state.run.state!=='IDLE'){command('reset');await wait(()=>state.run.state==='IDLE','reset');}
  await page.evaluate(()=>document.fonts.ready);
  await page.waitForFunction(()=>window.dashboardDiagnostics().cameras.every(c=>c.received));
  await geometry('idle');
  const before=await page.evaluate(()=>window.dashboardDiagnostics());
  await page.waitForTimeout(1500);
  const after=await page.evaluate(()=>window.dashboardDiagnostics());
  assert.ok(after.twin.frames>before.twin.frames);
  assert.equal(after.twin.robots,hello.drones.length+hello.limos.length);
  assert.equal(after.twin.droneTrails,hello.drones.length);
  assert.deepEqual(after.twin.spares,[]);
  assert.equal(after.twin.maxRenderFps,60);
  console.log('IDLE SCENE',JSON.stringify(after));
  await page.screenshot({path:`${images}/M4_idle_orbit.png`});
  command('preflight');await wait(()=>state.run.state==='READY','READY');command('start');
  for(const phase of ['observe','handover','search','capture','rescue_dispatch','rescue','done']){
    await wait(()=>state.mission?.phase===phase,phase,45000);
    await page.waitForSelector(`.visitor[data-phase="${phase}"]`);
    await page.waitForTimeout(250);
    await geometry(phase);
    console.log('PHASE',phase,JSON.stringify({run:state.run,mission:state.mission,diagnostics:await page.evaluate(()=>window.dashboardDiagnostics())}));
    await page.screenshot({path:`${images}/M4_${phase}.png`});
    if(phase==='capture'){
      assert.ok(state.mission.P_N);
      assert.match(await page.locator('#marker-value').textContent(),/3번 \(탐색 드론 B\)/);
      assert.notEqual(await page.locator('#target-value').textContent(),'—');
      await contrast();
    }
    if(phase==='search'){
      await page.setViewportSize({width:3840,height:2160});await page.waitForTimeout(250);
      await geometry('4K search');await page.screenshot({path:`${images}/M4_search_4k.png`});
      await page.setViewportSize({width:1920,height:1080});
    }
  }
  await page.goto(base+route+'?fx=low');
  await page.waitForFunction(()=>window.dashboardDiagnostics?.().twin?.frames>10);
  assert.equal(await page.evaluate(()=>window.dashboardDiagnostics().twin.pixelRatio),.75);
  await page.screenshot({path:`${images}/M4_fx_low.png`});
  const fixture=structuredClone(state),jpeg=await fs.readFile('dashboard/static/mock/camera.jpg');
  for(const id of hello.drones)fixture.robots[id].camera.stream_ok=false;
  await page.routeWebSocket('**/ws?role=visitor',socket=>{
    socket.send(JSON.stringify(hello));
    const send=()=>{socket.send(JSON.stringify(fixture));hello.drones.forEach((id,index)=>socket.send(Buffer.concat([Buffer.from([index]),jpeg])));};
    send();const timer=setInterval(send,100);fixtureTimers.add(timer);
    socket.onClose(async(code,reason)=>{clearInterval(timer);fixtureTimers.delete(timer);await socket.close({code,reason});});
  });
  await page.reload();
  await page.waitForFunction(()=>[...document.querySelectorAll('.camera-offline')].every(e=>!e.hidden&&e.textContent==='수신 끊김'));
  await page.screenshot({path:`${images}/M4_stream_lost.png`});
  console.log('STREAM_FALSE overlays',await page.locator('.camera-offline').allTextContents());
  // A separate page restores actual mock traffic for the sustained resource run.
  for(const timer of fixtureTimers)clearInterval(timer);fixtureTimers.clear();
  await page.close();
  assert.deepEqual(external,[]);assert.deepEqual(errors,[]);
  console.log('M4 UI PASS external requests 0; errors 0; warnings',JSON.stringify(warnings));
  const minutes=Number(process.env.M4_MEMORY_MINUTES||0);
  if(minutes>0){
    const sustained=await context.newPage();await sustained.goto(base+route);
    await sustained.waitForFunction(()=>window.dashboardDiagnostics?.().cameras.every(c=>c.received));
    const devtools=await context.newCDPSession(sustained);await devtools.send('Performance.enable');
    const start=Date.now(),samples=[];
    console.log('MEMORY_BEGIN',new Date().toISOString(),'minutes',minutes);
    for(let tick=0;Date.now()-start<=minutes*60000+1000;tick++){
      if(state.run.state==='DONE'){command('reset');}
      else if(state.run.state==='IDLE'){command('preflight');}
      else if(state.run.state==='READY'){command('start');}
      // State on the long-run page updates the same test observer.
      const metrics=await devtools.send('Performance.getMetrics');
      const sample={seconds:(Date.now()-start)/1000,stats:await sustained.evaluate(()=>window.dashboardDiagnostics()),
        heap:metrics.metrics.find(m=>m.name==='JSHeapUsedSize')?.value,
        dom:await devtools.send('Memory.getDOMCounters')};
      samples.push(sample);
      await fs.writeFile('dashboard/REPORTS/evidence/M4_memory_samples.json',JSON.stringify(samples,null,2));
      if(tick%10===0)console.log('MEMORY',JSON.stringify(sample));
      if(tick%100===0)await sustained.screenshot({path:`${images}/M4_memory_${Math.floor(sample.seconds/60)}min.png`});
      if(tick===0)sustained.on('websocket',()=>{});
      const current=await sustained.evaluate(()=>({mode:document.querySelector('.visitor').dataset.mode,phase:document.querySelector('.visitor').dataset.phase}));
      if(current.phase==='done')command('reset');else if(current.mode==='idle')command('preflight');
      await new Promise(resolve=>setTimeout(resolve,3000));
    }
    console.log('MEMORY_END',new Date().toISOString(),JSON.stringify(samples.at(-1)));
  }
}finally{
  for(const timer of fixtureTimers)clearInterval(timer);
  admin?.close();await browser.close();
}
