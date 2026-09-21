// Supplemental design investigation, not an acceptance claim for hypothetical placements.
import {createRequire} from 'node:module';
import path from 'node:path';
const require = createRequire(import.meta.url);
const modules = process.env.DASHBOARD_QA_NODE_MODULES;
const {chromium} = require(modules ? path.join(modules, 'playwright') : 'playwright');
const sharp = require(modules ? path.join(modules, 'sharp') : 'sharp');
const browser = await chromium.launch({executablePath: process.env.CHROMIUM_EXECUTABLE || undefined});
try {
  const page = await browser.newPage({viewport: {width: 1920, height: 1080}, deviceScaleFactor: 1});
  await page.goto((process.env.DASHBOARD_QA_URL || 'http://127.0.0.1:8088') + '/visitor.html');
  await page.addStyleTag({content: '.visitor {visibility:hidden}'});
  await page.evaluate(() => {
    for (const side of ['cool', 'warm']) {
      const element = document.createElement('div');
      element.className = 'glass'; element.id = side;
      Object.assign(element.style, {position: 'fixed', width: '240px', height: '120px', zIndex: '1000',
        ...(side === 'cool' ? {left: '0', top: '0'} : {right: '0', bottom: '0'})});
      document.body.append(element);
    }
  });
  const lum = rgb => rgb.map(v => v / 255).map(v => v <= .04045 ? v / 12.92 : ((v + .055) / 1.055) ** 2.4)
    .reduce((sum, v, i) => sum + v * [.2126, .7152, .0722][i], 0);
  console.log('EXPERIMENT: temporary 240x120 glass panels at background light corners; actual dashboard hidden. Sample interior >=16px from edges. This is NOT the shipped label layout.');
  for (const strong of [false, true]) {
    await page.evaluate(value => ['cool', 'warm'].forEach(id => document.getElementById(id).classList.toggle('glass-strong', value)), strong);
    const {data, info} = await sharp(await page.screenshot()).removeAlpha().raw().toBuffer({resolveWithObject: true});
    for (const [light, left, top] of [['cool', 0, 0], ['warm', 1680, 960]]) {
      let minimum = Infinity, sample;
      for (let y = top + 16; y < top + 104; y++) for (let x = left + 16; x < left + 224; x++) {
        const offset = (y * info.width + x) * info.channels;
        const bg = [...data.subarray(offset, offset + 3)];
        const fg = bg.map((v, i) => [244, 246, 251][i] * .72 + v * .28);
        const ratio = (lum(fg) + .05) / (lum(bg) + .05);
        if (ratio < minimum) {minimum = ratio; sample = {x, y, background: bg};}
      }
      console.log(JSON.stringify({light, glassAlpha: strong ? .16 : .1, minimum: +minimum.toFixed(3), ...sample,
        meets45: minimum >= 4.5}));
    }
  }
} finally {await browser.close();}
