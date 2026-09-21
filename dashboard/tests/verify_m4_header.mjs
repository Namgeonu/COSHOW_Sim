import assert from 'node:assert/strict';
import path from 'node:path';
import {createRequire} from 'node:module';
const require=createRequire(import.meta.url);
const {chromium}=require(path.join(process.env.DASHBOARD_QA_NODE_MODULES,'playwright'));
const browser=await chromium.launch({executablePath:process.env.CHROMIUM_EXECUTABLE});
try {
  const page=await browser.newPage({viewport:{width:3840,height:2160}});
  await page.goto((process.env.DASHBOARD_QA_URL || 'http://127.0.0.1:8088')+'/visitor.html');
  await page.waitForSelector('.robot-panel');
  await page.evaluate(()=>document.fonts.ready);
  const result=await page.locator('.head').evaluate(head=>{
    const bounds=head.getBoundingClientRect();
    return {parent:bounds.toJSON(),children:[...head.querySelectorAll('*')].map(e=>({name:e.id||e.className||e.tagName,rect:e.getBoundingClientRect().toJSON()}))};
  });
  console.log(JSON.stringify(result));
  const r=result.parent;
  for(const {name,rect:c} of result.children) assert.ok(c.left>=r.left-1 && c.right<=r.right+1 && c.top>=r.top-1 && c.bottom<=r.bottom+1,'header descendant escapes parent: '+name);
  console.log('PASS 4K header children inside actual header');
} finally {await browser.close();}
