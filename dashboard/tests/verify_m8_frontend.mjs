import assert from 'node:assert/strict';
import path from 'node:path';
import {createRequire} from 'node:module';
const require=createRequire(import.meta.url);
const {chromium}=require(path.join(process.env.DASHBOARD_QA_NODE_MODULES,'playwright'));
const base=process.env.DASHBOARD_QA_URL||'http://127.0.0.1:8089';
const browser=await chromium.launch({executablePath:process.env.CHROMIUM_EXECUTABLE});
const timers=new Set(),errors=[],external=[],failures=[];
function check(label,body){try{body();console.log('PASS',label);}catch(error){failures.push(label+': '+error.message);console.error('FAIL',label,error.message);}}
try {
  const context=await browser.newContext({viewport:{width:1600,height:1050}}),page=await context.newPage();
  await context.route('**/*',async route=>{if(new URL(route.request().url()).origin!==base){external.push(route.request().url());await route.abort();}else await route.continue();});
  page.on('pageerror',error=>errors.push(error.message));
  let hello,snapshot;
  page.on('websocket',socket=>socket.on('framereceived',({payload})=>{if(typeof payload==='string'){const value=JSON.parse(payload);if(value.type==='hello')hello=value;if(value.type==='state')snapshot=value;}}));
  await page.goto(base+'/admin.html');await page.waitForFunction(()=>document.querySelectorAll('#robot-rows tr').length===6);
  const fixture=structuredClone(snapshot);fixture.run.state='RUNNING';fixture.events=[];
  const spare=Object.keys(fixture.robots).find(id=>fixture.robots[id].role==null);
  assert.ok(spare,'mock supplies real spare inventory metadata');
  fixture.checklist=fixture.checklist.filter(row=>row.group!==spare);
  for(const [kind,status] of [['battery','warning'],['link','warning'],['ping','warning'],['pose','skipped'],['healthy','pass']]) {
    fixture.checklist.push({id:`${spare}.m8_${kind}`,group:spare,label:`M8 ${kind}`,status,ok:status==='pass',blocking:false,detail:`M8 ${kind} evidence`});
  }
  fixture.events=Array.from({length:30},(_,i)=>({t:100+i,level:'info',text:`M8 event ${i}`}));
  let transmitting=true,ws;
  await page.routeWebSocket('**/ws?role=admin',socket=>{
    ws=socket;let welcomed=false;
    const push=()=>{if(transmitting){if(!welcomed){socket.send(JSON.stringify(hello));welcomed=true;}socket.send(JSON.stringify(fixture));}};
    push();const timer=setInterval(push,100);timers.add(timer);
    socket.onMessage(()=>{});socket.onClose(()=>{clearInterval(timer);timers.delete(timer);});
  });
  await page.reload();await page.waitForFunction(()=>document.querySelector('#run-state').textContent==='RUNNING');
  const warningIds=await page.locator('#spare-warnings [data-check]').evaluateAll(rows=>rows.map(row=>row.dataset.check));
  check('B6 only warning spare checklist rows are visible below failures',()=>{
    assert.deepEqual(warningIds.filter(id=>id.startsWith(spare+'.m8_')),[`${spare}.m8_battery`,`${spare}.m8_link`,`${spare}.m8_ping`]);
  });
  const roleCount=await page.locator('#robot-rows tr').count(),inventoryOpen=await page.locator('#fleet-editor').getAttribute('open');
  check('B6 role table remains six rows and full inventory stays collapsed',()=>{assert.equal(roleCount,6);assert.equal(inventoryOpen,null);});
  if(warningIds.length)await page.locator('#spare-warnings').scrollIntoViewIfNeeded();
  if(warningIds.length)await page.locator('#spare-warnings').screenshot({path:'dashboard/REPORTS/img/M8_spare_warnings.png'});
  await page.locator('[data-cmd="estop"]').click();await page.waitForTimeout(1050);
  assert.match(await page.locator('#command-response').textContent(),/재전송/);
  transmitting=false;await ws.close({code:1001,reason:'M8 disconnect after ACK timeout'});
  await page.waitForFunction(()=>!document.querySelector('#connection-banner').hidden);
  const disconnected=await page.locator('#command-response').textContent();
  check('C4 disconnect replaces stale ACK retry with unavailable message',()=>assert.equal(disconnected,'서버 연결 없음 — 명령을 보낼 수 없습니다'));
  await page.evaluate(()=>window.scrollTo(0,document.documentElement.scrollHeight));await page.waitForTimeout(100);
  const bounds=await page.evaluate(()=>{const rect=id=>{const r=document.querySelector(id).getBoundingClientRect();return {top:r.top,bottom:r.bottom,left:r.left,right:r.right};};return {banner:rect('#connection-banner'),control:rect('.control-bar'),emergency:rect('[data-cmd="estop"]'),height:innerHeight,scroll:scrollY};});
  check('C3 scrolled disconnect banner never covers emergency control',()=>{assert.ok(bounds.scroll>0);assert.ok(bounds.control.top>=bounds.banner.bottom);assert.ok(bounds.emergency.top>=bounds.banner.bottom&&bounds.emergency.bottom<=bounds.height);});
  console.log('C3 BOUNDS',JSON.stringify(bounds));
  await page.screenshot({path:'dashboard/REPORTS/img/M8_admin_disconnected_scrolled.png'});
  transmitting=true;await page.reload();await page.waitForFunction(()=>document.querySelector('#run-state').textContent==='RUNNING');
  await page.locator('[data-cmd="estop"]').click();transmitting=false;await ws.close({code:1001,reason:'M8 disconnect before ACK timeout'});
  await page.waitForFunction(()=>!document.querySelector('#connection-banner').hidden);await page.waitForTimeout(1200);
  const early=await page.locator('#command-response').textContent();
  check('C4 pending ACK timer cannot overwrite disconnect after one second',()=>assert.equal(early,'서버 연결 없음 — 명령을 보낼 수 없습니다'));
  transmitting=true;await page.waitForFunction(()=>document.querySelector('#connection-banner').hidden&&!document.querySelector('[data-cmd="reset"]').disabled);
  const reconnectedControls=await page.locator('[data-cmd]:enabled').evaluateAll(rows=>rows.map(row=>row.dataset.cmd));
  check('C4 reconnect discards pending command and unlocks normal controls',()=>assert.deepEqual(reconnectedControls,['reset','estop']));

  const visitor=await context.newPage();visitor.on('pageerror',error=>errors.push(error.message));
  const visual=structuredClone(fixture);visual.run={...visual.run,state:'RUNNING',elapsed_s:50};
  visual.mission={...visual.mission,age:0,phase:'capture',target_id:3,finder:hello.drones[2],P_N:{x:0,y:.6,z:.8},led:{[hello.drones[2]]:'red'}};
  const jpeg=await (await page.request.get(base+'/static/mock/camera.jpg')).body();
  await visitor.routeWebSocket('**/ws?role=visitor',socket=>{
    socket.send(JSON.stringify(hello));
    const push=()=>{socket.send(JSON.stringify(visual));for(let index=0;index<hello.drones.length;index++)socket.send(Buffer.concat([Buffer.from([index]),jpeg]));};
    push();const timer=setInterval(push,100);timers.add(timer);socket.onClose(()=>{clearInterval(timer);timers.delete(timer);});
  });
  await visitor.goto(base+'/visitor.html?fx=low');await visitor.waitForFunction(()=>document.documentElement.dataset.fx==='low'&&document.querySelectorAll('.robot-panel').length===6);
  await visitor.evaluate(()=>document.fonts.ready);await visitor.waitForTimeout(500);
  const surfaces=await visitor.locator('.glass').evaluateAll(rows=>rows.map(row=>({className:row.className,background:getComputedStyle(row).backgroundColor,backdrop:getComputedStyle(row).backdropFilter})));
  check('C1 low effects retains distinct panel surfaces and opaque narrative',()=>{
    assert.ok(surfaces.length>2);
    for(const surface of surfaces){assert.equal(surface.backdrop,'none');assert.equal(surface.background,/narrative|idle-intro/.test(surface.className)?'rgba(14, 21, 48, 0.78)':'rgba(255, 255, 255, 0.08)');}
  });
  console.log('C1 SURFACES',JSON.stringify(surfaces));
  await visitor.screenshot({path:'dashboard/REPORTS/img/M8_fx_low.png'});
  check('no external requests or browser errors',()=>{assert.deepEqual(errors,[]);assert.deepEqual(external,[]);});
  assert.deepEqual(failures,[]);
} finally {for(const timer of timers)clearInterval(timer);await browser.close();}
