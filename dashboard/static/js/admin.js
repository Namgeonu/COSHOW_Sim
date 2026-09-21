import {connect} from './ws.js';
import {createFleetUI} from './fleet-ui.js';
import {buttons,commandReply,landingText,checklistRows} from './admin-model.js';
import {elapsed,makeNames,fixed,coordinate} from './view-model.js';
const $=selector=>document.querySelector(selector),actions=[...document.querySelectorAll('[data-cmd]')];
const statusLabels={pass:'통과',fail:'실패',warning:'주의',pending:'진행 중',skipped:'제외',unavailable:'정보 없음'};
const cameraSizes=new Map();let cameraEpoch=0;
let hello,state,connected=false,pending=null,ackTimer=null,holdTimer=null,name=id=>id,settings={};
const element=(tag,value,className)=>{const node=document.createElement(tag);if(value!=null)node.textContent=value;if(className)node.className=className;return node;};
const set=(selector,value)=>{const node=$(selector);if(node.textContent!==value)node.textContent=value;};
function bannerOffset(){document.body.style.setProperty('--connection-banner-height',`${$('#connection-banner').getBoundingClientRect().height}px`);}
const bannerObserver=new ResizeObserver(bannerOffset);bannerObserver.observe($('#connection-banner'));bannerOffset();
function controls(){
  const enabled=buttons(state,connected,pending?{elapsed:performance.now()-pending.since}:null);
  for(const button of actions)button.disabled=!enabled[button.dataset.cmd];
  document.body.classList.toggle('landing',state?.run?.state==='LANDING');
  if(!connected)set('#command-response','서버 연결 없음 — 명령을 보낼 수 없습니다');
  else if(state?.run?.state==='LANDING')set('#command-response',landingText(state));
}
function send(cmd){
  if(!buttons(state,connected,pending?{elapsed:performance.now()-pending.since}:null)[cmd])return;
  if(!connection.send(cmd))return;
  clearTimeout(ackTimer);
  pending={cmd,since:performance.now(),seen:new Set((state?.events||[]).map(e=>`${e.t}|${e.text}`))};
  set('#command-response','전송 중');controls();
  ackTimer=setTimeout(()=>{if(connected&&pending){set('#command-response','응답 대기 중 · 비상 착륙은 재전송할 수 있습니다');controls();}},1000);
}
for(const button of actions){
  const cmd=button.dataset.cmd;
  if(cmd!=='start'){button.addEventListener('click',()=>send(cmd));continue;}
  const cancel=()=>{clearTimeout(holdTimer);holdTimer=null;button.classList.remove('holding');};
  const begin=()=>{if(button.disabled||holdTimer)return;button.classList.add('holding');holdTimer=setTimeout(()=>{holdTimer=null;button.classList.remove('holding');send('start');},600);};
  button.addEventListener('pointerdown',event=>{if(event.button!==0)return;button.setPointerCapture(event.pointerId);begin();});
  for(const type of ['pointerup','pointercancel','lostpointercapture','blur'])button.addEventListener(type,cancel);
  button.addEventListener('keydown',event=>{if([' ','Enter'].includes(event.key)){event.preventDefault();if(!event.repeat)begin();}});
  button.addEventListener('keyup',cancel);
}
function checkRow(row){
  const line=element('div',null,'check-row');line.dataset.check=row.id;
  line.append(element('span',statusLabels[row.status]||'정보 없음','status-'+row.status),element('span',`${state?.robots?.[row.group]?name(row.group)+' · ':''}${row.label}`),element('p',row.detail));return line;
}
let checklistSignature='';
function checklist(){
  const {rows,spareWarnings}=checklistRows(state),signature=JSON.stringify([rows,spareWarnings].map(group=>group.map(({id,status,ok,detail})=>({id,status,ok,detail}))));
  if(signature===checklistSignature)return;checklistSignature=signature;
  const open=new Set([...$('#checklist').querySelectorAll('details[open]')].map(e=>e.dataset.group));
  const failures=rows.filter(r=>r.blocking&&!r.ok&&r.status!=='pending');
  $('#failures').replaceChildren(...failures.map(checkRow));
  $('#spare-warnings').hidden=spareWarnings.length===0;
  $('#spare-warning-rows').replaceChildren(...spareWarnings.map(checkRow));
  const groups=new Map();for(const row of rows){if(failures.includes(row))continue;if(!groups.has(row.group))groups.set(row.group,[]);groups.get(row.group).push(row);}
  const nodes=[];
  for(const [group,items] of groups){const box=element('details');box.dataset.group=group;box.open=open.has(group);
    box.append(element('summary',`${hello.display_names?.[group]||group} · ${items.filter(r=>r.ok).length}/${items.length}`),...items.map(checkRow));nodes.push(box);}
  $('#checklist').replaceChildren(...nodes);
  set('#check-summary',`${rows.filter(r=>r.blocking&&r.ok).length}/${rows.filter(r=>r.blocking).length} 필수 통과`);
}
function robotRows(){
  const rows=[];
  for(const id of [...hello.drones,...hello.limos]){
    const robot=state.robots?.[id]||{},drone=hello.drones.includes(id),row=element('tr');row.dataset.robot=id;
    const title=element('td',name(id));title.append(element('small',`${id} · ${robot.fleet_id||'미배정'}`));
    const ping=element('td',robot.ping?.ip||'정보 없음');ping.append(element('small',robot.ping?.ok===true?`${fixed(robot.ping.rtt_ms,1)} ms`:robot.ping?.ok===false?'응답 없음':'정보 없음'));
    const age=element('td',`${drone?fixed(robot.status_age,2):'—'} / ${fixed(robot.pose_age,2)} s`);
    const camera=element('td',drone?`${fixed(robot.camera?.fps,1)} fps`:'—');
    if(drone)camera.append(element('small',cameraSizes.get(id)||'해상도 대기'));
    const armed=element('td',robot.armed===true?'무장':robot.armed===false?'해제':'—',robot.armed===true?'armed':'');
    const fresh=Number.isFinite(robot.pose_age)&&robot.pose_age<(hello.freshness_s[drone?'pose':'odom']);
    const z=element('td',`${fixed(robot.pose?.z,2)} m`,fresh&&Number.isFinite(settings.landed_z)&&robot.pose?.z>settings.landed_z?'airborne':fresh?'':'stale');
    row.append(title,ping,age,element('td',`${fixed(robot.battery_v,2)} V`),camera,armed,z);rows.push(row);
  }
  $('#robot-rows').replaceChildren(...rows);
}
let lastEvents='';
function render(){
  if(!state||!hello)return;
  $('#external-bt').hidden=!state.run.external_bt;
  set('#run-state',state.run.state);set('#run-elapsed',elapsed(state.run.elapsed_s));
  set('#start-note',`무장된 드론 ${hello.drones.length}대가 이륙합니다`);
  const reply=commandReply(state.events||[],pending);
  if(reply){pending=null;clearTimeout(ackTimer);set('#command-response',reply.text);}
  controls();checklist();robotRows();fleetUI.update(hello,state,connected);
  const stages=state.preflight?.stages||[];
  $('#preflight-stages').replaceChildren(...['서버 준비','추정기 초기화','안전 관문','무장'].map((label,index)=>{
    const result=stages[index]?.result||'pending';return element('li',`${label} · ${statusLabels[result]||result}`,'status-'+result);
  }));
  set('#last-error',state.run.last_error||state.preflight?.abort_reason||'');
  const signature=JSON.stringify(state.events||[]);
  if(signature!==lastEvents){lastEvents=signature;$('#events').replaceChildren(...[...(state.events||[])].reverse().map(event=>{
    const row=element('li');row.dataset.level=event.level;row.append(element('time',new Date(event.t*1000).toLocaleTimeString()),element('span',event.text));return row;}));}
}
const fleetUI=createFleetUI({onSettings(value){settings=value;render();}});
const connection=connect({role:'admin',onHello(value){hello=value;cameraEpoch++;cameraSizes.clear();name=makeNames(value);set('#mock-state',value.mock?'MOCK':'');fleetUI.load();render();},
  onFrame(index,jpeg){
    const id=hello?.drones[index],epoch=cameraEpoch;if(!id||cameraSizes.has(id))return;cameraSizes.set(id,'해상도 확인 중');
    createImageBitmap(new Blob([jpeg],{type:'image/jpeg'})).then(bitmap=>{try{if(epoch===cameraEpoch)cameraSizes.set(id,`${bitmap.width}×${bitmap.height}`);}finally{bitmap.close();}}).catch(()=>{if(epoch===cameraEpoch)cameraSizes.delete(id);});
  },
  onState(value){state=value;set('#last-received',`마지막 수신 ${new Date().toLocaleTimeString()}`);render();},
  onConnection(value){
    connected=value;$('#connection-banner').hidden=value;bannerOffset();
    if(!value){pending=null;clearTimeout(ackTimer);ackTimer=null;clearTimeout(holdTimer);holdTimer=null;for(const button of actions)button.classList.remove('holding');}
    controls();fleetUI.update(hello,state,connected);if(value&&!pending)set('#command-response','서버 연결됨');
  }});
window.addEventListener('pagehide',()=>{connection();cameraEpoch++;clearTimeout(ackTimer);clearTimeout(holdTimer);bannerObserver.disconnect();},{once:true});
