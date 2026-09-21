import {connect} from './ws.js';
import {present, makeNames, interests, activity, fixed, coordinate, navigationGoal} from './view-model.js';
import {cameraDecoder,cameraMessage} from './camera.js';
import {createTwin} from './twin.js';

const find = selector => document.querySelector(selector);
const root = find('.visitor');
const cameras = find('#cameras');
const fleet = find('#fleet');
const canvas = find('#field-canvas');
const text = (element, value) => { if (element.textContent !== value) element.textContent = value; };
let hello = null;
let state = null;
let connected = false;
let name = id => id;
let twin=null;
let cameraRatio=324/244;
let narrativeTimer=null;
let cameraNodes = new Map();
let robotNodes = new Map();

function node(tag, className, content) {
  const element = document.createElement(tag);
  if (className) element.className = className;
  if (content !== undefined) element.textContent = content;
  return element;
}

function metric(label, unit = '') {
  const box = node('div');
  const value = node('span', 'metric-value', '—');
  const dd = node('dd');
  dd.append(value);
  if (unit) dd.append(node('span', 'unit', unit));
  box.append(node('dt', 'label', label), dd);
  return {box, value};
}

function configure(value) {
  if(JSON.stringify(hello)===JSON.stringify(value))return;
  twin?.dispose();
  for(const refs of cameraNodes.values())refs.decoder.dispose();
  hello = value;
  // A reconnect hello must not erase the previous state during the 2s grace.
  name = makeNames(hello);
  cameraNodes = new Map();
  robotNodes = new Map();
  cameras.replaceChildren();
  fleet.replaceChildren();
  const roles = [...hello.drones, ...hello.limos];
  fleet.style.setProperty('--role-count', Math.max(1, roles.length));
  for (const id of hello.drones) {
    const tile = node('article', 'camera-tile');
    tile.dataset.robot = id;
    tile.setAttribute('aria-label', `${name(id)} 카메라`);
    const frame = node('canvas');
    frame.width=324;frame.height=244;
    frame.setAttribute('aria-label',`${name(id)} 수신 영상`);
    const title = node('div', 'camera-title');
    const badge = node('span', 'interest');
    badge.hidden = true;
    title.append(node('span', 'camera-name', name(id)), badge);
    const offline = node('span', 'camera-offline', '카메라 대기');
    tile.append(frame, title);
    if(hello.mock)tile.append(node('span','mock-badge','MOCK'));
    tile.append(offline);
    const decoder=cameraDecoder(frame,{onSize(w,h){cameraRatio=w/h;tile.style.aspectRatio=`${w}/${h}`;layout();}});
    cameras.append(tile);
    cameraNodes.set(id, {tile, badge, offline,decoder});
  }
  for (const id of roles) {
    const drone = hello.drones.includes(id);
    const panel = node('article', 'robot-panel glass');
    panel.dataset.robot = id;
    panel.dataset.kind = drone ? 'drone' : 'limo';
    const task = node('p', 'robot-task', '다음 임무 대기');
    const values = node('dl', 'robot-values');
    const first = metric(drone ? '배터리' : '위치 (m)', drone ? 'V' : '');
    const second = metric(drone ? '고도' : '목표 (m)', drone ? 'm' : '');
    if (!drone) second.box.title = '마지막 이동 목표';
    values.append(first.box, second.box);
    panel.append(node('h3', '', name(id)), node('p', 'raw-name', id), task, values);
    fleet.append(panel);
    robotNodes.set(id, {panel, task, first: first.value, second: second.value});
  }
  const areas = [hello.field?.arena, hello.field?.limo_area].filter(Boolean);
  if (areas.length && areas.every(area => area.x?.length === 2 && area.y?.length === 2)) {
    const width = Math.max(...areas.map(area => area.x[1])) - Math.min(...areas.map(area => area.x[0]));
    const height = Math.max(...areas.map(area => area.y[1])) - Math.min(...areas.map(area => area.y[0]));
    if (width > 0 && height > 0) {
      text(find('#field-size'), `필드 ${fixed(width, 0)} × ${fixed(height, 0)} m`);
    }
  }
  const idle = [...document.querySelectorAll('.idle-intro p')];
  idle.forEach((element, index) => text(element, hello.idle_lines?.[index] || ''));
  twin=createTwin(canvas,hello,{fx:new URLSearchParams(location.search).get('fx')||'',
    reducedMotion:matchMedia('(prefers-reduced-motion: reduce)').matches});
  layout();
  render();
}

function layout() {
  const stage = find('.field-stage').getBoundingClientRect();
  canvas.style.width = `${stage.width}px`;
  canvas.style.height = `${stage.height}px`;
  const overlay=find(root.dataset.mode==='idle'?'.idle-intro':'.narrative').getBoundingClientRect();
  twin?.resize(stage.width,stage.height,overlay.height);
  const count = cameraNodes.size;
  if (!count) return;
  const bounds = cameras.getBoundingClientRect();
  const gap = parseFloat(getComputedStyle(cameras).gap);
  // Two columns for the default four roles. Other configurations choose the best fit.
  let best = {width: 0, columns: 1, rows: count};
  for (let columns = 1; columns <= count; columns++) {
    const rows = Math.ceil(count / columns);
    const tileWidth = Math.min((bounds.width - gap * (columns - 1)) / columns,
      (bounds.height - gap * (rows - 1)) / rows * cameraRatio);
    if (tileWidth > best.width) best = {width: tileWidth, columns, rows};
  }
  cameras.style.gridTemplateColumns = `repeat(${best.columns}, ${best.width}px)`;
  cameras.style.gridTemplateRows = `repeat(${best.rows}, ${best.width / cameraRatio}px)`;
}

function narrative(value) {
  const current=find('#narrative');
  if(current.dataset.next===value)return;
  clearTimeout(narrativeTimer);current.dataset.next=value;
  const previous=find('#narrative-previous');
  previous.textContent=current.textContent;previous.classList.remove('fade');
  current.textContent=value;current.classList.remove('appear');
  void current.offsetWidth;
  previous.classList.add('fade');current.classList.add('appear');
  narrativeTimer=setTimeout(()=>{previous.textContent='';},200);
}

function render() {
  if (!hello) return;
  const view = present(hello, state, {connected, name});
  root.dataset.mode = view.mode;
  root.dataset.phase = view.phase || '';
  root.dataset.signal = view.signal;
  text(find('#phase-label'), view.label);
  text(find('#elapsed'), view.elapsed);
  narrative(view.sentence);
  find('.idle-intro').hidden = view.mode !== 'idle';
  find('.narrative').hidden = view.mode === 'idle';
  find('#connection-note').hidden = connected;
  const mission = state?.mission;
  const active = view.mode === 'active';
  const discovered=active&&mission?.P_N;
  text(find('#marker-label'),discovered?'발견':'임무');
  text(find('#marker-value'),discovered ? `${mission.target_id}번 (${name(mission.finder)})` :
    active && mission?.mission_marker_id!=null ? `${mission.mission_marker_id}번 마커 → ${mission.target_id}번 조난자` : '—');
  text(find('#target-value'), active ? coordinate(mission?.P_N) : '—');
  const rescue = {rescue_dispatch: '이동 중', rescue: '구조 중', return: '복귀 중', done: '구조 완료'};
  text(find('#rescue-value'), active ? rescue[view.phase] || (mission?.target_confirmed ? '위치 확인 완료' : '발견 대기') : '미션 대기');
  for (const [id, refs] of cameraNodes) {
    const robot = state?.robots?.[id];
    const ids = active ? interests(hello, state, id, robot?.detections) : [];
    text(refs.badge, ids.map(id => `${id}번`).join(' · '));
    refs.badge.hidden = !ids.length;
    const message=cameraMessage(refs.decoder.received,robot?.camera,hello.freshness_s?.camera??2,connected);
    text(refs.offline,message);refs.offline.hidden=!message;
  }
  for (const [id, refs] of robotNodes) {
    const robot = state?.robots?.[id];
    const drone = hello.drones.includes(id);
    const fresh = Number.isFinite(robot?.pose_age) && robot.pose_age <= (hello.freshness_s?.[drone ? 'pose' : 'odom'] ?? 1);
    text(refs.task, activity(hello, state, id, view));
    text(refs.first, drone ? fixed(robot?.battery_v, 2) : coordinate(fresh ? robot?.pose : null));
    // The last BT navigation command is reported; active Nav2 goal status is not.
    text(refs.second, drone ? fixed(fresh ? robot?.pose?.z : null, 2) : navigationGoal(state, id));
  }
  find('#return-progress').hidden=!(active&&['return','done'].includes(view.phase));
  twin?.update(state,view);
  layout();
}

if (new URLSearchParams(location.search).get('fx') === 'low') document.documentElement.dataset.fx = 'low';
const observer=new ResizeObserver(layout);observer.observe(root);
const disconnect = connect({
  onHello: configure,
  onState(value) {state = value; render();},
  onConnection(value) {connected = value; render();},
  onFrame(index,jpeg){const refs=cameraNodes.get(hello?.drones[index]);if(refs)refs.decoder.draw(jpeg);},
});
// Read-only diagnostics used by the local memory probe; no control commands.
window.dashboardDiagnostics=()=>({twin:twin?.stats(),cameras:[...cameraNodes.values()].map(r=>({received:r.decoder.received,dropped:r.decoder.dropped}))});
window.addEventListener('pagehide',()=>{disconnect();observer.disconnect();twin?.dispose();clearTimeout(narrativeTimer);
  for(const refs of cameraNodes.values())refs.decoder.dispose();}, {once: true});
