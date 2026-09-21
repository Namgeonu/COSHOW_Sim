import {connect} from './ws.js';
import {present, interests, makeNames, fixed, coordinate} from './view-model.js';
import {cameraDecoder, cameraMessage} from './camera.js';
import {createTwin} from './twin.js';

const find = s => document.querySelector(s);
const root = find('.visitor');
const camsEl = find('#cams');
const dronesEl = find('#drones');
const canvas = find('#field-canvas');
const track = find('#track');
const reel = find('.reel');
const popup = find('#popup'), popupText = find('#popup-text');
const text = (el, v) => { if (el && el.textContent !== v) el.textContent = v; };

let hello = null, state = null, connected = false, name = id => id, twin = null;
let cameraNodes = new Map(), droneNodes = new Map(), steps = [];
let lastPhase = '__init__', popupTimer = null;

const SLOT = 132;
const STAGES = ['구출자 위치 확인', '구출자 탐색', '요구조자 구출', '복귀'];
const PHASE_STAGE = {observe:0, handover:0, search:1, capture:1, rescue_dispatch:1, rescue:2, return:3, done:3};

function node(tag, cls, content) {
  const el = document.createElement(tag);
  if (cls) el.className = cls;
  if (content !== undefined) el.textContent = content;
  return el;
}
// 복귀 국면은 흰색 신호로 강조 (LED 에는 없는 색)
function signalOf(view) { return ['return', 'done'].includes(view.phase) ? 'white' : view.signal; }

function carouselSub(phase, mission) {
  const finder = name(mission?.finder), obs = name(hello?.observe_drone);
  return ({
    observe: `${obs}가 구조 목표를 확인 중입니다.`,
    handover: '판독을 마쳤습니다. 탐색 드론이 이륙합니다.',
    search: '드론들이 목표를 탐색 중입니다.',
    capture: `${finder}가 구출자를 발견했습니다.`,
    rescue_dispatch: '구출 LIMO가 출동 중입니다.',
    rescue: '요구조자를 구출 중입니다.',
    return: '요구조자와 함께 복귀합니다.', done: '요구조자와 함께 복귀합니다.',
  })[phase] || '다음 데모를 준비하고 있습니다';
}
function popupFor(phase, mission) {
  const finder = name(mission?.finder), obs = name(hello?.observe_drone);
  const limoA = name((hello?.limos || [])[0]) || 'LIMO A';
  return ({
    observe: `${obs}이 구출 목표를 확인 중입니다.`,
    handover: '구출자를 탐색합니다.',
    search: '구출자를 탐색합니다.',
    capture: `구출자를 발견하였습니다!! (구출자 위치 : ${coordinate(mission?.P_N)})`,
    rescue_dispatch: '리모가 구출자 위치로 출동합니다.',
    rescue: '구출 중입니다.',
    return: '요구조자를 구출한 후 복귀 중입니다.',
  })[phase] || null;
}

// 각 드론의 현재 상태(칩). active 국면에서 pose 가 오래 끊기면 '이상'.
function droneStatus(id, view) {
  if (view.mode !== 'active') return '대기';
  const robot = state?.robots?.[id];
  const threshold = hello?.freshness_s?.pose ?? 1;
  const fresh = Number.isFinite(robot?.pose_age) && robot.pose_age <= threshold && robot?.pose;
  if (!fresh) return '이상';
  const phase = view.phase, z = robot?.pose?.z ?? 0, air = z > 0.25;
  const isObs = id === hello?.observe_drone, isFinder = id === state?.mission?.finder;
  if (['return', 'done'].includes(phase)) return air ? '복귀' : '대기';
  if (phase === 'rescue') return isFinder ? '구출' : '대기';
  if (phase === 'rescue_dispatch' || phase === 'capture') return isFinder ? '구출 대기' : (air ? '복귀' : '대기');
  if (phase === 'search') return air ? '탐색' : '대기';
  if (phase === 'observe') return isObs ? (air ? '탐색' : '이륙') : '대기';
  if (phase === 'handover') return isObs ? '복귀' : (air ? '이륙' : '대기');
  return '대기';
}

function configure(value) {
  if (JSON.stringify(hello) === JSON.stringify(value)) return;
  twin?.dispose();
  for (const r of cameraNodes.values()) r.decoder.dispose();
  hello = value;
  name = makeNames(hello);
  cameraNodes = new Map(); droneNodes = new Map();
  camsEl.replaceChildren(); dronesEl.replaceChildren(); track.replaceChildren();

  const drones = hello.drones || [];
  dronesEl.style.setProperty('--drone-count', Math.max(1, drones.length));
  drones.forEach((id, i) => {
    // 카메라 타일
    const cam = node('article', 'cam'); cam.dataset.robot = id;
    const frame = node('canvas'); frame.width = 324; frame.height = 244;
    frame.setAttribute('aria-label', `${name(id)} 수신 영상`);
    const tag = node('div', 'tag');
    tag.append(node('span', 'cam-name', `드론 ${'ABCD'[i] || i + 1} · ${id}`), node('span', 'see', '마커 검출'));
    const err = node('div', 'err');
    err.append(node('span', 'mark', '⚠ 신호 없음'), node('span', 'msg', `${id} 영상 두절 · 이상 판정`));
    cam.append(frame, tag, err, node('span', 'foot', '영상 검출 화면'));
    if (hello.mock) cam.append(node('span', 'mock-badge', 'MOCK'));
    const decoder = cameraDecoder(frame, {onSize(w, h) { cam.style.aspectRatio = `${w}/${h}`; }});
    camsEl.append(cam);
    cameraNodes.set(id, {cam, see: tag.querySelector('.see'), errMsg: err.querySelector('.msg'), decoder});

    // 드론 상태 카드
    const card = node('article', 'drone glass'); card.dataset.robot = id;
    card.innerHTML = `<h3>${id}</h3><span class="chip" data-st="대기">대기</span>
      <div class="bars">
        <div class="bar"><span class="ic">배터리</span><span class="segs" data-lv="0"><i></i><i></i><i></i></span></div>
        <div class="bar"><span class="ic">고도</span><span class="alt"><i></i></span></div>
      </div>`;
    dronesEl.append(card);
    droneNodes.set(id, {card, chip: card.querySelector('.chip'), segs: card.querySelector('.segs'), alt: card.querySelector('.alt i')});
  });

  // 미션 캐러셀 4단계
  STAGES.forEach((title, i) => {
    const step = node('div', 'step');
    step.innerHTML = `<p class="big"><span class="idx">${i + 1}</span>${title}</p><p class="sub"></p>`;
    track.append(step);
  });
  steps = [...track.children];

  // 필드 크기 라벨
  const areas = [hello.field?.arena, hello.field?.limo_area].filter(a => a?.x?.length === 2 && a?.y?.length === 2);
  if (areas.length) {
    const w = Math.max(...areas.map(a => a.x[1])) - Math.min(...areas.map(a => a.x[0]));
    const h = Math.max(...areas.map(a => a.y[1])) - Math.min(...areas.map(a => a.y[0]));
    if (w > 0 && h > 0) text(find('#field-size'), `${fixed(w, 0)} × ${fixed(h, 0)} m · LIMO 대기선 왼쪽`);
  }
  lastPhase = '__init__';
  twin = createTwin(canvas, hello, {fx: new URLSearchParams(location.search).get('fx') || '',
    reducedMotion: matchMedia('(prefers-reduced-motion: reduce)').matches});
  layout(); render();
}

function layout() {
  const stage = find('.field-stage').getBoundingClientRect();
  canvas.style.width = `${stage.width}px`; canvas.style.height = `${stage.height}px`;
  twin?.resize(stage.width, stage.height, 0);
  recenter();
}
function recenter() {
  const view = present(hello, state, {connected, name});
  const stage = view.mode === 'active' ? Math.max(0, PHASE_STAGE[view.phase] ?? 0) : 0;
  track.style.transform = `translateY(${(reel.clientHeight - SLOT) / 2 - stage * SLOT}px)`;
}
function showPopup(msg, sig) {
  if (!msg) return;
  text(popupText, msg); popup.dataset.sig = sig;
  popup.classList.remove('show'); void popup.offsetWidth; popup.classList.add('show');
  clearTimeout(popupTimer); popupTimer = setTimeout(() => popup.classList.remove('show'), 5000);
}

function render() {
  if (!hello) return;
  const view = present(hello, state, {connected, name});
  const sig = signalOf(view);
  root.dataset.mode = view.mode; root.dataset.phase = view.phase || ''; root.dataset.signal = sig;
  text(find('#phase-label'), view.label);
  text(find('#elapsed'), view.elapsed);
  find('#connection-note').hidden = connected;
  const mission = state?.mission, active = view.mode === 'active';

  // 팝업: 국면이 바뀔 때 1회 (5초)
  const phaseKey = active ? view.phase : (view.mode);
  if (phaseKey !== lastPhase) { lastPhase = phaseKey; if (active) showPopup(popupFor(view.phase, mission), sig); }

  // 캐러셀
  const stage = active ? Math.max(0, PHASE_STAGE[view.phase] ?? 0) : 0;
  track.dataset.sig = sig;
  text(find('#narr-stage'), active ? `${stage + 1} / ${STAGES.length}` : '대기');
  steps.forEach((el, i) => {
    el.classList.toggle('cur', i === stage);
    if (i === stage) text(el.querySelector('.sub'), active ? carouselSub(view.phase, mission) : '다음 데모를 준비하고 있습니다');
  });
  track.style.transform = `translateY(${(reel.clientHeight - SLOT) / 2 - stage * SLOT}px)`;

  // 카메라
  for (const [id, r] of cameraNodes) {
    const robot = state?.robots?.[id];
    const ids = active ? interests(hello, state, id, robot?.detections) : [];
    const offline = cameraMessage(r.decoder.received, robot?.camera, hello.freshness_s?.camera ?? 2, connected);
    const dead = active && (droneStatus(id, view) === '이상' || !!offline);
    r.cam.classList.toggle('dead', dead);
    r.cam.classList.toggle('seeing', ids.length > 0 && !dead);   // '마커 검출' 배지 표시
    if (dead) text(r.errMsg, offline || `${id} 영상 두절 · 이상 판정`);
  }

  // 드론 카드
  for (const [id, r] of droneNodes) {
    const robot = state?.robots?.[id];
    const st = droneStatus(id, view);
    r.chip.dataset.st = st; r.chip.textContent = st;
    r.card.dataset.fx = active ? (mission?.led?.[id] || (st === '복귀' ? 'white' : 'off')) : 'off';
    const threshold = hello.freshness_s?.pose ?? 1;
    const fresh = Number.isFinite(robot?.pose_age) && robot.pose_age <= threshold && robot?.pose;
    const v = robot?.battery_v;
    r.segs.dataset.lv = !Number.isFinite(v) ? 0 : v >= 3.85 ? 3 : v >= 3.70 ? 2 : 1;
    const z = fresh ? (robot.pose.z ?? 0) : 0;
    r.alt.style.width = `${Math.max(0, Math.min(100, z / 1.5 * 100))}%`;
  }

  twin?.update(state, view);
}

if (new URLSearchParams(location.search).get('fx') === 'low') document.documentElement.dataset.fx = 'low';
const observer = new ResizeObserver(layout); observer.observe(root);
const disconnect = connect({
  onHello: configure,
  onState(v) { state = v; render(); },
  onConnection(v) { connected = v; render(); },
  onFrame(index, jpeg) { const r = cameraNodes.get(hello?.drones?.[index]); if (r) r.decoder.draw(jpeg); },
});
window.dashboardDiagnostics = () => ({twin: twin?.stats(), cameras: [...cameraNodes.values()].map(r => ({received: r.decoder.received, dropped: r.decoder.dropped}))});
window.addEventListener('pagehide', () => { disconnect(); observer.disconnect(); twin?.dispose(); clearTimeout(popupTimer);
  for (const r of cameraNodes.values()) r.decoder.dispose(); }, {once: true});
