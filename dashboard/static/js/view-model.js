// Display policy consumes protocol data only; no ROS names or coordinate tables.
const labels = {
  waiting_poses: '준비 중', blocked_preflight: '준비 중', observe: '미션 판독',
  handover: '탐색 준비', search: '수색 중', capture: '위치 확인',
  rescue_dispatch: '구조 출동', rescue: '구조 중', return: '기지 복귀', done: '구조 완료',
};

export function makeNames(hello, warn = console.warn) {
  const warned = new Set();
  return name => {
    if (!name) return '기체';
    const label = hello?.display_names?.[name];
    if (label) return label;
    if (!warned.has(name)) {
      warned.add(name);
      warn(`표시 이름 설정 없음: ${name}`);
    }
    return name;
  };
}

export function elapsed(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return '—:—';
  const whole = Math.floor(seconds);
  return `${String(Math.floor(whole / 60)).padStart(2, '0')}:${String(whole % 60).padStart(2, '0')}`;
}

export function present(hello, state, {connected = true, name = makeNames(hello)} = {}) {
  const run = state?.run || {};
  const mission = state?.mission;
  const base = {elapsed: elapsed(run.elapsed_s), signal: 'off', phase: null,
    label: '준비 중', sentence: '다음 데모를 준비하고 있습니다'};
  if (!connected) return {...base, mode: 'offline', label: '연결 대기'};
  if (['IDLE', 'CHECKING', 'READY'].includes(run.state) && !run.external_bt)
    return {...base, mode: 'idle'};
  if (['LANDING', 'ABORTED'].includes(run.state))
    return {...base, mode: 'paused', sentence: '잠시 후 다시 시작합니다'};
  if (!mission || (run.state !== 'DONE' && (!Number.isFinite(mission.age) || mission.age > 3)))
    return {...base, mode: 'waiting'};
  const phase = mission.phase;
  const target = mission.target_id ?? '—';
  const count = Object.keys(hello?.lanes || {}).length;
  const sentences = {
    waiting_poses: base.sentence, blocked_preflight: '잠시 후 다시 시작합니다',
    observe: '판독 드론이 미션 마커를 읽고 있습니다',
    handover: '판독을 마쳤습니다. 탐색 드론이 이륙합니다',
    search: `탐색 드론 ${count}대가 ${target}번 조난자를 찾고 있습니다`,
    capture: `${name(mission.finder)}가 ${target}번 조난자를 발견했습니다. 위치를 확인합니다`,
    rescue_dispatch: '구조 차량이 조난자에게 이동합니다', rescue: '구조 중입니다',
    return: '임무 완료. 모두 기지로 복귀합니다', done: '구조 완료',
  };
  const colors = (hello?.drones || []).map(id => mission.led?.[id]);
  const signal = ['green', 'red', 'blue'].find(color => colors.includes(color)) || 'off';
  return {...base, mode: 'active', phase, label: labels[phase] || base.label,
    sentence: sentences[phase] || base.sentence, signal};
}

export function interests(hello, state, robot, ids) {
  const mission = state?.mission;
  if (!mission || !Array.isArray(ids)) return [];
  if (robot === hello.observe_drone) {
    const range = hello.mission_marker_ids;
    if (!Array.isArray(range) || range.length !== 2 || !range.every(Number.isFinite)) return [];
    return ids.filter(id => id >= range[0] && id <= range[1]);
  }
  return ids.filter(id => id === mission.target_id);
}

export function activity(hello, state, robot, view) {
  if (view.mode !== 'active') return '다음 임무 대기';
  const drone = hello.drones.includes(robot);
  const searcher = robot in (hello.lanes || {});
  const finder = robot === state.mission?.finder;
  if (view.phase === 'search') return drone ? (searcher ? '구역 수색 중' : '미션 판독 완료') : '구조 요청 대기';
  if (view.phase === 'observe') return drone ? (searcher ? '이륙 대기' : '미션 마커 판독') : '관측 임무 진행';
  if (view.phase === 'capture') return finder ? '목표 위치 확인' : '구조 준비';
  if (view.phase === 'done') return '임무 완료';
  return labels[view.phase] || '임무 진행';
}

export function fixed(value, digits = 1) {
  return Number.isFinite(value) ? value.toFixed(digits).replace('-', '−') : '—';
}

export function coordinate(point) {
  if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) return '—';
  return `${fixed(point.x)}, ${fixed(point.y)}`;
}

export function navigationGoal(state, robot) {
  const command = state?.mission?.cmd?.[robot];
  if (command?.kind !== 'nav' || !Array.isArray(command.goal)) return '—';
  return coordinate({x: command.goal[0], y: command.goal[1]});
}
