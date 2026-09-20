"""BT 시작 전 사전점검 게이트 — 점검표 브리핑 + y/n 승인.

main.py 가 로스터 확정 직후, BTRunner import 전에 부른다. 임시 ROS 노드로
gate_window_sec 동안 수집한 뒤 기체×항목 점검표를 출력하고 승인을 받는다.

  왜 BT 밖인가: 트리 안에서 input() 을 받으면 tick/ROS 콜백이 멈추는 데다,
  쏟아지는 로그가 프롬프트 줄을 덮어 실전에서 읽을 수 없다. 로스터 프롬프트와
  같은 단계(조용한 화면)에서 처리한다. in-BT PreflightReady 는 그대로 남아
  승인 시점과 이륙 시점 사이의 상태 변화를 잡는 마지막 안전망 역할을 한다.

  점검 항목 (PreflightReady 와 같은 기준):
    pose    수신 여부 + 신선도 + 슬롯 base 대비 축별 오차 <= pose_tolerance
    칼만    최근 kalman_history_len 샘플의 축별 max-min < kalman_threshold   (required 일 때만)
    상태    /status 신선도 + IS_TUMBLED/IS_LOCKED/PM_SHUTDOWN 아님            (required 일 때만)
    카메라  marker_detections 수신 프레임 수 (검출 노드는 실제 프레임을 받았을 때만
            발행하므로 발행 수 = 영상이 흐른 프레임 수)

  승인의 의미: 문제 기체를 안고 y 를 하면 그 기체 목록이 config['coshow']['degraded'] 로
  BT 에 넘어가고, health 가 t=0 부터 그 기체를 DEGRADED 로 확정해 유예 없이 재배치가
  발동한다 (미션기 결함 → 탐색기 승계 / 탐색기 결함 → 미션기 투입·구역 재배분).
  "무시하고 정상인 척 진행" 이 아니다.

  비대화형(자동 실행): 문제 없으면 자동 진행, 있으면 중단 (안전 기본값).
"""
import sys
import time

_ITEMS = ('pose', '칼만', '초기위치', '상태', '카메라')


def _role_label(c, drone):
    slot = (c.get('slot_of') or {}).get(drone, '')
    if slot == 'mission':
        return '미션'
    if slot.startswith('search_'):
        return '탐색' + slot.split('_', 1)[1]
    return slot or '?'


class _Collector:
    """gate_window_sec 동안의 토픽 수집기. 판정은 끝난 뒤 한 번에 한다."""

    def __init__(self, node, drones, required):
        from geometry_msgs.msg import PoseStamped
        from coshow_interfaces.msg import MarkerDetections
        self.pose = {}          # drone -> (msg 위치 튜플, 수신 시각)
        self.frames = {d: 0 for d in drones}
        self.frame_t = {}
        self.kal = {d: [] for d in drones}          # [(vx,vy,vz), ...]
        self.status = {}        # drone -> (supervisor_info, pm_state, 수신 시각)
        for d in drones:
            node.create_subscription(PoseStamped, f'/{d}/pose',
                                     lambda m, r=d: self._on_pose(r, m), 10)
            node.create_subscription(MarkerDetections, f'/{d}/marker_detections',
                                     lambda m, r=d: self._on_det(r, m), 10)
        if required:
            # crazyswarm2 전용 토픽 — 시뮬(required=false)에는 없다
            from crazyflie_interfaces.msg import LogDataGeneric, Status as CfStatus
            self._CfStatus = CfStatus
            for d in drones:
                node.create_subscription(LogDataGeneric, f'/{d}/kalman_variance',
                                         lambda m, r=d: self._on_kal(r, m), 10)
                node.create_subscription(CfStatus, f'/{d}/status',
                                         lambda m, r=d: self._on_status(r, m), 10)

    def _on_pose(self, d, m):
        p = m.pose.position
        self.pose[d] = ((p.x, p.y, p.z), time.monotonic())

    def _on_det(self, d, m):
        self.frames[d] += 1
        self.frame_t[d] = time.monotonic()

    def _on_kal(self, d, m):
        if len(m.values) >= 3:
            self.kal[d].append(tuple(float(v) for v in m.values[:3]))

    def _on_status(self, d, m):
        self.status[d] = (int(m.supervisor_info), int(m.pm_state), time.monotonic())


def _judge(col, c, pf, drones, required, t_end):
    """드론별 {항목: 실패사유 또는 None}. 실패가 하나도 없으면 그 드론은 정상."""
    tol = float(pf.get('pose_tolerance', 0.30))
    hist = int(pf.get('kalman_history_len', 10))
    kthr = float(pf.get('kalman_threshold', 0.001))
    cam_stale = max(2.0 * float(pf.get('cam_max_age', 2.0)), 3.0)
    out = {}
    for d in drones:
        r = dict.fromkeys(_ITEMS)
        entry = col.pose.get(d)
        if entry is None:
            r['pose'] = '수신 없음'
            r['초기위치'] = '-'
        else:
            (x, y, z), rx = entry
            if t_end - rx > 1.5:
                r['pose'] = f'끊김 {t_end - rx:.1f}s'
            ex, ey = (float(v) for v in c['drones'][d]['base'])
            err = (abs(x - ex), abs(y - ey), abs(z - 0.0))
            if any(e > tol for e in err):
                r['초기위치'] = '오차 ({:.2f},{:.2f},{:.2f})m'.format(*err)
        if required:
            k = col.kal[d]
            if len(k) < hist:
                r['칼만'] = f'샘플 {len(k)}/{hist}'
            else:
                recent = k[-hist:]
                rng = [max(s[a] for s in recent) - min(s[a] for s in recent) for a in range(3)]
                if any(v >= kthr for v in rng):
                    r['칼만'] = '미수렴'
            st = col.status.get(d)
            if st is None:
                r['상태'] = '수신 없음'
            else:
                info, pm, rx = st
                CfStatus = col._CfStatus
                if t_end - rx > 2.0:
                    r['상태'] = f'끊김 {t_end - rx:.1f}s'
                elif info & CfStatus.SUPERVISOR_INFO_IS_TUMBLED:
                    r['상태'] = 'IS_TUMBLED'
                elif info & CfStatus.SUPERVISOR_INFO_IS_LOCKED:
                    r['상태'] = 'IS_LOCKED'
                elif pm == CfStatus.PM_STATE_SHUTDOWN:
                    r['상태'] = 'PM_SHUTDOWN'
        n = col.frames[d]
        if n == 0:
            r['카메라'] = '프레임 0장'
        elif t_end - col.frame_t.get(d, 0.0) > cam_stale:
            r['카메라'] = f'스트림 정체 {t_end - col.frame_t[d]:.1f}s'
        out[d] = r
    return out


def _print_table(c, drones, judged, col, required):
    names = [f'{d}({_role_label(c, d)})' for d in drones]
    w = max(len(n) for n in names) + 2
    print('\n═══ 사전점검 결과 ═══', flush=True)
    header = ' ' * w + ''.join(f'{h:^14}' for h in _ITEMS)
    print(header, flush=True)
    problems = {}
    for d, name in zip(drones, names):
        r = judged[d]
        cells = []
        for item in _ITEMS:
            if not required and item in ('칼만', '상태'):
                cells.append('-(시뮬)')
            elif r[item] is None:
                cells.append(f'✓ ({col.frames[d]}프레임)' if item == '카메라' else '✓')
            elif r[item] == '-':
                cells.append('-')
            else:
                cells.append('✗ ' + r[item])
        print(f'{name:<{w}}' + ''.join(f'{cell:^14}' for cell in cells), flush=True)
        bad = [item for item in _ITEMS
               if r[item] not in (None, '-') and (required or item not in ('칼만', '상태'))]
        if bad:
            problems[d] = bad
    return problems


def run_gate(config):
    """사전점검 게이트 진입점. 통과하면 그냥 돌아오고, 결함 승인 시 config['coshow']['degraded']
    를 채운다. 거부/비대화형 결함 시 SystemExit."""
    c = config.get('coshow')
    if not c:
        return
    pf = c.get('preflight') or {}
    if not pf.get('gate', False):
        return
    drones = list(c['drones'])
    required = bool(pf.get('required', False))
    window = float(pf.get('gate_window_sec', 10.0))

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    # 전용 Context: 뒤에 뜨는 BTRunner 의 기본 rclpy Context 를 건드리지 않는다
    ctx = rclpy.Context()
    rclpy.init(context=ctx)
    try:
        node = rclpy.create_node('coshow_preflight_gate', context=ctx)
        col = _Collector(node, drones, required)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        print(f'[사전점검] {window:.0f} 초 수집 중... (pose·카메라'
              + ('·칼만·상태' if required else ' — 시뮬이라 칼만·상태 생략') + ')', flush=True)
        t0 = time.monotonic()
        while time.monotonic() - t0 < window:
            ex.spin_once(timeout_sec=0.2)
        t_end = time.monotonic()
        judged = _judge(col, c, pf, drones, required, t_end)
        problems = _print_table(c, drones, judged, col, required)
        node.destroy_node()
    finally:
        try:
            rclpy.shutdown(context=ctx)
        except Exception:
            pass

    interactive = sys.stdin.isatty()
    if not problems:
        print('\n문제 없음 — 전 기체 정상.', flush=True)
        if not interactive:
            print('[사전점검] 비대화형: 자동 진행', flush=True)
            return
        ans = input('BT 를 시작할까? [Y/n]: ').strip().lower()
        if ans in ('', 'y', 'yes'):
            return
        raise SystemExit('[사전점검] 운영자가 시작을 취소했다')

    print(f'\n⚠ 문제 {sum(len(v) for v in problems.values())}건: '
          + ', '.join(f'{d}({", ".join(v)})' for d, v in problems.items()), flush=True)
    print('  계속하면 해당 기체는 결함(DEGRADED) 처리되어 자동 재배치로 대응한다', flush=True)
    print('  (미션기 결함 → 탐색기가 미션 승계 / 탐색기 결함 → 미션기 탐색 투입·구역 재배분)', flush=True)
    if not interactive:
        raise SystemExit('[사전점검] 비대화형 + 결함 → 중단 (안전 기본값). 수리 후 재실행할 것')
    ans = input('그래도 계속 진행? [y/N]: ').strip().lower()
    if ans not in ('y', 'yes'):
        raise SystemExit('[사전점검] 중단. 기체 수리/교체 후 재실행 (로스터부터 다시)')
    c['degraded'] = {d: list(v) for d, v in problems.items()}
    print('[사전점검] 결함 승인 — DEGRADED 로 BT 에 전달: ' + ', '.join(problems), flush=True)
