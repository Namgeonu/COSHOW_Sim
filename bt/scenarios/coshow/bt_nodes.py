"""COSHOW 중앙 제어 BT 노드 (py_bt_ros 시나리오 패키지).

- 블랙보드 단일 작성자: UpdateBlackboard 가 매 tick 토픽기반 정보 기록.

- 모든 이동은 ReactiveFallback(조건, 액션) 으로 가드 → 서비스 재호출 방지, pose 감시 도착 판정.
- 시각은 BT 프로세스의 단조 시계(time.monotonic) 로 통일 (/clock 부재, 메시지 stamp 는 비교에 쓰지 않음).

블랙보드 키 (bb)
  now                      : 현재 시각(monotonic)
  pose[robot]              : 드론 {x,y,z,t} / LIMO {x,y,yaw,t}
  detections[drone]        : 최신 MarkerDetections (원시)
  mission_marker           : {found, id, drone, pose, t}
  target_id                : mission id - offset (None 이면 미확정)
  target_marker            : {found, drone, pose(P_N), t}   # 최초 1회 latch. pose 는 이후 갱신 안 함
  target_seen_now[drone]   : 마지막으로 target 을 본 시각
  target_confirmed         : 재검출(또는 타임아웃) 확정 여부
  finder                   : 목표 위에 배치된 드론
  drone_arrived[robot]     : {goal, t}  (IsDroneAt 이 기록)
  limo_arrived[robot]      : {goal, t}  (LimoNavigateTo / IsLimoAt 이 기록)
  rescue_done_t            : 구조 완료 시각 (0 = 미완)
  cmd[robot]               : 마지막 명령 {kind, goal, t}
  led[robot]               : 마지막 LED 색
  health[drone]            : OK / LOST / STUCK / BLIND  (UpdateBlackboard 가 판정, 명령은 안 냄)
  health_reason[drone]     : 판정 사유 문자열 (로그용)
  retired                  : 고장으로 퇴역한 탐색 드론 집합 (Search 가 기록, 이번 실행 동안 영구)
  zone_assign[drone]       : 드론이 순환할 구역 id 목록 (구역 id = 원래 담당 드론 이름)
  search_progress[drone]   : (지금 도는 구역, 웨이포인트 인덱스)
  observe / searchers / all_drones : 이번 실행의 기체 명단. XML 은 기체 이름 대신 이 키를
                             robot_key / robots_key 로 참조한다 (roster 로 명단을 바꿔 끼우기 위해)
"""
import math
import signal
import threading
import time
from collections import deque

from modules.utils import config
from modules.base_bt_nodes import (  # noqa: F401 — 제어 노드는 bt_constructor 가 이 모듈에서 찾음
    BTNodeList, Status, Node, SyncCondition,
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel, AlwaysSuccess, AlwaysFailure,
)
from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService, ActionWithROSTopic,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from builtin_interfaces.msg import Duration as DurationMsg
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from crazyflie_interfaces.msg import LogDataGeneric, Status as CfStatus   # BT 의 Status 와 이름이 겹친다
from crazyflie_interfaces.srv import Takeoff, GoTo, Land, Arm
from coshow_interfaces.msg import MarkerDetections

# ── BT Node registration ──────────────────────────────────────────────────────
CUSTOM_ACTION_NODES = [
    'UpdateBlackboard',      # 조건형이지만 화재 시나리오의 GatherLocalInfo 처럼 액션으로 등록해도 무방
    'DroneTakeoff', 'DroneGoTo', 'DroneLand',
    'LimoNavigateTo',
    'Search', 'ReturnDrones', 'CatchTarget',
    'SetLed', 'Idle',
]
CUSTOM_CONDITION_NODES = [
    'PreflightReady',
    'IsMMFound', 'IsTMFound', 'IsTargetConfirmed',
    'IsMissionComplete', 'IsRescue',
    'IsDroneAt', 'IsDroneAirborne', 'IsDroneLanded', 'AreDronesReturn',
    'IsLimoAt',
]
BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)

# ── Config shortcuts ──────────────────────────────────────────────────────────
C = config['coshow']
DRONES = C['drones']
LIMOS = C['limos']
SEARCHERS = list(C['searchers'])
TOL = C['tolerances']
OBS = C['observe_point']


def now():
    return time.monotonic()


def _dist2(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _dur(sec):
    d = DurationMsg()
    d.sec = int(sec)
    d.nanosec = int((sec - int(sec)) * 1e9)
    return d


# ── Ctrl+C controlled landing (safe_two_cf_square_final.py 의 값을 따른다) ─────────────
CTRLC_LAND_HEIGHT = 0.05     # land 목표 고도(m). 궤적이 끝나면 펌웨어 HL commander 가 스스로 모터를 멈춘다
CTRLC_GROUND_Z = 0.0         # crazyflies.yaml initial_position 의 z (네 대 모두 0)
CTRLC_LANDED_TOL = 0.15      # 신선한 pose 의 z <= GROUND_Z + 이 값이면 착륙 확인 → disarm 허용
CTRLC_POSE_MAX_AGE = 0.8     # 이보다 오래된 pose 는 없는 것과 같이 취급한다 (z=0 으로 간주하지 않음)
CTRLC_SETTLE = 1.0           # land duration 뒤 추가로 기다리는 시간(초)


def _install_emergency_land(agent, ub):
    """Ctrl+C 로 BT 를 끌 때 종료 전에 공중의 기체를 controlled landing 시킨다.

    드론 명령은 서비스라 액션처럼 취소되지 않는다. halt() 는 명령 서명만 지울 뿐이고,
    이미 나간 go_to 는 펌웨어가 그대로 완주한 뒤 그 자리에서 무한 호버한다. 그러면
    조종할 주체가 사라진 채 기체가 공중에 남는다. 그래서 종료 경로에서 직접 내린다.

    절차 (emergency/모터 즉시 차단은 쓰지 않는다):
      Ctrl+C → 분류 → land(0.05 m) → duration+settle 대기 → pose 로 확인
             → 미확인 기체만 land 1회 재시도 → 확인된 기체만 disarm → 종료
      분류: UpdateBlackboard 가 받은 신선한(CTRLC_POSE_MAX_AGE 이내) pose 의 z 가
            tolerances.landed_z 이하이고 마지막 BT 명령이 takeoff/go_to 가 아닌 기체는
            "이미 바닥" 으로 보고 land 를 보내지 않는다. 펌웨어 plan_land() 는 planner 가
            IDLE 이어도 land 를 받아 현재 위치에서 0.05 m 로 가는 궤적을 다시 돌리므로,
            바닥의 기체에 land 를 또 보내면 duration 동안 모터가 돈다. pose 가 없거나
            오래됐거나 이륙 명령 직후(z 가 아직 낮아도 올라가는 중)면 안전을 위해 보낸다.
      확인: 신선한 pose 의 z <= CTRLC_GROUND_Z + CTRLC_LANDED_TOL. pose 부재/stale 은 미확인.
      disarm: 확인된 기체에만 /cfX/arm(arm=False). 미확인 기체는 경고만 남긴다.
            (아직 높이 있는 것으로 보이는 기체에 자동 disarm 을 보내지 않는다)

    핸들러는 메인 스레드에서 돌며 그동안 BT tick 이 멈추므로 새 임무 명령은 나가지 않는다.
    ROS spin 이 별도 스레드라(ros_bridge) 핸들러 안에서 call_async 해도 실제로 전송되고
    pose 도 계속 갱신된다. 착륙 중 Ctrl+C 를 다시 누르면 안내만 하고 절차를 계속한다.
    절차가 끝나면 KeyboardInterrupt 를 올려 main 의 정리 경로(close → halt_tree)로 넘긴다.
    """
    if not bool(C.get('emergency_land_on_exit', True)):
        return
    node = ub.ros.node
    land_cli = {d: node.create_client(Land, f'/{d}/land') for d in DRONES}
    arm_cli = {d: node.create_client(Arm, f'/{d}/arm') for d in DRONES}
    dur = float(C['durations']['land'])
    landed_z = float(TOL['landed_z'])
    state = {'landing': False}

    def _fresh_z(d):
        """신선한 pose 의 z. 없거나 오래됐으면 None."""
        with ub._lock:
            p = ub._pose.get(d)
        if p is None or now() - p['t'] > CTRLC_POSE_MAX_AGE:
            return None
        return float(p['z'])

    def _ztxt(d):
        z = _fresh_z(d)
        return f'{d} (no fresh pose)' if z is None else f'{d} (z={z:.2f})'

    def _confirmed(d):
        z = _fresh_z(d)
        return z is not None and z <= CTRLC_GROUND_Z + CTRLC_LANDED_TOL

    def _send_land(targets):
        sent = []
        for d in targets:
            if not land_cli[d].service_is_ready():
                continue
            req = Land.Request()
            req.group_mask = 0
            req.height = CTRLC_LAND_HEIGHT
            req.duration = _dur(dur)
            land_cli[d].call_async(req)
            sent.append(d)
        return sent

    def _wait(sec):
        end = now() + sec
        while now() < end:
            time.sleep(0.1)

    def _handler(signum, frame):
        if state['landing']:
            print('\n[CTRL+C] Controlled landing already in progress.', flush=True)
            return
        state['landing'] = True
        t0 = now()
        print('\n[CTRL+C] Controlled landing requested.', flush=True)

        cmd = agent.blackboard.get('cmd', {})
        pending, on_ground = [], []
        for d in DRONES:
            z = _fresh_z(d)
            last = (cmd.get(d) or {}).get('kind')
            if z is not None and z <= landed_z and last not in ('takeoff', 'go_to'):
                on_ground.append(d)
            else:
                pending.append(d)
        print('[LAND] already on ground (no land sent): {} | to land: {}'.format(
            ', '.join(_ztxt(d) for d in on_ground) or '-',
            ', '.join(_ztxt(d) for d in pending) or '-'), flush=True)

        for attempt in (1, 2):
            if not pending:
                break
            sent = _send_land(pending)
            no_srv = [d for d in pending if d not in sent]
            print('[LAND] attempt {}: land(height={:.2f} m, duration={:.1f} s) -> {}{}'.format(
                attempt, CTRLC_LAND_HEIGHT, dur, ', '.join(sent) or '-',
                f' | service not ready: {", ".join(no_srv)}' if no_srv else ''), flush=True)
            if not sent:
                break
            print(f'[LAND] waiting {dur + CTRLC_SETTLE:.1f} s for descent...', flush=True)
            _wait(dur + CTRLC_SETTLE)
            pending = [d for d in pending if not _confirmed(d)]
            if pending and attempt == 1:
                print('[LAND] landing not confirmed for {}; sending one additional land command.'
                      .format(', '.join(_ztxt(d) for d in pending)), flush=True)

        confirmed = [d for d in DRONES if _confirmed(d)]
        unconfirmed = [d for d in DRONES if d not in confirmed]
        print('[LAND] ground proximity confirmed: {}'.format(
            ', '.join(_ztxt(d) for d in confirmed) or '-'), flush=True)

        futs = {}
        for d in confirmed:
            if not arm_cli[d].service_is_ready():
                print(f'[LAND] {d}: /arm service not ready. Disarm skipped.', flush=True)
                continue
            req = Arm.Request()
            req.arm = False
            futs[d] = arm_cli[d].call_async(req)
        if futs:
            end = now() + 1.0
            while now() < end and not all(f.done() for f in futs.values()):
                time.sleep(0.05)
            acked = [d for d, f in futs.items() if f.done()]
            print('[LAND] disarm sent: {} (acknowledged: {})'.format(
                ', '.join(futs), ', '.join(acked) or '-'), flush=True)
        for d in unconfirmed:
            print(f'[LAND WARNING] {d} landing could not be confirmed. '
                  'Automatic disarm was NOT sent.', flush=True)

        print(f'[CTRL+C] Controlled landing finished ({now() - t0:.1f} s). Shutting down.',
              flush=True)
        # 착륙 절차가 끝났으니 이후 Ctrl+C 는 기본 동작(즉시 종료)으로 돌려 놓는다.
        # 정리 경로가 어딘가에서 멈추더라도 빠져나올 수 있게 하기 위함이다.
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGINT, _handler)
    except ValueError:
        # 메인 스레드가 아니면 설치할 수 없다. 그 경우 조용히 포기한다.
        pass


def _param(block, key, robot=None):
    """설정값 조회의 단일 출처. 2단 구조로 드론별 재정의를 허용한다.

      drones.<robot>.<block>.<key>  가 있으면 그 값       (드론별 재정의)
      없으면 <block>.<key>                                (전역 기본값)

    block 이름이 전역 블록 이름과 같아서 어느 값을 덮는지 config 만 봐도 드러난다.
    robot 을 안 넘기거나 그 드론에 재정의가 없으면 전역값이므로, 재정의를 쓰지 않는
    시나리오는 동작이 달라지지 않는다. 없는 키는 즉시 KeyError 로 드러낸다.
    """
    if robot is not None:
        per = DRONES.get(robot, {}).get(block, {})
        if key in per:
            return float(per[key])
    return float(C[block][key])


def _alt(key, robot=None):
    """고도(m). altitudes 블록 + drones.<robot>.altitudes 재정의."""
    return _param('altitudes', key, robot)


def _dur_of(key, robot=None):
    """명령 duration(초). durations 블록 + drones.<robot>.durations 재정의."""
    return _param('durations', key, robot)


def _robots_from_attrs(bb, robots=None, robots_key=None, exclude_key=None):
    """XML 속성(robots="cf230,cf231") 또는 블랙보드 키(robots_key="finder")로 로봇 목록 결정."""
    if robots_key:
        v = bb.get(robots_key)
        lst = [v] if isinstance(v, str) else list(v or [])
    else:
        lst = [r.strip() for r in str(robots).split(',') if r.strip()]
    if exclude_key:
        ex = bb.get(exclude_key)
        lst = [r for r in lst if r != ex]
    return lst


def _resolve_goal(bb, x=None, y=None, z=None, target_key=None):
    """(x,y[,z]) 직접 지정 또는 블랙보드 키(target_key → {'x','y',...}) 에서 목표 좌표."""
    if target_key:
        t = bb.get(target_key)
        if not t:
            return None
        gx, gy = float(t['x']), float(t['y'])
    else:
        if x is None or y is None:
            return None
        gx, gy = float(x), float(y)
    gz = None if z is None else float(z)
    return gx, gy, gz


# ═════════════════════════════════════════════════════════════════════════════
# 1. 블랙보드 갱신 (단일 작성자)
# ═════════════════════════════════════════════════════════════════════════════
class UpdateBlackboard(ConditionWithROSTopics):
    """모든 토픽을 구독해 블랙보드에 현재 상황을 기록한다.

    - 드론 pose: /cf_x/pose (PoseStamped)
    - LIMO pose: limos[x].pose_topic (Odometry, map 기준)
    - 검출:      /cf_x/marker_detections — 콜백에서 직접 처리(연속 프레임 카운트가 tick 보다 빠를 수 있으므로)
    """

    def __init__(self, name, agent):
        super().__init__(name, agent, msg_types_topics=[])
        self._cache['ready'] = True          # 베이스의 '캐시 비면 RUNNING' 우회
        self._lock = threading.Lock()
        node = self.ros.node

        self._pose = {}                      # robot -> dict
        self._det_latest = {}                # drone -> msg
        self._streak = {}                    # drone -> (id, count)   연속 검출 카운트
        self._mission_event = None           # 확정 대기 중인 이벤트
        self._target_events = []             # 확정된 target 검출 이벤트 큐
        self._seen_now = {}                  # drone -> t
        self._det_t = {}                     # drone -> 마지막 카메라 프레임 수신 시각 (BLIND 판정)

        # 기체 상태 판정 상태 (_update_health)
        self._health = {}                    # drone -> OK/LOST/STUCK/BLIND
        self._health_reason = {}
        self._last_bad_t = {}                # drone -> 마지막으로 이상이 관측된 시각 (t_recover 용)
        self._prog_ref = {}                  # drone -> 진행 기준점 {x,y,z,t,cmd_t} (STUCK 판정)

        for d in DRONES:
            node.create_subscription(PoseStamped, f'/{d}/pose',
                                     lambda m, r=d: self._on_drone_pose(r, m), 10)
            node.create_subscription(MarkerDetections, f'/{d}/marker_detections',
                                     lambda m, r=d: self._on_det(r, m), 10)
        for l, cfg in LIMOS.items():
            node.create_subscription(Odometry, cfg['pose_topic'],
                                     lambda m, r=l: self._on_limo_pose(r, m), 10)

        self._target_id_snapshot = None      # 콜백 스레드가 참조할 target_id 사본

        # Ctrl+C 로 BT 를 끌 때 기체를 공중에 두고 나가지 않도록.
        # 트리 구성은 메인 스레드에서 일어나므로 여기서 시그널을 잡을 수 있다.
        # 이 노드의 pose 캐시(_pose/_lock)와 bb['cmd'] 를 착륙 판단에 쓴다.
        _install_emergency_land(agent, self)

    # ---- ROS 콜백 (spin 스레드) ----
    def _on_drone_pose(self, robot, msg):
        p = msg.pose.position
        with self._lock:
            self._pose[robot] = {'x': p.x, 'y': p.y, 'z': p.z, 't': now()}

    def _on_limo_pose(self, robot, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._pose[robot] = {'x': p.x, 'y': p.y, 'yaw': yaw, 't': now()}

    def _on_det(self, drone, msg):
        t = now()
        ids = [m.id for m in msg.markers]
        with self._lock:
            self._det_latest[drone] = msg
            self._det_t[drone] = t
            tid = self._target_id_snapshot

            # 연속 프레임 카운트: 관심 ID(미션 범위 또는 target) 하나만 추적
            lo, hi = C['mission_marker_ids']
            interest = None
            if drone == C['observe_drone']:
                cand = [i for i in ids if lo <= i <= hi]
                interest = cand[0] if cand else None
            elif tid is not None and tid in ids:
                interest = tid
            prev_id, cnt = self._streak.get(drone, (None, 0))
            if interest is None:
                self._streak[drone] = (None, 0)
            else:
                cnt = cnt + 1 if prev_id == interest else 1
                self._streak[drone] = (interest, cnt)
                if cnt >= int(C['confirm_frames']):
                    # interest 로 고른 그 마커의 역투영 좌표를 쓴다. 한 프레임에 마커가
                    # 여러 개 잡혀도 타겟만 정확히 골라진다.
                    # (예전에는 msg.drone_pose 를 읽었다. 검출 노드가 거기에 마커 좌표를
                    #  덮어써 보내는 우회책이었는데, 마커가 2개 이상이면 어느 것인지 몰라
                    #  덮어쓰기를 건너뛰었고, 그러면 "드론이 서 있던 자리" 가 마커 위치로
                    #  둔갑해 조용히 엉뚱한 곳으로 갔다.)
                    # world_x/world_y 가 0 이면 역투영 실패이므로 그 프레임은 버리고
                    # 다음 프레임에 다시 시도한다. z 는 지면 마커라 역투영이 주지 않아
                    # 드론 고도를 그대로 쓴다 (CatchTarget 이 어차피 호버 고도로 덮는다).
                    m = next((k for k in msg.markers if k.id == interest), None)
                    if m is not None and (m.world_x != 0.0 or m.world_y != 0.0):
                        ev = {'drone': drone, 'id': interest, 't': t,
                              'pose': {'x': float(m.world_x),
                                       'y': float(m.world_y),
                                       'z': msg.drone_pose.pose.position.z}}
                        if drone == C['observe_drone']:
                            self._mission_event = ev
                        else:
                            self._target_events.append(ev)
            if tid is not None and tid in ids:
                self._seen_now[drone] = t

    # ---- tick (BT 스레드) ----
    def _predicate(self, agent, bb):
        t = now()
        bb['now'] = t
        bb.setdefault('drone_arrived', {})
        bb.setdefault('limo_arrived', {})
        bb.setdefault('cmd', {})
        bb.setdefault('led', {})
        bb.setdefault('mission_marker', {'found': False})
        bb.setdefault('target_marker', {'found': False})
        bb.setdefault('target_id', None)
        bb.setdefault('target_confirmed', False)
        bb.setdefault('finder', None)
        bb.setdefault('rescue_done_t', 0.0)
        bb.setdefault('target_seen_now', {})
        # 기체 명단 (roster 가 config 에 넣은 값). XML 의 robot_key="observe" 등이 읽는다.
        bb['observe'] = C['observe_drone']
        bb['searchers'] = list(SEARCHERS)
        bb['all_drones'] = list(DRONES)
        # 관측점(P0)을 블랙보드에 노출. phase1_observe.xml 이 좌표를 하드코딩하지 않고
        # target_key="observe_point" 로 읽어, 통합(-2,0)·리허설(0,0)이 각자 config 값으로 돈다.
        bb['observe_point'] = dict(OBS)

        with self._lock:
            bb['pose'] = {k: dict(v) for k, v in self._pose.items()}
            bb['detections'] = dict(self._det_latest)
            mission_ev = self._mission_event
            self._mission_event = None
            target_evs = self._target_events
            self._target_events = []
            bb['target_seen_now'] = dict(self._seen_now)
            self._target_id_snapshot = bb['target_id']

        # 필수 pose 수신 확인
        required = list(DRONES) + list(LIMOS)
        if any(r not in bb['pose'] for r in required):
            missing = [r for r in required if r not in bb['pose']]
            bb['missing_pose'] = missing
            return False

        # [기체 상태] 판정만 기록. 대응(퇴역·비상착륙·구역 이어받기)은 Search 가 한다.
        self._update_health(bb, t)

        # [미션 마커 확정] 게이팅: 발신=cf230, ID 범위(콜백에서 필터), cf230 가 관측점에 도착한 이후 스탬프
        if mission_ev and not bb['mission_marker']['found']:
            arr = bb['drone_arrived'].get(C['observe_drone'])
            ok = arr is not None and _dist2(arr['goal'][0], arr['goal'][1], OBS['x'], OBS['y']) < 0.3 \
                and mission_ev['t'] >= arr['t']
            if ok:
                bb['mission_marker'] = dict(mission_ev, found=True)
                bb['target_id'] = int(mission_ev['id']) - int(C['marker_id_offset'])

        # [타겟 마커] 발신 ∈ searchers, id == target_id (콜백 필터). 최초 1회만 latch (P_N 고정)
        for ev in target_evs:
            if ev['drone'] not in SEARCHERS and ev['drone'] != bb.get('finder'):
                continue
            if not bb['target_marker']['found']:
                bb['target_marker'] = dict(ev, found=True)
                bb['finder'] = ev['drone']
                # P_N 은 이 순간 확정하고 이후 갱신하지 않는다. 갱신하면
                # CatchTarget 의 목표가 매 tick 흔들려 go_to 가 재발행되고,
                # 궤적이 t=0 으로 리셋되어 감속 구간에 도달하지 못한다.
            # [재검출 확정] finder 가 P_N 도착 이후의 검출
            fa = bb['drone_arrived'].get(bb.get('finder'))
            if fa is not None and ev['t'] >= fa['t'] and ev['drone'] == bb.get('finder'):
                bb['target_confirmed'] = True
        # 재검출 타임아웃: finder 도착 후 recheck_timeout 초 동안 확정 없으면 P_N 그대로 확정
        if bb['target_marker']['found'] and not bb['target_confirmed']:
            fa = bb['drone_arrived'].get(bb.get('finder'))
            if fa is not None and t - fa['t'] > float(C['recheck_timeout']):
                bb['target_confirmed'] = True
                bb['target_confirm_note'] = 'timeout'

        # 편의 키 두 개.
        #   P_N       마커 실제 위치. 드론(CatchTarget)이 그 위에 호버할 때 쓴다.
        #   P_N_limo  P_N 에서 x 로 limo_approach_dx 만큼 물러선 접근점.
        #             마커가 건물 지붕에 있으면 P_N 은 건물이 서 있는 자리라,
        #             리모가 그대로 가면 건물을 들이받는다. 리모는 x- 쪽에서
        #             오므로 x 를 빼면 건물 앞에 선다. dx=0 이면 P_N 과 같다.
        if bb['target_marker']['found']:
            pn = bb['target_marker']['pose']
            bb['P_N'] = pn
            dx = float(C.get('limo_approach_dx', 0.0))
            bb['P_N_limo'] = dict(pn, x=pn['x'] + dx)
        return True

    # ---- 기체 상태 판정 ----
    def _update_health(self, bb, t):
        """bb['health'][d] ∈ {OK, LOST, STUCK, BLIND}. 판정만 하고 명령은 내지 않는다.

        LOST  : pose 가 pose_max_age 넘게 안 옴 (통신 두절).
        STUCK : takeoff/go_to 가 살아 있는데 movement_time_allowance 안에
                required_movement_radius 만큼 못 움직임. 목표에 도달해 호버 중이면 정상.
                리모 Nav2 에 이미 쓰고 있는 SimpleProgressChecker 와 같은 규칙이다
                (required_movement_radius / movement_time_allowance).
        BLIND : 카메라 프레임(marker_detections)이 frame_max_age 넘게 안 옴. 첫 프레임을
                받기 전에는 판정하지 않으므로 검출 노드가 BT 보다 늦게 떠도 오판이 없다.
                frame_max_age 가 0 이면 끈다.
        우선순위 LOST > STUCK > BLIND. 이상이 사라져도 t_recover 동안은 이전 상태를
        유지한다 (깜빡임 방지). 이미 퇴역한 탐색 드론은 회복해도 Search 가 다시 쓰지 않는다.
        health 블록이 config 에 없으면 전부 OK.
        """
        h = C.get('health') or {}
        bb.setdefault('health', {})
        bb.setdefault('health_reason', {})
        with self._lock:
            det_t = dict(self._det_t)
        for d in DRONES:
            raw, why = self._judge_health(bb, d, t, h, det_t) if h else (None, '')
            prev = self._health.get(d, 'OK')
            if raw is not None:
                new = raw
                self._last_bad_t[d] = t
            elif prev != 'OK' and t - self._last_bad_t.get(d, t) >= float(h.get('t_recover', 3.0)):
                new, why = 'OK', ''
            else:
                new, why = prev, self._health_reason.get(d, '')   # 회복 대기 중이거나 원래 OK
            if new != prev:
                print(f'[HEALTH] {d}: {prev} → {new}' + (f' ({why})' if why else ''), flush=True)
            self._health[d], self._health_reason[d] = new, why
            bb['health'][d], bb['health_reason'][d] = new, why

    def _judge_health(self, bb, d, t, h, det_t):
        """이번 tick 의 원시 판정. (상태, 사유) 또는 (None, '') 을 돌려준다."""
        p = bb['pose'].get(d)
        age = (t - p['t']) if p else float('inf')
        if age > float(h.get('pose_max_age', 1.0)):
            self._prog_ref.pop(d, None)
            return 'LOST', f'pose {age:.1f} s 끊김'

        cmd = bb['cmd'].get(d)
        if cmd and cmd['kind'] in ('takeoff', 'go_to'):
            # 기준점: 이 명령을 처음 본 순간의 위치. 거기서 반경만큼 벗어날 때마다 갱신.
            ref = self._prog_ref.get(d)
            if ref is None or ref['cmd_t'] != cmd['t']:
                ref = {'x': p['x'], 'y': p['y'], 'z': p['z'], 't': t, 'cmd_t': cmd['t']}
                self._prog_ref[d] = ref
            g = cmd['goal']
            # goal 형식이 두 가지다: Search/ReturnDrones 계열은 값만 담고
            # (go_to (x,y,z) / takeoff (height,)), XML 노드(_DroneService)는 맨 앞에
            # 종류 문자열을 함께 담는다 (('go_to',x,y,z) / ('takeoff',height)).
            # 문자열이 앞에 오면 벗겨내 값만 남긴다.
            if g and isinstance(g[0], str):
                g = g[1:]
            gtol = float(h.get('goal_reached_tol', 0.3))
            if cmd['kind'] == 'go_to':
                reached = _dist2(p['x'], p['y'], g[0], g[1]) <= gtol and abs(p['z'] - g[2]) <= gtol
            else:
                reached = abs(p['z'] - g[0]) <= gtol
            moved = math.sqrt((p['x'] - ref['x']) ** 2 + (p['y'] - ref['y']) ** 2 + (p['z'] - ref['z']) ** 2)
            if reached or moved >= float(h.get('required_movement_radius', 0.3)):
                ref.update(x=p['x'], y=p['y'], z=p['z'], t=t)
            elif t - ref['t'] > float(h.get('movement_time_allowance', 8.0)):
                return 'STUCK', f'{cmd["kind"]} 뒤 {t - ref["t"]:.1f} s 동안 {moved:.2f} m 이동'
        else:
            self._prog_ref.pop(d, None)            # land 중이거나 명령 없음: 정지가 정상

        fmax = float(h.get('frame_max_age', 0.0))
        if fmax > 0 and d in det_t and t - det_t[d] > fmax:
            return 'BLIND', f'카메라 {t - det_t[d]:.1f} s 끊김'
        return None, ''


# ═════════════════════════════════════════════════════════════════════════════
# 2. 조건 노드 (블랙보드 읽기 전용)
# ═════════════════════════════════════════════════════════════════════════════
class BBCondition(SyncCondition):
    """XML 속성을 그대로 인스턴스 속성으로 받는 블랙보드 조건 베이스."""

    def __init__(self, name, agent, **kw):
        super().__init__(name, self._check)
        self.attrs = kw
        for k, v in kw.items():
            setattr(self, k, v)

    def _check(self, agent, bb):
        raise NotImplementedError

    @staticmethod
    def _st(ok):
        return Status.SUCCESS if ok else Status.FAILURE


class PreflightReady(BBCondition):
    """실기체 비행 전 안전 관문. 트리 맨 위에 두어 첫 비행 명령이 나가기 전에 전체를 막는다.

    crazyswarm2 가 이미 내는 토픽만 읽어 판단한다. 명령은 어떤 것도 보내지 않는다
    (estimator reset·arm·takeoff·go_to·land·펌웨어 파라미터 쓰기 없음). 그래서 tick 이
    반복돼도 기체 상태를 바꾸는 부작용이 없고, 별도의 tools/preflight_node.py 를 띄울
    필요가 없다.

      /cfX/kalman_variance  LogDataGeneric  values[0:3] = kalman.varPX, varPY, varPZ
      /cfX/status           Status          supervisor_info, pm_state
      /cfX/pose             PoseStamped     UpdateBlackboard 가 bb['pose'] 에 적은 것을 읽는다
                                            (같은 토픽을 두 번 구독하지 않는다)

    기체마다 kalman_ready AND pose_ready AND supervisor_ready 이고, 네 대 모두 참이어야 SUCCESS.
      kalman     축별 최근 kalman_history_len 샘플의 max-min < kalman_threshold.
                 샘플이 다 쌓이기 전에는 아님.
      pose       실제 pose 를 받았고, 나이 <= pose_max_age 이며, 기대 초기위치
                 (drones.<cf>.base, z=0 — crazyflies.yaml 의 initial_position 과 같아야 함)
                 와 x/y/z 축별 오차 <= pose_tolerance 인 상태가 pose_stable_time 동안
                 끊기지 않고 유지. 한 번이라도 벗어나면 타이머를 0 부터 다시 센다.
      supervisor status 나이 <= status_max_age, IS_TUMBLED·IS_LOCKED 아님, PM_STATE_SHUTDOWN
                 아님. auto-arm 기체는 지상에서 IS_ARMED=True, CAN_BE_ARMED=False 가
                 정상이므로 그대로 통과. manual-arm 기체는 CAN_BE_ARMED 또는 IS_ARMED.
                 배터리 전압은 보지 않는다 (safe_two_cf_square_final.py 와 같은
                 power-state 수준까지만. AI-Deck 부하 검증 전이라 별도 임계값을 두지 않는다).

    준비 전에는 FAILURE 다. 루트 ReactiveSequence 가 FAILURE 로 끝나도 bt_runner 는 트리
    결과를 보지 않고 다음 tick 에 다시 돌리므로(main.py 의 loop), BT 는 종료되지 않고
    여기서 계속 기다리며 재평가한다. 그동안 아래 노드는 tick 되지 않으므로 명령이 0 건이다.

    네 대가 모두 통과한 순간 latch 되어 이후에는 항상 SUCCESS 다. 이륙하면 pose 가
    초기위치를 벗어나고 칼만 분산도 흔들리므로, latch 가 없으면 매 tick 관문이 닫혀
    임무가 멈춘다. 비행 중 감시는 이 노드의 몫이 아니다.

    config 의 preflight.required 가 false 면(시뮬: crazyswarm2 토픽 없음) 항상 통과한다.
    """

    _DEFAULTS = dict(kalman_history_len=10, kalman_threshold=0.001,
                     pose_max_age=0.8, pose_tolerance=0.10, pose_stable_time=2.0,
                     status_max_age=1.0)

    def __init__(self, name, agent, **kw):
        super().__init__(name, agent, **kw)
        pf = dict(self._DEFAULTS, **(C.get('preflight') or {}))
        self.required = bool(pf.get('required', False))
        self.hist_len = int(pf['kalman_history_len'])
        self.kal_thr = float(pf['kalman_threshold'])
        self.pose_max_age = float(pf['pose_max_age'])
        self.pose_tol = float(pf['pose_tolerance'])
        self.pose_stable = float(pf['pose_stable_time'])
        self.status_max_age = float(pf['status_max_age'])
        self.drones = list(DRONES)
        # 기대 초기위치. crazyflies.yaml 의 initial_position 과 같아야 한다.
        self.expected = {d: (float(DRONES[d]['base'][0]), float(DRONES[d]['base'][1]), 0.0)
                         for d in self.drones}

        self._lock = threading.Lock()
        self._kal = {d: [deque(maxlen=self.hist_len) for _ in range(3)] for d in self.drones}
        self._status = {}                                  # drone -> (Status msg, 수신시각)
        self._pose_good_since = {d: None for d in self.drones}
        self._last_check_t = None
        self._last_log_t = None
        self._passed = False

        if not self.required:
            print('[PRE-FLIGHT] 관문 비활성 (preflight.required=false). 실기체에서는 true 여야 한다.',
                  flush=True)
            return
        node = agent.ros_bridge.node
        for d in self.drones:
            node.create_subscription(LogDataGeneric, f'/{d}/kalman_variance',
                                     lambda m, r=d: self._on_kalman(r, m), 10)
            node.create_subscription(CfStatus, f'/{d}/status',
                                     lambda m, r=d: self._on_status(r, m), 10)
        print('[PRE-FLIGHT] 관문 활성: {} — 칼만 수렴·초기위치·슈퍼바이저 통과 전에는 명령 없음'
              .format(', '.join(self.drones)), flush=True)

    # ---- ROS 콜백 (spin 스레드) ----
    def _on_kalman(self, drone, msg):
        if len(msg.values) < 3:
            return
        with self._lock:
            for axis in range(3):
                self._kal[drone][axis].append(float(msg.values[axis]))

    def _on_status(self, drone, msg):
        with self._lock:
            self._status[drone] = (msg, now())

    # ---- 관문 (BT 스레드, _lock 안에서 호출) — 읽고 판단만 한다 ----
    def _kalman_ready(self, drone):
        hist = self._kal[drone]
        n = min(len(a) for a in hist)
        if n < self.hist_len:
            return False, f'samples {n}/{self.hist_len}'
        rng = tuple(max(a) - min(a) for a in hist)
        return all(r < self.kal_thr for r in rng), 'range=({:.6f},{:.6f},{:.6f})'.format(*rng)

    def _pose_ready(self, drone, bb, t):
        p = bb.get('pose', {}).get(drone)
        if p is None:
            self._pose_good_since[drone] = None
            return False, 'no pose'
        age = t - p['t']
        if age > self.pose_max_age:
            self._pose_good_since[drone] = None
            return False, f'pose stale {age:.1f}s'
        ex, ey, ez = self.expected[drone]
        err = (abs(p['x'] - ex), abs(p['y'] - ey), abs(p['z'] - ez))
        if any(e > self.pose_tol for e in err):
            self._pose_good_since[drone] = None
            return False, 'position error=({:.3f},{:.3f},{:.3f})'.format(*err)
        if self._pose_good_since[drone] is None:
            self._pose_good_since[drone] = t
        held = t - self._pose_good_since[drone]
        if held < self.pose_stable:
            return False, f'stable {held:.1f}/{self.pose_stable:.1f}s'
        return True, ''

    def _supervisor_ready(self, drone, t):
        entry = self._status.get(drone)
        if entry is None:
            return False, 'no /status'
        s, rx_t = entry
        if t - rx_t > self.status_max_age:
            return False, f'status stale {t - rx_t:.1f}s'
        info = int(s.supervisor_info)
        if info & CfStatus.SUPERVISOR_INFO_IS_TUMBLED:
            return False, 'IS_TUMBLED'
        if info & CfStatus.SUPERVISOR_INFO_IS_LOCKED:
            return False, 'IS_LOCKED'
        if int(s.pm_state) == CfStatus.PM_STATE_SHUTDOWN:
            return False, 'PM_STATE_SHUTDOWN'
        if info & CfStatus.SUPERVISOR_INFO_AUTO_ARM:
            # 기본 auto-arm 설정에서는 지상에서 IS_ARMED=True, CAN_BE_ARMED=False 가 정상. 거부하지 않는다.
            return True, ''
        if not (info & (CfStatus.SUPERVISOR_INFO_CAN_BE_ARMED | CfStatus.SUPERVISOR_INFO_IS_ARMED)):
            return False, 'manual-arm: neither CAN_BE_ARMED nor IS_ARMED'
        return True, ''

    @staticmethod
    def _why(ok, why):
        return '' if ok or not why else f'({why})'

    def _check(self, agent, bb):
        if not self.required or self._passed:
            return Status.SUCCESS
        t = bb.get('now', now())
        # "연속 유지" 는 이 노드가 매 tick 보고 있을 때만 보장된다. 평가가 한동안 끊겼으면
        # (일시정지 등) 그 사이를 알 수 없으므로 타이머를 다시 센다.
        if self._last_check_t is not None and t - self._last_check_t > self.pose_max_age:
            for d in self.drones:
                self._pose_good_since[d] = None
        self._last_check_t = t

        lines, all_ok = [], True
        with self._lock:
            for d in self.drones:
                k_ok, k_why = self._kalman_ready(d)
                p_ok, p_why = self._pose_ready(d, bb, t)
                s_ok, s_why = self._supervisor_ready(d, t)
                all_ok = all_ok and k_ok and p_ok and s_ok
                lines.append('[PRE-FLIGHT] {}: kalman={}{} | pose={}{} | supervisor={}{}'.format(
                    d, k_ok, self._why(k_ok, k_why), p_ok, self._why(p_ok, p_why),
                    s_ok, self._why(s_ok, s_why)))
        if all_ok:
            self._passed = True
            print('[PRE-FLIGHT] PASS: {} ready'.format(', '.join(self.drones)), flush=True)
            return Status.SUCCESS
        # 약 1 초에 한 번만 이유를 찍는다 (10 Hz tick 에서 로그 폭주 방지)
        if self._last_log_t is None or t - self._last_log_t >= 1.0:
            self._last_log_t = t
            print('\n'.join(lines), flush=True)
        return Status.FAILURE


class IsMMFound(BBCondition):
    def _check(self, agent, bb):
        return self._st(bb.get('mission_marker', {}).get('found', False))


class IsTMFound(BBCondition):
    def _check(self, agent, bb):
        return self._st(bb.get('target_marker', {}).get('found', False))


class IsTargetConfirmed(BBCondition):
    def _check(self, agent, bb):
        return self._st(bb.get('target_confirmed', False))


class IsRescue(BBCondition):
    """limo_b 가 P_N 에 도착한 뒤 rescue_sec 경과 → SUCCESS (rescue_done_t latch)."""

    def __init__(self, name, agent, robot='limo_b', target_key='P_N', **kw):
        super().__init__(name, agent, robot=robot, target_key=target_key, **kw)

    def _check(self, agent, bb):
        if bb.get('rescue_done_t', 0.0) > 0:
            return Status.SUCCESS
        arr = bb.get('limo_arrived', {}).get(self.robot)
        pn = bb.get(self.target_key)
        if arr is None or pn is None:
            return Status.FAILURE
        if _dist2(arr['goal'][0], arr['goal'][1], pn['x'], pn['y']) > float(TOL['limo_at']) + 0.1:
            return Status.FAILURE
        if bb['now'] - arr['t'] >= float(C['rescue_sec']):
            bb['rescue_done_t'] = bb['now']
            return Status.SUCCESS
        return Status.FAILURE


def _drone_home(bb, robot):
    # 고장으로 퇴역한 기체는 "돌아온 것" 으로 친다. 이미 착륙했거나 착륙을 시도했고 더
    # 시킬 수 있는 게 없다. 이렇게 안 하면 퇴역 기체 하나가 AreDronesReturn / ReturnDrones /
    # IsMissionComplete 를 영원히 막아 임무가 끝나지 않는다.
    if robot in bb.get('retired', ()):
        return True
    p = bb['pose'].get(robot)
    if p is None:
        return False
    bx, by = DRONES[robot]['base']
    return _dist2(p['x'], p['y'], bx, by) <= float(TOL['drone_at']) * 2 and p['z'] <= float(TOL['landed_z'])



def _limo_home(bb, robot):
    """base 로의 Nav2 goal 이 SUCCEEDED 된 기록이 있으면 home (pose 미사용, IsLimoAt 과 같은 기준)."""
    arr = bb['limo_arrived'].get(robot)
    if arr is None:
        return False
    bx, by = LIMOS[robot]['base']
    return _dist2(arr['goal'][0], arr['goal'][1], bx, by) <= float(TOL['limo_at'])


class IsMissionComplete(BBCondition):
    """종료 조건: 구출 완료 + 드론 전원 base 착륙 + LIMO 전원 base 도착(Nav2 결과). 참이면 트리 SUCCESS 로 정지."""

    def _check(self, agent, bb):
        if bb.get('rescue_done_t', 0.0) <= 0:
            return Status.FAILURE
        ok = all(_drone_home(bb, d) for d in DRONES) and all(_limo_home(bb, l) for l in LIMOS)
        return self._st(ok)


class IsDroneAt(BBCondition):
    """드론이 목표 반경 내에 hold 초 이상 머물면 SUCCESS. 도착 시 bb['drone_arrived'][robot] 기록."""

    def __init__(self, name, agent, robot=None, robot_key=None, x=None, y=None, z=None,
                 alt=None, target_key=None, tol=None, hold=None, **kw):
        super().__init__(name, agent, robot=robot, robot_key=robot_key, x=x, y=y, z=z,
                         alt=alt, target_key=target_key, tol=tol, hold=hold, **kw)
        self._since = None

    def _check(self, agent, bb):
        robot = bb.get(self.robot_key) if self.robot_key else self.robot
        goal = _resolve_goal(bb, self.x, self.y, self.z, self.target_key)
        p = bb['pose'].get(robot) if robot else None
        if p is None or goal is None:
            self._since = None
            return Status.FAILURE
        gx, gy, gz = goal
        # alt 를 주면 DroneGoTo 와 같은 고도로 판정해야 도착이 성립한다.
        # 드론별 재정의도 같은 robot 으로 읽어야 목표와 판정이 어긋나지 않는다.
        if gz is None and self.alt:
            gz = _alt(self.alt, robot)
        tol = float(self.tol) if self.tol is not None else float(TOL['drone_at'])
        hold = float(self.hold) if self.hold is not None else float(TOL['drone_hold'])
        inside = _dist2(p['x'], p['y'], gx, gy) <= tol and (gz is None or abs(p['z'] - gz) <= tol)
        if not inside:
            self._since = None
            return Status.FAILURE
        if self._since is None:
            self._since = bb['now']
        if bb['now'] - self._since >= hold:
            arr = bb['drone_arrived'].get(robot)
            if arr is None or _dist2(arr['goal'][0], arr['goal'][1], gx, gy) > 1e-3:
                bb['drone_arrived'][robot] = {'goal': (gx, gy), 't': self._since}
            return Status.SUCCESS
        return Status.FAILURE


class IsDroneAirborne(BBCondition):
    def _check(self, agent, bb):
        robot = bb.get(self.robot_key) if getattr(self, 'robot_key', None) else self.robot
        p = bb['pose'].get(robot)
        return self._st(p is not None and p['z'] >= float(getattr(self, 'z_min', TOL['airborne_z'])))


class IsDroneLanded(BBCondition):
    def _check(self, agent, bb):
        robot = bb.get(self.robot_key) if getattr(self, 'robot_key', None) else self.robot
        p = bb['pose'].get(robot)
        return self._st(p is not None and p['z'] <= float(getattr(self, 'z_max', TOL['landed_z'])))


class AreDronesReturn(BBCondition):
    """robots="cf230,cf231" / robots_key="finder" / exclude_key="finder" 조합. 목록이 비면 SUCCESS."""

    def __init__(self, name, agent, robots='', robots_key=None, exclude_key=None, **kw):
        super().__init__(name, agent, robots=robots, robots_key=robots_key, exclude_key=exclude_key, **kw)

    def _check(self, agent, bb):
        lst = _robots_from_attrs(bb, self.robots, self.robots_key, self.exclude_key)
        return self._st(all(_drone_home(bb, r) for r in lst))


class IsLimoAt(BBCondition):
    """LIMO 도착: Nav2 액션이 SUCCEEDED 를 반환한 기록(bb['limo_arrived'], LimoNavigateTo 가 기록)만 인정.

    pose 는 보지 않는다 — LIMO 는 Nav2 액션이 완료를 보고하므로 그 결과가 유일한 도착 기준.
    기록의 goal 이 이 노드의 목표와 같아야 한다 (limo_b: P_N 도착 ≠ base 도착).
    """

    def __init__(self, name, agent, robot, x=None, y=None, target_key=None, tol=None, **kw):
        super().__init__(name, agent, robot=robot, x=x, y=y, target_key=target_key, tol=tol, **kw)

    def _check(self, agent, bb):
        goal = _resolve_goal(bb, self.x, self.y, None, self.target_key)
        if goal is None:
            return Status.FAILURE
        gx, gy, _ = goal
        tol = float(self.tol) if self.tol is not None else float(TOL['limo_at'])
        arr = bb['limo_arrived'].get(self.robot)
        return self._st(arr is not None and _dist2(arr['goal'][0], arr['goal'][1], gx, gy) <= tol)


# ═════════════════════════════════════════════════════════════════════════════
# 3. 드론 명령 (서비스) — 가드된 ReactiveFallback 안에서만 사용
# ═════════════════════════════════════════════════════════════════════════════
class _DroneService(ActionWithROSService):
    """같은 목표는 1회만 호출하고 이후 RUNNING 유지. 조건이 참이 되면 halt → 다음 목표에 재사용 가능.

    ActionWithROSService 는 응답 직후 _sent=False 로 리셋되어 Reactive 구조에서 매 tick 재호출되므로,
    run 을 오버라이드해 '목표 서명(signature)이 바뀔 때만' 호출한다.
    """
    KIND = 'srv'

    def __init__(self, name, agent, srv_type, srv_suffix, robot=None, robot_key=None, **kw):
        self.robot_attr, self.robot_key = robot, robot_key
        self.attrs = kw
        for k, v in kw.items():
            setattr(self, k, v)
        self._clients = {}
        self._srv_type, self._srv_suffix = srv_type, srv_suffix
        # 베이스는 클라이언트 1개를 만들지만 robot_key 로 대상이 바뀔 수 있어 로봇별 클라이언트를 따로 둔다
        super().__init__(name, agent, (srv_type, f'/{robot or "unresolved"}/{srv_suffix}'))
        if robot:
            self._clients[robot] = self.client
        self._last_sig = None

    def _client_for(self, robot):
        if robot not in self._clients:
            self._clients[robot] = self.ros.node.create_client(self._srv_type, f'/{robot}/{self._srv_suffix}')
        return self._clients[robot]

    def _target(self, bb):
        return bb.get(self.robot_key) if self.robot_key else self.robot_attr

    # 하위 클래스: (request, signature) 반환. None 이면 FAILURE
    def _make(self, bb, robot):
        raise NotImplementedError

    async def run(self, agent, bb):
        robot = self._target(bb)
        if not robot:
            self.status = Status.FAILURE
            return self.status
        made = self._make(bb, robot)
        if made is None:
            self.status = Status.FAILURE
            return self.status
        req, sig = made
        sig = (robot,) + tuple(sig)
        if sig != self._last_sig:
            cli = self._client_for(robot)
            if not cli.wait_for_service(timeout_sec=0.0):
                self.status = Status.RUNNING
                return self.status
            cli.call_async(req)
            self._last_sig = sig
            bb['cmd'][robot] = {'kind': self.KIND, 'goal': sig[1:], 't': bb['now']}
        self.status = Status.RUNNING
        return self.status

    def halt(self):
        self._last_sig = None


def _goto_duration(bb, robot, gx, gy):
    """현재 위치→목표 xy 거리로 short/long duration 선택. 세 값 모두 드론별 재정의 가능."""
    p = bb['pose'].get(robot)
    d = _dist2(p['x'], p['y'], gx, gy) if p else float('inf')
    return (_dur_of('goto_short', robot) if d <= _dur_of('short_dist', robot)
            else _dur_of('goto_long', robot))


def _goto_request(gx, gy, gz, yaw, duration):
    req = GoTo.Request()
    req.group_mask = 0
    req.relative = False
    req.goal.x, req.goal.y, req.goal.z = float(gx), float(gy), float(gz)
    req.yaw = float(yaw)
    req.duration = _dur(float(duration))
    return req


class DroneTakeoff(_DroneService):
    KIND = 'takeoff'

    def __init__(self, name, agent, robot=None, robot_key=None, height=None, duration=None):
        super().__init__(name, agent, Takeoff, 'takeoff', robot, robot_key,
                         height=height, duration=duration)

    def _make(self, bb, robot):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = float(self.height) if self.height is not None else _alt('takeoff', robot)
        req.duration = _dur(float(self.duration) if self.duration is not None else _dur_of('takeoff', robot))
        return req, ('takeoff', req.height)


class DroneGoTo(_DroneService):
    """x,y 직접 지정 또는 target_key(P_N) 사용.

    고도는 alt="<altitudes 키>" 로 고른다 (예: alt="observe"). 미지정 시 altitudes.capture.
    z="1.5" 처럼 숫자를 직접 줄 수도 있으나, 값이 XML 로 흩어지므로 alt 사용을 권한다.
    """
    KIND = 'go_to'

    def __init__(self, name, agent, robot=None, robot_key=None, x=None, y=None, z=None,
                 alt=None, target_key=None, yaw=0.0, duration=None):
        super().__init__(name, agent, GoTo, 'go_to', robot, robot_key,
                         x=x, y=y, z=z, alt=alt, target_key=target_key,
                         yaw=yaw, duration=duration)

    def _make(self, bb, robot):
        goal = _resolve_goal(bb, self.x, self.y, self.z, self.target_key)
        if goal is None:
            return None
        gx, gy, gz = goal
        if gz is None:
            gz = _alt(self.alt, robot) if self.alt else _alt('capture', robot)
        dur = float(self.duration) if self.duration is not None else _goto_duration(bb, robot, gx, gy)
        return _goto_request(gx, gy, gz, self.yaw, dur), ('go_to', round(gx, 3), round(gy, 3), round(gz, 3))


class DroneLand(_DroneService):
    KIND = 'land'

    def __init__(self, name, agent, robot=None, robot_key=None, duration=None):
        super().__init__(name, agent, Land, 'land', robot, robot_key, duration=duration)

    def _make(self, bb, robot):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = _dur(float(self.duration) if self.duration is not None else _dur_of('land', robot))
        return req, ('land',)


# ═════════════════════════════════════════════════════════════════════════════
# 4. LIMO 명령 (Nav2 액션)
# ═════════════════════════════════════════════════════════════════════════════
class LimoNavigateTo(ActionWithROSAction):
    """NavigateToPose. SUCCEEDED → bb['limo_arrived'][robot]={goal,t} 기록 (LIMO 도착의 유일한 기준).
    ABORTED/CANCELED → retry_sec 후 같은 goal 재전송. halt 는 베이스 그대로(취소 안 함)."""

    def __init__(self, name, agent, robot, x=None, y=None, yaw=None, target_key=None, retry_sec=3.0):
        super().__init__(name, agent, (NavigateToPose, f'/{robot}/navigate_to_pose'))
        self.robot, self.x, self.y, self.yaw, self.target_key = robot, x, y, yaw, target_key
        self.retry_sec = float(retry_sec)
        self._goal_xy = None
        self._retry_at = None

    def _build_goal(self, agent, bb):
        goal = _resolve_goal(bb, self.x, self.y, None, self.target_key)
        if goal is None:
            return None
        gx, gy, _ = goal
        yaw = float(self.yaw) if self.yaw is not None else 0.0
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = gx, gy
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = _yaw_to_quat(yaw)
        g = NavigateToPose.Goal()
        g.pose = ps
        self._goal_xy = (gx, gy)
        bb['cmd'][self.robot] = {'kind': 'nav', 'goal': (round(gx, 3), round(gy, 3)), 't': bb['now']}
        return g

    def _on_running(self, agent, bb):
        # 베이스 기본값은 None 을 반환해 부모가 FAILURE 로 오판한다 → 반드시 RUNNING 반환 (MoveToTarget 과 동일)
        return Status.RUNNING

    def _interpret_result(self, result, agent, bb, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            bb['limo_arrived'][self.robot] = {'goal': self._goal_xy, 't': bb['now']}
            return Status.SUCCESS
        # 실패/취소: 바로 FAILURE 로 트리를 흔들지 않고 잠시 후 재전송
        self._retry_at = bb['now'] + self.retry_sec
        return Status.RUNNING

    async def run(self, agent, bb):
        if self._phase == 'idle' and self._retry_at is not None:
            if bb['now'] < self._retry_at:
                self.status = Status.RUNNING
                return self.status
            self._retry_at = None
        return await super().run(agent, bb)



# ═════════════════════════════════════════════════════════════════════════════
# 5. 복합 액션 (여러 드론에 연속 명령) — Explore 패턴: 항상 RUNNING
# ═════════════════════════════════════════════════════════════════════════════
class _MultiDroneAction(Node):
    """여러 드론에 서비스 명령을 내는 노드 공통부. 로봇별 마지막 명령 서명으로 재전송 방지."""

    def __init__(self, name, agent):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = 'Action'
        self._cli = {}
        self._sig = {}

    def _client(self, robot, kind):
        key = (robot, kind)
        if key not in self._cli:
            t = {'takeoff': Takeoff, 'go_to': GoTo, 'land': Land}[kind]
            self._cli[key] = self.ros.node.create_client(t, f'/{robot}/{kind}')
        return self._cli[key]

    def _send(self, bb, robot, kind, req, sig):
        sig = (kind,) + tuple(sig)
        if self._sig.get(robot) == sig:
            return False
        cli = self._client(robot, kind)
        if not cli.wait_for_service(timeout_sec=0.0):
            return False
        cli.call_async(req)
        self._sig[robot] = sig
        bb['cmd'][robot] = {'kind': kind, 'goal': sig[1:], 't': bb['now']}
        return True

    def _takeoff(self, bb, robot):
        req = Takeoff.Request()
        req.group_mask = 0
        req.height = _alt('takeoff', robot)
        req.duration = _dur(_dur_of('takeoff', robot))
        return self._send(bb, robot, 'takeoff', req, (req.height,))

    def _goto(self, bb, robot, gx, gy, gz, yaw=0.0, duration=None):
        # duration 을 주면 거리 기반 short/long 선택을 건너뛴다 (CatchTarget 전용).
        dur = float(duration) if duration is not None else _goto_duration(bb, robot, gx, gy)
        return self._send(bb, robot, 'go_to', _goto_request(gx, gy, gz, yaw, dur),
                          (round(gx, 3), round(gy, 3), round(gz, 3)))

    def _land(self, bb, robot):
        req = Land.Request()
        req.group_mask = 0
        req.height = 0.0
        req.duration = _dur(_dur_of('land', robot))
        return self._send(bb, robot, 'land', req, ())

    @staticmethod
    def _at(bb, robot, gx, gy, gz=None, tol=None):
        p = bb['pose'].get(robot)
        if p is None:
            return False
        tol = float(tol if tol is not None else TOL['drone_at'])
        return _dist2(p['x'], p['y'], gx, gy) <= tol and (gz is None or abs(p['z'] - gz) <= tol)

    def halt(self):
        self._sig = {}


def _lawnmower(zone, spacing, lane_axis='y', robot=None):
    """레인 탐색 웨이포인트. lane_axis 로 레인을 쪼개고, 다른 축으로 왕복. 드론은 레인 정중앙을 달린다.

    lane_axis='y' (기본): 구역 y폭을 spacing 으로 쪼개 각 레인의 중앙 y 를 유지하며 x축 왕복.
                          예) y[-3,-1], spacing 0.5 → 레인중앙 y -2.75,-2.25,-1.75,-1.25 / 각 레인 x 왕복.
    lane_axis='x':        구역 x폭을 쪼개 중앙 x 를 유지하며 y축 왕복.
    반환: [(x,y,z), ...]  (레인마다 시작↔끝 2점, 지그재그)
    """
    x0, x1 = zone['x']
    y0, y1 = zone['y']
    z = _alt('search', robot)   # 전역 altitudes.search, 드론별 재정의가 있으면 그 값
    if lane_axis == 'y':
        lane_lo, lane_hi, run_lo, run_hi = y0, y1, x0, x1
    else:
        lane_lo, lane_hi, run_lo, run_hi = x0, x1, y0, y1
    n_lanes = max(1, int(round(abs(lane_hi - lane_lo) / float(spacing))))
    pts = []
    for i in range(n_lanes):
        lane_c = lane_lo + float(spacing) * (i + 0.5)           # 레인 정중앙
        runs = (run_lo, run_hi) if i % 2 == 0 else (run_hi, run_lo)  # 지그재그
        for r in runs:
            pts.append((float(r), float(lane_c), z) if lane_axis == 'y' else (float(lane_c), float(r), z))
    return pts


def _zone_center(zone):
    return (sum(float(v) for v in zone['x']) / 2.0, sum(float(v) for v in zone['y']) / 2.0)


class Search(_MultiDroneAction):
    """탐색 드론들에게 이륙 → 구역 레인 웨이포인트를 순차 go_to. 항상 RUNNING.

    - 출격은 B→C→D 순, stagger_sec 시간차
    - 블랙보드 pose 로 웨이포인트 도착 판정 후 다음 점 전송
    - 레인 끝까지 가면 역순으로 반복(핑퐁)
    - 타겟 발견 시 상위 ReactiveFallback 이 halt → 더 이상 명령 안 냄 (finder 는 Phase 3 이 go_to 로 덮어씀)

    고장 대응 (판정은 UpdateBlackboard 의 bb['health'], 대응은 전부 여기서):
    - 탐색 드론이 OK 가 아니게 되면 그 자리에서 퇴역(retired). 이번 실행 동안 영구.
      공중에 있으면 land 를 1회 보낸다 (LOST 라도 best-effort 로 보낸다).
    - 퇴역 드론의 구역은 "클리어" 된 뒤에야 다른 드론이 이어받는다.
        · 신선한 pose 로 착륙(z <= landed_z) 확인          → 클리어
        · LOST(pose 없음): 퇴역 후 lost_clear_timeout 경과  → 확인 불가, 타임아웃 클리어 (경고)
        · 여전히 공중                                       → 클리어 안 함. 통제 불능 기체 위로 보내지 않는다
      착륙한 Crazyflie 는 높이 3 cm 라 탐색 고도 아래로 지나가도 안전하다.
    - 클리어된 구역은 생존 드론 중 구역 중심에 가장 가까운 한 대가 이어받는다.
      동점이면 searchers 목록 순. 한 번 정해지면 그 드론이 퇴역하기 전엔 바꾸지 않는다
      (매 tick 재계산하면 두 드론이 번갈아 가까워지며 배정이 흔들린다).
    - 이어받은 드론은 [자기 구역, 이어받은 구역...] 을 순환한다. 현재 구역의 왕복(시작점
      복귀)을 마친 시점에만 다음 구역으로 넘어가므로 하던 일이 끊기지 않는다.
      남의 구역이라도 고도는 지금 나는 드론의 altitudes.search 를 쓴다
      (드론별 고도 분리가 충돌 회피 수단이라 그 층을 유지해야 한다).
    """

    def __init__(self, name, agent, stagger_sec=None):
        super().__init__(name, agent)
        s = C['search']
        self.stagger = float(stagger_sec if stagger_sec is not None else s['stagger_sec'])
        self.tol = float(s['wp_tol'])
        self.zones = s['zones']
        # 구역 id = 원래 담당 드론 이름. 경로는 구역마다 하나. z 는 사용 시점에 나는 드론 것으로 바꾼다.
        self.paths = {z: _lawnmower(self.zones[z], float(s['lane_spacing']), s.get('lane_axis', 'y'), robot=z)
                      for z in SEARCHERS}
        self.h = C.get('health') or {}
        # 이번 실행 동안 영구인 상태 (halt 로 초기화하지 않는다)
        self.retired = {}       # drone -> 퇴역 시각
        self.cleared = set()    # 이어받아도 되는 퇴역 구역
        self.land_sent = {}     # drone -> 비상 land 를 보낸 시각
        self.taken_by = {}      # 퇴역 구역 -> 이어받은 드론
        self._warn_t = {}
        self._reset_state()

    def _reset_state(self):
        self.zone_list = {d: [d] for d in SEARCHERS}   # 드론 -> 순환할 구역 목록 (자기 구역이 중복될 수 있음)
        self.cyc = {d: 0 for d in SEARCHERS}           # 드론 -> zone_list 안의 현재 위치 (값이 아니라 인덱스로 순환)
        self.cur_zone = {d: d for d in SEARCHERS}      # 드론 -> 지금 도는 구역 (= zone_list[cyc], 경로/로그 편의용)
        self.idx = {d: 0 for d in SEARCHERS}           # 현재 구역 경로 안의 웨이포인트
        self.dirn = {d: 1 for d in SEARCHERS}          # 핑퐁 방향
        self.started_t = {}

    # ---- 고장 처리 ----
    def _retire_check(self, bb, t):
        health = bb.get('health', {})
        for d in SEARCHERS:
            if d not in self.retired and health.get(d, 'OK') != 'OK':
                self.retired[d] = t
                why = bb.get('health_reason', {}).get(d, '')
                print(f'[SEARCH] {d} 퇴역: {health.get(d)}' + (f' ({why})' if why else '')
                      + f'. 구역 {d} 는 착륙 확인 후 다른 드론이 이어받는다', flush=True)
            if d in self.retired:
                self._handle_retired(bb, d, t)
        bb['retired'] = set(self.retired)

    def _handle_retired(self, bb, d, t):
        if d in self.cleared:
            return
        p = bb['pose'].get(d)
        fresh = p is not None and (t - p['t']) <= float(self.h.get('pose_max_age', 1.0))
        if fresh and p['z'] <= float(TOL['landed_z']):
            self.cleared.add(d)
            print(f'[SEARCH] {d} 착륙 확인 (z={p["z"]:.2f} m). 구역 {d} 이어받기 허용', flush=True)
            return
        # 공중이거나 LOST: 비상 착륙 1회. 서비스가 준비 안 됐으면 다음 tick 재시도.
        if d not in self.land_sent and self._land(bb, d):
            self.land_sent[d] = t
            if fresh:
                print(f'[SEARCH] {d} 비상 착륙 명령 (z={p["z"]:.2f} m)', flush=True)
            else:
                print(f'[SEARCH] {d} LOST — land 를 best-effort 로 보냄 (도달 여부 확인 불가)', flush=True)
        if not fresh:
            timeout = float(self.h.get('lost_clear_timeout', 12.0))
            if t - self.retired[d] >= timeout:
                self.cleared.add(d)
                print(f'[SEARCH WARNING] {d} LOST 로 착륙 미확인. 퇴역 후 {timeout:.0f} s 경과 → '
                      f'내려앉았다고 보고 구역 {d} 를 넘긴다', flush=True)
            return
        # 신선한 pose 인데 아직 공중: 하강 시간은 기다리고, 그 뒤에도 떠 있으면 주기적으로 경고
        settle = _dur_of('land', d) + 2.0
        since_land = (t - self.land_sent[d]) if d in self.land_sent else 0.0
        if since_land >= settle and t - self._warn_t.get(d, -1e9) >= float(self.h.get('stale_warn_period', 5.0)):
            self._warn_t[d] = t
            print(f'[SEARCH WARNING] {d} 여전히 공중 (z={p["z"]:.2f} m). 착륙 확인 전까지 '
                  f'구역 {d} 는 넘기지 않는다', flush=True)

    def _effective_zones(self, bb):
        """드론 -> 순환할 구역 id 목록. 자기 구역을 이어받은 구역 사이사이에 끼운다.

        예) cf232 가 cf231·cf233 을 이어받으면 [cf232, cf231, cf232, cf233] 로 만들어,
        순환이 cf232→cf231→cf232→cf233→cf232→... 가 된다. 이어받은 구역으로 갈 때마다
        자기 구역을 거치므로, 위 구역에서 아래 구역으로 가운데를 건너뛰지 않는다.
        (자기 구역이 목록에 여러 번 나오므로 run() 은 값 검색이 아니라 위치 포인터 cyc 로 순환한다.)
        """
        alive = [d for d in SEARCHERS if d not in self.retired]
        if not alive:
            return {d: [d] for d in alive}
        borrowed = {d: [] for d in alive}
        for dead in SEARCHERS:
            if dead in alive or dead not in self.cleared:
                continue
            helper = self.taken_by.get(dead)
            if helper is None or helper in self.retired:
                cx, cy = _zone_center(self.zones[dead])

                def key(d):
                    p = bb['pose'].get(d)
                    dist = _dist2(p['x'], p['y'], cx, cy) if p else float('inf')
                    return (dist, SEARCHERS.index(d))
                helper = min(alive, key=key)
                self.taken_by[dead] = helper
                print(f'[SEARCH] 구역 {dead} → {helper} 이어받음 (구역 중심 최근접, 동점은 목록 순)', flush=True)
            borrowed[helper].append(dead)
        assign = {}
        for d in alive:
            lst = [d]
            for b in borrowed[d]:
                lst += [b, d]            # 이어받은 구역 뒤에 자기 구역을 끼운다
            assign[d] = lst[:-1] if len(lst) > 1 else lst   # 맨 끝 자기 구역은 순환이 되메우므로 제거
        return assign

    def _apply_assign(self, bb):
        assign = self._effective_zones(bb)
        for d, lst in assign.items():
            if lst != self.zone_list[d]:
                print(f'[SEARCH] {d} 구역 목록 {self.zone_list[d]} → {lst}', flush=True)
                self.zone_list[d] = list(lst)
                # 목록이 바뀌면 자기 구역(맨 앞)부터 다시 순환한다. 배정 변화는 드물어(퇴역·클리어
                # 시점) 진행 중 왕복을 끊는 대가가 작고, 중복 항목이 있어 포인터를 새로 잡아야 한다.
                self.cyc[d] = 0
                self.cur_zone[d] = lst[0]
                self.idx[d], self.dirn[d] = 0, 1
        bb['zone_assign'] = {d: list(v) for d, v in assign.items()}

    async def run(self, agent, bb):
        t = bb['now']
        self._retire_check(bb, t)
        self._apply_assign(bb)

        prev_started = None
        for d in SEARCHERS:
            if d in self.retired:
                continue
            # 출격 게이트: 앞 드론이 출격한 지 stagger 이상
            if d not in self.started_t:
                if prev_started is not None and t - prev_started < self.stagger:
                    break
                self.started_t[d] = t
            prev_started = self.started_t[d]

            p = bb['pose'].get(d)
            if p is None:
                continue
            if p['z'] < float(TOL['airborne_z']):
                self._takeoff(bb, d)
                continue
            path = self.paths[self.cur_zone[d]]
            gz = _alt('search', d)                    # 남의 구역이라도 내 고도로 난다
            gx, gy, _ = path[self.idx[d]]
            if self._at(bb, d, gx, gy, None, self.tol):
                n = self.idx[d] + self.dirn[d]
                if n >= len(path) or n < 0:
                    lst = self.zone_list[d]
                    if self.idx[d] == 0 and len(lst) > 1:
                        # 시작점으로 되돌아옴 = 왕복 완료. 목록의 다음 위치로 (자기 구역이 중복될 수
                        # 있으므로 값 검색이 아니라 위치 포인터 cyc 를 한 칸 전진시킨다)
                        self.cyc[d] = (self.cyc[d] + 1) % len(lst)
                        nz = lst[self.cyc[d]]
                        print(f'[SEARCH] {d} 구역 {self.cur_zone[d]} 왕복 완료 → 구역 {nz} 로', flush=True)
                        self.cur_zone[d], self.dirn[d] = nz, 1
                        path, n = self.paths[nz], 0
                    else:
                        # 끝점: 방향을 뒤집어 핑퐁
                        self.dirn[d] *= -1
                        n = self.idx[d] + self.dirn[d]
                self.idx[d] = n
                gx, gy, _ = path[n]
            self._goto(bb, d, gx, gy, gz)
        bb['search_progress'] = {d: (self.cur_zone[d], self.idx[d]) for d in SEARCHERS if d not in self.retired}
        self.status = Status.RUNNING
        return self.status

    def halt(self):
        # 타겟 발견으로 halt 되면 다음 사이클을 위해 출격 순서·구역 포인터 초기화.
        # 퇴역·클리어·이어받기 기록은 이번 실행 동안 유효하므로 남긴다.
        super().halt()
        self._reset_state()


class ReturnDrones(_MultiDroneAction):
    """지정 드론들을 순차(stagger)로 base 상공 go_to → 착륙. 이미 착륙해 있으면 건너뜀. 항상 RUNNING."""

    def __init__(self, name, agent, robots='', robots_key=None, exclude_key=None, stagger_sec=None):
        super().__init__(name, agent)
        self.robots, self.robots_key, self.exclude_key = robots, robots_key, exclude_key
        self.stagger = float(stagger_sec if stagger_sec is not None else C['search']['stagger_sec'])
        self.started_t = {}

    async def run(self, agent, bb):
        t = bb['now']
        prev_started = None
        for d in _robots_from_attrs(bb, self.robots, self.robots_key, self.exclude_key):
            if _drone_home(bb, d):
                continue
            if d not in self.started_t:
                if prev_started is not None and t - prev_started < self.stagger:
                    break
                self.started_t[d] = t
            prev_started = self.started_t[d]
            bx, by = DRONES[d]['base']
            bz = _alt('return', d)
            p = bb['pose'].get(d)
            if p is None:
                continue
            if self._at(bb, d, bx, by, None, TOL['drone_at'] * 1.5):
                self._land(bb, d)
            elif p['z'] >= float(TOL['airborne_z']):
                self._goto(bb, d, bx, by, bz)
            # 공중도 아니고 base 도 아닌 경우(비정상): 명령 보류
        self.status = Status.RUNNING
        return self.status

    def halt(self):
        super().halt()
        self.started_t = {}


class CatchTarget(_MultiDroneAction):
    """P_N 에 xy 최근접 드론을 finder 로 정해 P_N 상공으로 go_to. 도착 시 drone_arrived[finder] 기록."""

    def __init__(self, name, agent):
        super().__init__(name, agent)
        self._since = None
        self._sent_goal = {}     # finder -> 실제로 보낸 목표 (finder 당 1회만 발행)

    async def run(self, agent, bb):
        pn = bb.get('P_N')
        if pn is None:
            self.status = Status.FAILURE
            return self.status
        gx, gy = float(pn['x']), float(pn['y'])
        # 퇴역 드론은 후보에서 뺀다. LOST 드론은 bb['pose'] 에 옛 값이 남아 공중으로 보일 수 있고,
        # finder 였던 드론이 퇴역하면 여기서 자동으로 최근접 생존 드론으로 넘어간다.
        retired = bb.get('retired', ())
        cands = [d for d in SEARCHERS if d not in retired
                 and d in bb['pose'] and bb['pose'][d]['z'] >= float(TOL['airborne_z'])]
        if not cands:
            self.status = Status.FAILURE
            return self.status
        if bb.get('finder') not in cands:
            bb['finder'] = min(cands, key=lambda d: _dist2(bb['pose'][d]['x'], bb['pose'][d]['y'], gx, gy))
        f = bb['finder']
        # 고도는 finder 가 정해진 뒤에 읽는다 (드론별 재정의를 반영하기 위해)
        gz = _alt('capture', f)
        # 명령은 finder 당 1회만. 이후 P_N 이 갱신돼도 재발행하지 않는다.
        # (매 tick 재발행하면 go_to 궤적이 t=0 으로 리셋되어 감속 구간에 못 간다)
        # 서비스 미준비로 실패하면 기록하지 않아 다음 tick 에 다시 시도한다.
        if f not in self._sent_goal and self._goto(bb, f, gx, gy, gz,
                                                   duration=_dur_of('capture', f)):
            self._sent_goal[f] = (gx, gy, gz)
        # 도착(hold) 판정 → 재검출 게이팅용 시각 기록.
        # 기준은 "실제로 보낸 목표". 갱신되는 P_N 으로 재면 드론이 선 자리와
        # 어긋나 도착 판정이 영영 안 나고 확정·타임아웃이 둘 다 막힌다.
        tgt = self._sent_goal.get(f)
        if tgt is not None and self._at(bb, f, *tgt):
            if self._since is None:
                self._since = bb['now']
            if bb['now'] - self._since >= float(TOL['drone_hold']):
                arr = bb['drone_arrived'].get(f)
                if arr is None or _dist2(arr['goal'][0], arr['goal'][1], tgt[0], tgt[1]) > 1e-3:
                    bb['drone_arrived'][f] = {'goal': (tgt[0], tgt[1]), 't': self._since}
        else:
            self._since = None
        self.status = Status.RUNNING
        return self.status

    def halt(self):
        super().halt()
        self._since = None
        self._sent_goal = {}


# ═════════════════════════════════════════════════════════════════════════════
# 6. 보조 액션
# ═════════════════════════════════════════════════════════════════════════════
class SetLed(Node):
    """LED-Ring stub: /{robot}/led (std_msgs/String) 에 색상 발행. 색이 바뀔 때만 발행, 항상 SUCCESS.
    실물에서는 crazyswarm2 파라미터 쓰기로 교체."""

    def __init__(self, name, agent, color, robot=None, robot_key=None):
        super().__init__(name)
        self.ros = agent.ros_bridge
        self.type = 'Action'
        self.color, self.robot, self.robot_key = str(color), robot, robot_key
        self._pubs = {}
        self._sent = set()   # 이 노드 인스턴스가 이미 보낸 로봇 (Reactive 재tick 시 중복 발행 방지)

    async def run(self, agent, bb):
        robots = list(DRONES) if self.robot == 'all' else _robots_from_attrs(bb, self.robot, self.robot_key)
        for r in robots:
            if not r or r in self._sent:
                continue
            if r not in self._pubs:
                self._pubs[r] = self.ros.node.create_publisher(String, f'/{r}/{C["led_topic_suffix"]}', 10)
            self._pubs[r].publish(String(data=self.color))
            bb['led'][r] = self.color
            self._sent.add(r)
        self.status = Status.SUCCESS
        return self.status

    def halt(self):
        self._sent = set()


class Idle(Node):
    """아무것도 하지 않고 RUNNING (조건 달성을 기다리는 자리표시)."""

    def __init__(self, name, agent):
        super().__init__(name)
        self.type = 'Action'

    async def run(self, agent, bb):
        self.status = Status.RUNNING
        return self.status
