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
"""
import math
import threading
import time

from modules.utils import config
from modules.base_bt_nodes import (  # noqa: F401 — 제어 노드는 bt_constructor 가 이 모듈에서 찾음
    BTNodeList, Status, Node, SyncCondition,
    Sequence, Fallback, ReactiveSequence, ReactiveFallback, Parallel, AlwaysSuccess, AlwaysFailure,
)
from modules.base_bt_nodes_ros import (
    ConditionWithROSTopics, ActionWithROSAction, ActionWithROSService, ActionWithROSTopic,
)

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from builtin_interfaces.msg import Duration as DurationMsg
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from crazyflie_interfaces.srv import Takeoff, GoTo, Land
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
    'IsReady',
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

        for d in DRONES:
            node.create_subscription(PoseStamped, f'/{d}/pose',
                                     lambda m, r=d: self._on_drone_pose(r, m), 10)
            node.create_subscription(MarkerDetections, f'/{d}/marker_detections',
                                     lambda m, r=d: self._on_det(r, m), 10)
        for l, cfg in LIMOS.items():
            node.create_subscription(Odometry, cfg['pose_topic'],
                                     lambda m, r=l: self._on_limo_pose(r, m), 10)

        # tools/preflight_node.py 의 실기체 안전 점검 결과. 시뮬에는 그 노드가 없으므로
        # preflight.required 가 false 면 이 값을 보지 않는다 (IsReady 참조).
        # 발행 측이 transient_local 이라 BT 가 늦게 떠도 마지막 값을 받는다.
        self._preflight_ready = False
        node.create_subscription(
            Bool, '/preflight/ready', self._on_preflight,
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self._target_id_snapshot = None      # 콜백 스레드가 참조할 target_id 사본

    # ---- ROS 콜백 (spin 스레드) ----
    def _on_drone_pose(self, robot, msg):
        p = msg.pose.position
        with self._lock:
            self._pose[robot] = {'x': p.x, 'y': p.y, 'z': p.z, 't': now()}

    def _on_preflight(self, msg):
        with self._lock:
            self._preflight_ready = bool(msg.data)

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

        with self._lock:
            bb['pose'] = {k: dict(v) for k, v in self._pose.items()}
            bb['detections'] = dict(self._det_latest)
            bb['preflight_ready'] = self._preflight_ready
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

        # 편의 키: P_N (LimoNavigateTo/DroneGoTo 의 target_key 로 사용)
        if bb['target_marker']['found']:
            bb['P_N'] = bb['target_marker']['pose']
        return True


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


class IsReady(BBCondition):
    """실기체 안전 점검 통과 여부. 트리 맨 위에 두어 전체를 막는 관문.

    tools/preflight_node.py 가 /preflight/ready 로 알린다. 그 노드가 칼만 수렴·
    위치 일치·슈퍼바이저 상태를 확인하고 무장까지 끝낸 뒤에야 True 가 된다.
    비행 중 이상(통신 두절, 자세 이상, 기체 간 최소거리 위반)이 생기면 False 로
    내려가고, 그러면 이 조건이 FAILURE 가 되어 BT 가 그 tick 부터 명령을 멈춘다.
    (명령이 멈춘 뒤 preflight 노드가 착륙시키므로 서로 다투지 않는다.)

    시뮬에는 그 노드가 없다. config 의 preflight.required 가 false 면 항상 통과한다.
    """

    def _check(self, agent, bb):
        if not bool(C.get('preflight', {}).get('required', False)):
            return Status.SUCCESS
        return self._st(bb.get('preflight_ready', False))


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
        p = bb['pose'].get(self.robot)
        return self._st(p is not None and p['z'] >= float(getattr(self, 'z_min', TOL['airborne_z'])))


class IsDroneLanded(BBCondition):
    def _check(self, agent, bb):
        p = bb['pose'].get(self.robot)
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


class Search(_MultiDroneAction):
    """탐색 드론들에게 이륙 → 구역 레인 웨이포인트를 순차 go_to. 항상 RUNNING.

    - 출격은 B→C→D 순, stagger_sec 시간차
    - 블랙보드 pose 로 웨이포인트 도착 판정 후 다음 점 전송
    - 레인 끝까지 가면 역순으로 반복(핑퐁)
    - 타겟 발견 시 상위 ReactiveFallback 이 halt → 더 이상 명령 안 냄 (finder 는 Phase 3 이 go_to 로 덮어씀)
    """

    def __init__(self, name, agent, stagger_sec=None):
        super().__init__(name, agent)
        s = C['search']
        self.stagger = float(stagger_sec if stagger_sec is not None else s['stagger_sec'])
        self.tol = float(s['wp_tol'])
        self.paths = {d: _lawnmower(s['zones'][d], float(s['lane_spacing']), s.get('lane_axis', 'y'), robot=d)
                      for d in SEARCHERS}
        self._reset_state()

    def _reset_state(self):
        self.idx = {d: 0 for d in SEARCHERS}
        self.dirn = {d: 1 for d in SEARCHERS}
        self.started_t = {}

    async def run(self, agent, bb):
        t = bb['now']
        prev_started = None
        for d in SEARCHERS:
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
            path = self.paths[d]
            gx, gy, gz = path[self.idx[d]]
            if self._at(bb, d, gx, gy, None, self.tol):
                # 다음 웨이포인트 (핑퐁)
                n = self.idx[d] + self.dirn[d]
                if n >= len(path) or n < 0:
                    self.dirn[d] *= -1
                    n = self.idx[d] + self.dirn[d]
                self.idx[d] = n
                gx, gy, gz = path[n]
            self._goto(bb, d, gx, gy, gz)
        bb['search_progress'] = dict(self.idx)
        self.status = Status.RUNNING
        return self.status

    def halt(self):
        # 타겟 발견으로 halt 되면 다음 사이클을 위해 출격 순서·레인 인덱스 초기화
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
        cands = [d for d in SEARCHERS if d in bb['pose'] and bb['pose'][d]['z'] >= float(TOL['airborne_z'])]
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
