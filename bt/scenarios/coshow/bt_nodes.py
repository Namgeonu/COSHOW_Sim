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
  health[drone]            : OK / LOST / STUCK / BLIND / DEGRADED  (UpdateBlackboard 판정, 명령은 안 냄)
  health_reason[drone]     : 판정 사유 문자열 (로그용)
  retired                  : 고장으로 퇴역한 드론 집합 (원본은 ROLE['retired'] — Observer/Search 가
                             기록, UpdateBlackboard 가 매 tick 사본을 씀. 이번 실행 동안 영구)
  ever_airborne            : 이륙 이력이 있는 드론 집합 (퇴역 클리어의 "공중일 수 없음" 판단용)
  zone_assign[drone]       : 드론이 순환할 구역 id 목록 (구역 id = 원래 담당 드론 이름)
  search_progress[drone]   : (지금 도는 구역, 웨이포인트 인덱스)
  observe / searchers / all_drones : 이번 실행의 기체 명단. XML 은 기체 이름 대신 이 키를
                             robot_key / robots_key 로 참조한다 (roster 로 명단을 바꿔 끼우기 위해)
"""
import json
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ── BT Node registration ──────────────────────────────────────────────────────
CUSTOM_ACTION_NODES = [
    'UpdateBlackboard',      # 조건형이지만 화재 시나리오의 GatherLocalInfo 처럼 액션으로 등록해도 무방
    'DroneTakeoff', 'DroneGoTo', 'DroneLand',
    'LimoNavigateTo',
    'Search', 'ReturnDrones', 'CatchTarget',
    'SetLed', 'Idle',
    'Observer',              # 미션 역할 감시·승계 + 전 기체 퇴역·비상착륙 집행 (모든 국면)
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
# 사전점검 게이트(preflight_gate.py)에서 운영자가 "결함 있어도 계속" 을 승인한 기체 {드론: [항목]}.
# health 가 t=0 부터 이 기체들을 DEGRADED 로 확정해, 유예 시간 없이 바로 재배치가 발동한다.
DEGRADED = C.get('degraded') or {}

# 실행 중 바뀌는 역할·퇴역 상태 — UpdateBlackboard / Observer / Search 가 공유하는 단일 원본.
#   observe   : 현재 미션(관측) 역할 기체. Observer 가 승계 시 바꾼다. bb['observe'] 는 매 tick 사본.
#               XML 은 robot_key="observe" 로 읽으므로 이 값만 바뀌면 트리는 자동 추종한다.
#   retired   : 드론 -> 퇴역 시각. Observer(전 기체 판정)와 Search(구역 이양)가 함께 본다.
#   land_sent : 드론 -> 마지막 비상 land 전송 시각. 두 노드가 같은 기체에 중복 전송하지 않게 공유.
#   obs_joined: 미션기가 탐색에 투입됐는가 (Search 가 설정). bb['searchers'] 확장에 쓴다.
ROLE = {'observe': C['observe_drone'], 'retired': {}, 'land_sent': {}, 'obs_joined': False}


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
        self._hits = {}                      # drone -> (id, [(t,x,y,z,off)])  슬라이딩 윈도우 확정(방식 B)
        self._mission_event = None           # 확정 대기 중인 이벤트
        self._target_events = []             # 확정된 target 검출 이벤트 큐
        self._seen_now = {}                  # drone -> t
        self._det_t = {}                     # drone -> 마지막 카메라 프레임 수신 시각 (BLIND 판정)
        self._start_t = now()                # 이 노드 생성 시각. 첫 프레임을 받기 전 BLIND 유예의 기준점
        self._ever_air = set()               # 이륙 이력이 있는 드론. "공중일 수 없음" 판단(퇴역 클리어)에 쓴다
        self._mm_found = False               # 미션 마커 확정 여부 사본 (_on_det 콜백 스레드가 참조)

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

        # ── 대시보드용 미션 상태 발행부 ──────────────────────────────────────
        # BT 는 로봇 행동만 담당한다는 원칙에 따라, 여기서 하는 일은 "이미 블랙보드에
        # 있는 미션 상태를 그대로 직렬화해 한 토픽으로 내보내는" 얇은 발행뿐이다.
        # 별도 관제 로직·의사결정은 없다. 관람자 대시보드(ros_io.py)가 이 토픽 하나만
        # 구독해 신호색·후레쉬·캐러셀·팝업·상태칩을 그린다.
        #   토픽 : mission_state_topic (기본 /coshow/mission_state)
        #   타입 : std_msgs/String (JSON) — 대시보드 _validate_protocol 스키마에 맞춘다
        #   QoS  : latched(TRANSIENT_LOCAL/RELIABLE/depth 1) — 늦게 붙은 대시보드도 즉시 최신 상태 수신
        # mission_state_topic 을 null/'' 로 두면 발행하지 않는다(순수 실기 운용 시 끄기 가능).
        self._mission_pub = None
        topic = C.get('mission_state_topic', '/coshow/mission_state')
        if topic:
            self._mission_pub = node.create_publisher(
                String, topic,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._rescue_limo = C.get('rescue_limo') or (list(LIMOS)[-1] if LIMOS else 'limo_b')

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

            # 검출 확정: 관심 ID(미션 범위 또는 target) 하나를 추적.
            # 분류는 기체가 아니라 "국면" 기준이다: MM 확정 전의 현재 미션 담당만 미션 마커를
            # 찾고, 그 외(확정 후의 미션기 포함)는 전부 타겟을 찾는다. 기체 기준으로 가르면
            # 미션기가 탐색에 투입되거나 탐색기가 미션을 승계했을 때 타겟 검출이
            # 미션 필터에 삼켜져 IsTMFound 가 영영 서지 않는다.
            #
            # 확정 기준: "연속 N프레임" 이 아니라 "최근 confirm_window_sec 초 안에 confirm_frames
            # 프레임 이상". AI-deck 스트림은 프레임이 불규칙해 한 장만 놓쳐도 연속 카운트가
            # 리셋됐는데(뷰어엔 보여도 BT 는 확정 못 함), 슬라이딩 윈도우는 중간 유실에 강하다.
            # confirm_window_sec 를 0(또는 미설정)으로 두면 예전처럼 "연속" 으로 동작한다(폴백).
            lo, hi = C['mission_marker_ids']
            interest = None
            on_mission = (drone == ROLE['observe'] and not self._mm_found)
            if on_mission:
                cand = [i for i in ids if lo <= i <= hi]
                interest = cand[0] if cand else None
            elif tid is not None and tid in ids:
                interest = tid
            need = int(C['confirm_frames'])
            win = float(C.get('confirm_window_sec', 0.0))
            confirmed = False
            if interest is None:
                # 이 프레임엔 관심 마커가 없다.
                #   윈도우 방식(win>0): 리셋하지 않는다 — 순간 유실이 누적을 지우면 안 된다.
                #                       오래된 기록은 아래에서 시간으로 자연 만료된다.
                #   연속 방식(win<=0): 끊긴 것이므로 리셋 (연속의 정의).
                if win <= 0:
                    self._hits[drone] = (None, [])
            else:
                # 이 프레임의 추정 위치와 "드론 바로 아래로부터의 수평 거리"(off)를 기록한다.
                # 크기기반 역투영의 x,y 오차는 이 수평 거리에 비례한다(드론이 마커 바로
                # 위일수록 정확). 그래서 확정 시 윈도우 안에서 off 가 가장 작은 프레임을
                # 고른다(방식 B). 픽셀 중심 근접과 동치이면서 이미지 크기 설정이 필요 없다.
                m = next((k for k in msg.markers if k.id == interest), None)
                entry = None
                if m is not None:
                    dp = msg.drone_pose.pose.position
                    wx, wy = float(m.world_x), float(m.world_y)
                    entry = (t, wx, wy, float(dp.z), _dist2(wx, wy, dp.x, dp.y))
                prev_id, hist = self._hits.get(drone, (None, []))
                if prev_id != interest:
                    hist = []                       # 다른 ID 로 바뀌면 그 ID 의 기록만 새로 센다
                if win > 0:
                    hist = [e for e in hist if t - e[0] <= win]   # 윈도우 밖 오래된 것 제거
                if entry is not None:
                    hist.append(entry)
                if win <= 0:
                    hist = hist[-need:]             # 폴백(연속): 마지막 need 개만
                self._hits[drone] = (interest, hist)
                confirmed = len(hist) >= need
            if confirmed:
                # 방식 B: 윈도우 안에서 드론 바로 아래에 가장 가까운(=x,y 오차 최소) 프레임 채택.
                # (예전엔 확정시키는 마지막 한 프레임만 썼다. 그 한 장이 마커가 화면
                #  가장자리에 걸린 프레임이면 크기기반 오차가 그대로 P_N 에 박혔다.)
                # z 는 선택된 프레임의 드론 고도 — 수평 위치엔 안 쓰이고 CatchTarget 이
                # 어차피 호버 고도로 덮는다.
                hist = self._hits[drone][1]
                if on_mission:
                    # 미션마커: pose 는 이후 안 쓰이고 id 만 target_id 로 쓴다. (0,0) 도 후보에
                    # 남긴다 — 마커가 원점에 있으면 역투영도 (0,0)이라 가드를 두면 확정을 놓친다.
                    best = min(hist, key=lambda e: e[4])
                    self._mission_event = {'drone': drone, 'id': interest, 't': t,
                                           'pose': {'x': best[1], 'y': best[2], 'z': best[3]}}
                else:
                    # 타겟마커: P_N 으로 이동해야 하므로 역투영 실패(0,0) 프레임은 후보에서 뺀다.
                    valid = [e for e in hist if e[1] != 0.0 or e[2] != 0.0]
                    if valid:
                        best = min(valid, key=lambda e: e[4])
                        self._target_events.append({'drone': drone, 'id': interest, 't': t,
                                                    'pose': {'x': best[1], 'y': best[2], 'z': best[3]}})
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
        # 기체 명단. observe 는 승계(Observer)로, searchers 는 미션기 탐색 투입(Search)으로
        # 실행 중 바뀔 수 있다 — 원본은 ROLE, XML 의 robot_key/robots_key 는 여기 사본을 읽는다.
        bb['observe'] = ROLE['observe']
        searchers = list(SEARCHERS)
        if ROLE['obs_joined'] and ROLE['observe'] not in searchers:
            searchers.append(ROLE['observe'])   # 투입된 미션기: 복귀 대상·finder 후보에 포함
        bb['searchers'] = searchers
        bb['all_drones'] = list(DRONES)
        bb['retired'] = set(ROLE['retired'])    # _drone_home 등이 모든 국면에서 보도록 여기서 기록

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

        # 이륙 이력 기록. 퇴역 처리에서 "한 번도 안 떴으면 공중일 수 없다" 판단에 쓴다.
        for d in DRONES:
            p = bb['pose'].get(d)
            if p is not None and p['z'] >= float(TOL['airborne_z']):
                self._ever_air.add(d)
        bb['ever_airborne'] = set(self._ever_air)

        # 필수 pose 수신 확인. 사전점검에서 결함 승인된(DEGRADED) 기체는 pose 가 영영 없을 수
        # 있으므로 기다리지 않는다 — 안 빼면 BT 가 시작조차 못 한다.
        # 리모는 기본적으로 게이트에서 뺀다: BT 는 리모 pose 내용을 쓰지 않고(리모 도착·복귀는
        # Nav2 액션 결과 limo_arrived 로 판정), 이 항목은 리모 연결 라이브니스용일 뿐이다.
        # 나중에 리모 odom 수신까지 시작 조건에 넣고 싶으면 config 에 require_limo_pose: true.
        required = [d for d in DRONES if d not in DEGRADED]
        if C.get('require_limo_pose', False):
            required += list(LIMOS)
        if any(r not in bb['pose'] for r in required):
            missing = [r for r in required if r not in bb['pose']]
            bb['missing_pose'] = missing
            self._publish_mission_state(bb)
            return False
        bb['missing_pose'] = []

        # [기체 상태] 판정만 기록. 대응(퇴역·비상착륙·구역 이어받기)은 Search 가 한다.
        self._update_health(bb, t)

        # [미션 마커 확정] 게이팅: 발신=현재 미션 담당, ID 범위(콜백에서 필터),
        # 그 기체가 관측점에 도착한 이후 스탬프. 승계 시 새 담당의 도착 기록으로 판정된다.
        if mission_ev and not bb['mission_marker']['found']:
            arr = bb['drone_arrived'].get(ROLE['observe'])
            ok = arr is not None and _dist2(arr['goal'][0], arr['goal'][1], OBS['x'], OBS['y']) < 0.3 \
                and mission_ev['t'] >= arr['t']
            if ok:
                bb['mission_marker'] = dict(mission_ev, found=True)
                bb['target_id'] = int(mission_ev['id']) - int(C['marker_id_offset'])
                self._mm_found = True          # 콜백 라우팅 전환: 이후 미션기 검출도 타겟으로 분류

        # [타겟 마커] 발신 ∈ searchers(투입된 미션기 포함), id == target_id (콜백 필터). 최초 1회만 latch
        for ev in target_evs:
            if ev['drone'] not in bb['searchers'] and ev['drone'] != bb.get('finder'):
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

        # [탐색 전환 플래그] 미션을 마친 미션 담당이 착륙 대신 탐색으로 넘어갈지.
        # 반드시 이번 tick 의 mission/target 확정 결과를 반영한 뒤 계산한다 — 앞쪽에서 계산하면
        # 미션마커가 확정되는 tick 에 한 박자 늦어, 복귀 관문이 미션기를 잠깐 기지로 보내는
        # 잘못된 명령이 한 번 나간다(실기에서 엉뚱한 방향으로 출발). 여기서 계산해 그 틈을 없앤다.
        # 미션 마커 확정 ~ 타겟 발견 전(탐색 국면)에만 참. 이때 복귀 관문은 이 드론을
        # 착륙시키지 않고(_return_satisfied), Search 가 공중 그대로 탐색 진입점으로 보낸다.
        # 타겟이 발견되면(구조 국면) 다시 False → 최종 전원 복귀에서 정상 착륙한다.
        obs = ROLE['observe']
        searching = bb['mission_marker'].get('found', False) and not bb['target_marker'].get('found', False)
        will_search = False
        if searching and obs not in ROLE['retired']:
            if obs in SEARCHERS:
                will_search = True
            elif bool(C.get('search', {}).get('observe_join', True)) \
                    and len([d for d in SEARCHERS if d not in ROLE['retired']]) < len(SEARCHERS):
                will_search = True
        bb['observe_will_search'] = will_search
        self._publish_mission_state(bb)
        return True

    # ---- 대시보드용 미션 상태 발행 (행동 결정 없음, 블랙보드 직렬화만) ----
    def _derive_phase(self, bb):
        """블랙보드 상태 조합으로 국면을 도출한다. BT 는 phase 변수를 따로 두지 않으므로
        (mission_marker/target_marker/rescue_done + 이착륙·리모 도착) 으로 계산한다.
        대시보드 라벨/캐러셀/신호색이 쓰는 값과 동일한 이름을 낸다."""
        if bb.get('missing_pose'):
            return 'waiting_poses'
        airborne_z = float(TOL['airborne_z'])

        def airborne(r):
            p = bb.get('pose', {}).get(r)
            return p is not None and p.get('z', 0.0) >= airborne_z

        mm = bb.get('mission_marker', {}).get('found', False)
        tm = bb.get('target_marker', {}).get('found', False)
        rescue_done = float(bb.get('rescue_done_t', 0.0) or 0.0) > 0.0

        if rescue_done:
            return 'return' if any(airborne(d) for d in DRONES) else 'done'
        if tm:
            rl = self._rescue_limo
            arr = bb.get('limo_arrived', {}).get(rl)
            pn = bb.get('P_N')
            if arr and pn and _dist2(arr['goal'][0], arr['goal'][1],
                                     pn['x'], pn['y']) <= float(TOL['limo_at']) + 0.1:
                return 'rescue'                       # 구조 리모가 P_N 도착
            c = bb.get('cmd', {}).get(rl)
            if c and c.get('kind') == 'nav':
                return 'rescue_dispatch'              # 구조 리모 출동 명령 나감
            return 'capture'                          # 타겟 발견, 위치 확인 중
        if mm:
            searchers = bb.get('searchers', [])
            return 'search' if any(airborne(d) for d in searchers) else 'handover'
        return 'observe'                              # 미션 마커 판독 전

    def _publish_mission_state(self, bb):
        if self._mission_pub is None:
            return
        mm = bb.get('mission_marker', {})
        payload = {
            't': float(bb.get('now', now())),
            'phase': self._derive_phase(bb),
            'led': {r: c for r, c in (bb.get('led') or {}).items() if isinstance(c, str)},
            'mission_marker_id': int(mm['id']) if mm.get('found') and mm.get('id') is not None else None,
            'target_id': int(bb['target_id']) if bb.get('target_id') is not None else None,
            'finder': bb.get('finder'),
            'target_confirmed': bool(bb.get('target_confirmed', False)),
            'preflight_required': bool((C.get('preflight') or {}).get('gate', False)),
            'preflight_ready': True,   # 트리 tick 시점은 사전점검 게이트 통과 이후
            'rescue_done_t': float(bb.get('rescue_done_t', 0.0) or 0.0),
            'missing_pose': [str(r) for r in bb.get('missing_pose', [])],
        }
        note = bb.get('target_confirm_note')
        if note:
            payload['target_confirm_note'] = str(note)
        pn = bb.get('P_N')
        if isinstance(pn, dict) and all(
                isinstance(pn.get(k), (int, float)) and math.isfinite(pn.get(k)) for k in ('x', 'y', 'z')):
            payload['P_N'] = {'x': float(pn['x']), 'y': float(pn['y']), 'z': float(pn['z'])}
        cmd = {}
        for r, c in (bb.get('cmd') or {}).items():
            if not isinstance(c, dict) or 'kind' not in c:
                continue
            goal = c.get('goal')
            goal = ([float(g) for g in goal]
                    if isinstance(goal, (list, tuple))
                    and all(isinstance(g, (int, float)) and math.isfinite(g) for g in goal)
                    else [])
            cmd[r] = {'kind': str(c['kind']), 'goal': goal, 't': float(c.get('t', bb.get('now', now())))}
        if cmd:
            payload['cmd'] = cmd
        try:
            msg = String()
            msg.data = json.dumps(payload, allow_nan=False)
            self._mission_pub.publish(msg)
        except (ValueError, TypeError):
            pass   # 발행 실패는 로봇 행동에 영향 없음 — 조용히 다음 tick 재시도

    # ---- 기체 상태 판정 ----
    def _update_health(self, bb, t):
        """bb['health'][d] ∈ {OK, LOST, STUCK, BLIND, DEGRADED}. 판정만 하고 명령은 내지 않는다.

        DEGRADED: 사전점검 게이트에서 운영자가 결함을 알고 승인한 기체 (t=0 부터 영구).

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
        # PreflightReady(별도 노드)가 카메라 프레임 수신을 읽을 수 있게 노출.
        # 값이 없는 드론은 아직 프레임을 한 장도 못 받은 것. start_t 도 함께 준다.
        bb['cam_seen'] = det_t
        bb['cam_start_t'] = self._start_t
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
        # 사전점검 게이트에서 결함을 안고 계속하기로 승인된 기체: 유예(LOST 5s·BLIND 8s) 없이
        # 첫 tick 부터 확정한다. 매 tick 같은 값을 돌려주므로 회복 로직도 타지 않는다 (영구).
        if d in DEGRADED:
            return 'DEGRADED', '사전점검 결함 승인 (' + ', '.join(DEGRADED[d]) + ')'
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

        # BLIND: 검출 노드는 "실제 카메라 프레임을 받았을 때만" marker_detections 를 발행한다
        # (마커 0개여도 발행 = 영상은 흐름 = 정상). 따라서 '발행 수신 시각(det_t)' 이 곧 영상 흐름이다.
        #   - 발행 이력 있음: 마지막 발행 후 frame_max_age 넘게 조용하면 BLIND (도중 끊김)
        #   - 발행 이력 없음: 노드 시작 후 blind_startup_grace 넘게 첫 발행이 없으면 BLIND
        #     (노드 자체가 안 떴거나 카메라 연결이 안 된 경우. 첫 프레임까지의 지연은 유예로 흡수)
        fmax = float(h.get('frame_max_age', 0.0))
        if fmax > 0:
            last = det_t.get(d)
            if last is not None:
                if t - last > fmax:
                    return 'BLIND', f'카메라 {t - last:.1f} s 끊김'
            else:
                grace = float(h.get('blind_startup_grace', 8.0))
                if t - self._start_t > grace:
                    return 'BLIND', f'카메라 프레임 없음 ({t - self._start_t:.1f} s 동안 0장)'
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
                     pose_max_age=0.8, pose_tolerance=0.30, pose_stable_time=2.0,
                     status_max_age=1.0, cam_max_age=2.0, cam_startup_grace=8.0)

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
        self.cam_max_age = float(pf['cam_max_age'])            # 카메라 프레임 신선도(초)
        self.cam_startup_grace = float(pf['cam_startup_grace'])  # 시작 후 첫 프레임까지 유예(초)
        # 사전점검 게이트에서 결함 승인된 기체는 검사에서 뺀다. 안 빼면 그 기체가 영영 통과를
        # 못 해 여기서 전체가 멈춘다 — 승인의 의미(자동 재배치로 계속)가 무효가 된다.
        self.drones = [d for d in DRONES if d not in DEGRADED]
        if DEGRADED:
            print('[PRE-FLIGHT] 결함 승인 기체 제외: ' + ', '.join(sorted(DEGRADED)), flush=True)
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

    def _camera_ready(self, drone, bb, t):
        # 영상 검출이 살아있는지. 검출 노드는 실제 프레임을 받았을 때만 marker_detections 를
        # 발행하므로(UpdateBlackboard 가 bb['cam_seen'] 에 그 수신 시각을 적는다), 발행이 곧 영상이다.
        # 프레임을 한 장도 못 받았으면 시작 후 cam_startup_grace 까지 기다렸다가 실패로 본다.
        seen = bb.get('cam_seen', {}).get(drone)
        if seen is None:
            ref = bb.get('cam_start_t', t)
            waited = t - ref
            if waited > self.cam_startup_grace:
                return False, f'no camera ({waited:.1f}s)'
            return False, f'waiting camera {waited:.1f}/{self.cam_startup_grace:.0f}s'
        age = t - seen
        if age > self.cam_max_age:
            return False, f'camera stale {age:.1f}s'
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
                c_ok, c_why = self._camera_ready(d, bb, t)
                all_ok = all_ok and k_ok and p_ok and s_ok and c_ok
                lines.append('[PRE-FLIGHT] {}: kalman={}{} | pose={}{} | supervisor={}{} | camera={}{}'.format(
                    d, k_ok, self._why(k_ok, k_why), p_ok, self._why(p_ok, p_why),
                    s_ok, self._why(s_ok, s_why), c_ok, self._why(c_ok, c_why)))
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


def _return_satisfied(bb, robot):
    """복귀 관문(AreDronesReturn/ReturnDrones) 충족 여부.

    미션을 마친 뒤 탐색으로 전환하는 미션 담당(bb['observe_will_search'])은 착륙시키지 않는다 —
    공중 그대로 Search 가 탐색 진입점으로 몰고 간다. 착륙→재이륙 전이에서 나던 이륙 실패 방지.
    그 외에는 실제로 기지에 착륙(_drone_home)해야 충족이다."""
    if robot == ROLE['observe'] and bb.get('observe_will_search'):
        return True
    return _drone_home(bb, robot)


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
    """robots="cf230,cf231" / robots_key="finder" / exclude_key="finder" 조합. 목록이 비면 SUCCESS.

    latch="true": 한 번 SUCCESS 가 되면 계속 SUCCESS. 미션기 착륙 게이트 전용 —
    미션기가 탐색에 투입되어 재이륙하면 이 조건이 도로 FAILURE 가 되면서 짝인 ReturnDrones 가
    "복귀해!" 를 다시 보내 Search 의 탐색 명령과 같은 기체를 두고 싸운다. latch 가 그걸 끊는다.
    (임무 마지막의 전원 복귀 인스턴스에는 달지 않는다 — 거기는 매 tick 재확인이 맞다.)
    """

    def __init__(self, name, agent, robots='', robots_key=None, exclude_key=None, latch=None, **kw):
        super().__init__(name, agent, robots=robots, robots_key=robots_key, exclude_key=exclude_key, **kw)
        self.latch = str(latch).lower() == 'true'
        self._latched = False

    def _check(self, agent, bb):
        if self._latched:
            return Status.SUCCESS
        lst = _robots_from_attrs(bb, self.robots, self.robots_key, self.exclude_key)
        ok = all(_return_satisfied(bb, r) for r in lst)
        if ok and self.latch:
            self._latched = True
        return self._st(ok)


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


def _retired_phase(actor, bb, d, t, h):
    """퇴역 기체 d 의 비상착륙을 진행시키고 현재 국면을 돌려준다. Observer/Search 공용.

    반환:
      'landed'      신선한 pose 로 착륙 확인 (z <= landed_z) — 구역 이양·역할 승계 가능
      'never_flew'  이륙 이력 없음 → 공중일 수 없다 — 즉시 이양·승계 가능
      'airborne'    신선한 pose 인데 아직 공중 — 대기
      'lost'        pose 두절 — 착륙 확인 불가

    land 는 최초 1회 + (pose 가 살아 있는데 land 완료 시간이 지나도 공중이면) stale_warn_period
    주기로 재전송한다 — LOST 였다가 pose 가 공중 상태로 돌아온 기체를 다시 내리기 위해서다.
    전송 기록은 ROLE['land_sent'] 공유라 Observer 와 Search 가 같은 기체에 중복 전송하지 않는다.
    """
    p = bb['pose'].get(d)
    fresh = p is not None and (t - p['t']) <= float(h.get('pose_max_age', 1.0))
    if fresh and p['z'] <= float(TOL['landed_z']):
        return 'landed'
    if d not in bb.get('ever_airborne', ()):
        # 늦게 도착할지 모를 takeoff 에 대비해 land 는 심어두되(무해), 공중일 수는 없으므로 통과
        if d not in ROLE['land_sent'] and actor._land(bb, d):
            ROLE['land_sent'][d] = t
        return 'never_flew'
    sent = ROLE['land_sent'].get(d)
    if sent is None:
        if actor._land(bb, d):
            ROLE['land_sent'][d] = t
    elif fresh and t - sent >= _dur_of('land', d) + float(h.get('stale_warn_period', 5.0)):
        actor._sig.pop(d, None)                  # _send 의 동일 서명 dedup 을 풀고 재전송
        if actor._land(bb, d):
            ROLE['land_sent'][d] = t
    return 'airborne' if fresh else 'lost'


def _lost_clear_ok(h, retired_t, t):
    """LOST 기체의 이양·승계 허용 여부. strict(실기 기본)면 절대 불가 — 착륙 확인이 유일한 열쇠.

    timeout 정책(시뮬)은 퇴역 후 lost_clear_timeout 경과 시 허용한다. 실기에서 이 정책은
    'BT 만 pose 를 잃고 기체는 계속 호버' 인 경우 비행 중인 기체의 구역으로 다른 기체를
    보낼 수 있어 금지한다 (2026-09 실기 리허설에서 실제로 목격된 사고 경로).
    """
    if str(h.get('lost_clear_policy', 'timeout')) != 'timeout':
        return False
    return t - retired_t >= float(h.get('lost_clear_timeout', 12.0))


class Observer(_MultiDroneAction):
    """전 기체 고장 대응 + 미션(관측) 역할 승계. 루트 최상단에서 매 tick 돌고 항상 SUCCESS.

    Search 는 미션 마커 확정 후에야 tick 되므로, 관측 국면(그리고 탐색이 halt 된 구조 국면)에는
    고장에 대응할 주체가 없다. 이 노드가 그 사각을 메운다. 판정은 UpdateBlackboard 의
    bb['health'], 여기는 대응만 한다:

    1) 퇴역: health 가 OK 아닌 기체를 국면과 무관하게 즉시 퇴역(ROLE['retired'])시키고
       비상 land 를 보낸다 (_retired_phase — 착륙할 때까지 재전송 포함).
       "이상 있는 기체는 무조건 확실히 비상착륙" 요구의 집행 지점이다.
       구역 이양은 여기서 하지 않는다 — Search 가 착륙 확인 후에만 한다.
    2) 승계: MM 확정 전에 현재 미션 담당이 퇴역하면, 내려앉음이 확인된 뒤에야
       (착륙 확인 / 이륙 이력 없음 / timeout 정책의 타임아웃 — _lost_clear_ok)
       health OK 인 탐색기 중 슬롯 순서 첫째로 ROLE['observe'] 를 교체한다.
       게이트가 있는 이유: 헌 담당(하강 중)과 새 담당(같은 관측 고도층으로 상승)이
       공중에서 만나지 않게. phase1 은 robot_key="observe" 라 다음 tick 부터 자동 추종.
       승계자는 searchers 에서 빠지지 않으므로 미션 후 자기 구역 탐색으로 복귀한다.
    3) MM 확정 후의 미션기 고장: 역할은 끝났으므로 퇴역+착륙만. retired 는 _drone_home 이
       복귀 완료로 치므로 AreDronesReturn(observe) 게이트가 열려 시나리오가 멈추지 않는다.
    4) 승계 후보 전멸: 주기 경고 후 대기 (운영자 판단 영역. Ctrl+C 전체 착륙은 기존 그대로).

    config: health.mission_takeover=false 면 이 노드 전체 비활성 (판정도 대응도 기존 그대로).
    """

    def __init__(self, name, agent):
        super().__init__(name, agent)
        self.h = C.get('health') or {}
        self.enabled = bool(self.h.get('mission_takeover', True)) and bool(self.h)
        self._warn_t = {}

    def _warn(self, t, key, msg):
        if t - self._warn_t.get(key, -1e9) >= float(self.h.get('stale_warn_period', 5.0)):
            self._warn_t[key] = t
            print(msg, flush=True)

    async def run(self, agent, bb):
        self.status = Status.SUCCESS
        if not self.enabled:
            return self.status
        t = bb['now']
        health = bb.get('health', {})

        # 1) 전 기체 퇴역 판정 + 비상착륙 집행 (모든 국면)
        for d in DRONES:
            if d not in ROLE['retired'] and health.get(d, 'OK') != 'OK':
                ROLE['retired'][d] = t
                why = bb.get('health_reason', {}).get(d, '')
                print(f'[OBSERVER] {d} 퇴역: {health.get(d)}' + (f' ({why})' if why else '')
                      + '. 비상 착륙을 집행한다', flush=True)
        phase = {}
        for d in ROLE['retired']:
            phase[d] = _retired_phase(self, bb, d, t, self.h)

        # 2) 미션 역할 승계 (MM 확정 전, 현 담당이 퇴역했을 때만)
        cur = ROLE['observe']
        if cur not in ROLE['retired'] or bb.get('mission_marker', {}).get('found', False):
            return self.status
        ph = phase.get(cur, 'lost')
        cleared = ph in ('landed', 'never_flew') or \
            (ph == 'lost' and _lost_clear_ok(self.h, ROLE['retired'][cur], t))
        if not cleared:
            self._warn(t, 'settle', f'[OBSERVER] 미션 담당 {cur} 내려앉음 미확인 ({ph}) — '
                                    f'같은 고도층 충돌을 막기 위해 승계를 보류한다')
            return self.status
        cand = next((d for d in SEARCHERS
                     if d not in ROLE['retired'] and health.get(d, 'OK') == 'OK'), None)
        if cand is None:
            self._warn(t, 'cand', '[OBSERVER] 미션 역할 승계 불가 — health OK 인 탐색기가 없다. 대기')
            return self.status
        ROLE['observe'] = cand
        slot = (C.get('slot_of') or {}).get(cand, '')
        print(f'[OBSERVER] 미션 역할 승계: {cur} → {cand}' + (f' ({slot})' if slot else '')
              + '. 승계자는 미션 완수 후 자기 탐색 구역으로 복귀한다', flush=True)
        return self.status


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
    - 클리어된 구역은 생존 참여 기체 중 (보유 구역 수, 구역 중심까지 거리, 목록 순) 최소인
      한 대가 이어받는다 — 부하 우선이라 두 구역이 동시에 비면 한 대가 독식하지 않고 나눈다.
      한 번 정해지면 그 드론이 퇴역하기 전엔 바꾸지 않는다
      (매 tick 재계산하면 두 드론이 번갈아 가까워지며 배정이 흔들린다).
    - 이어받은 드론은 [자기 구역, 이어받은 구역...] 을 순환한다. 현재 구역의 왕복(시작점
      복귀)을 마친 시점에만 다음 구역으로 넘어가므로 하던 일이 끊기지 않는다.
      남의 구역이라도 고도는 지금 나는 드론의 altitudes.search 를 쓴다
      (드론별 고도 분리가 충돌 회피 수단이라 그 층을 유지해야 한다).

    미션기 탐색 투입 (search.observe_join, 기본 true):
    - 생존 탐색기가 정원(len(SEARCHERS)) 미만이고, 현재 미션 담당이 "여분 기체"
      (순수 미션기 — 승계된 탐색기가 아님)이며, health OK 이고, 미션을 끝내고(MM 확정)
      기지에 착륙해 있고, 클리어된(=원 담당의 착륙이 확인된) 구역이 실제로 존재할 때만
      pool 에 합류한다. 매 tick 재평가하므로 탐색 도중의 추가 고장에도 그때 투입된다.
    - 합류 기체는 자기 구역이 없다 — 클리어된 빈 구역만 배분받아 순환한다. 고도는 자기 슬롯의
      altitudes.search (실기: mission 슬롯 전용 층). 죽은 기체의 고도층은 쓰지 않는다.
    - 배정은 _effective_zones 가 매 tick 고정 구역 순번(홈 번호)으로 다시 계산한다(래치 없음).
      미션기 홈=가운데·담당 0 이라 빈 구역을 인접 순으로 먼저 흡수하고, 생존 탐색기는 자기
      구역을 유지한다. 예) 231·232 퇴역·233 생존·230 합류 → 230=[상,중], 233=[하].
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
        self.join_enabled = bool(s.get('observe_join', True))
        # 이번 실행 동안 영구인 상태 (halt 로 초기화하지 않는다).
        # 퇴역(ROLE['retired'])·land 기록(ROLE['land_sent'])은 Observer 와 공유한다.
        self.pool = list(SEARCHERS)   # 탐색 참여 기체. 미션기 투입 시 뒤에 붙는다 (출격 순서 유지)
        self.cleared = set()    # 이어받아도 되는 퇴역 구역 (원 담당 착륙 확인/무이륙/타임아웃 정책)
        self._warn_t = {}
        self._reset_state()

    def _reset_state(self):
        # 미션기(pool 에 있지만 SEARCHERS 아님)는 자기 구역이 없다 → 빈 목록에서 시작
        self.zone_list = {d: ([d] if d in SEARCHERS else []) for d in self.pool}
        self.cyc = {d: 0 for d in self.pool}           # 드론 -> zone_list 안의 현재 위치 (인덱스 순환)
        self.cur_zone = {d: (d if d in SEARCHERS else None) for d in self.pool}
        self.idx = {d: 0 for d in self.pool}           # 현재 구역 경로 안의 웨이포인트
        self.dirn = {d: 1 for d in self.pool}          # 핑퐁 방향
        self.started_t = {}
        self.entered = set()   # 진입점(레인 시작점)에 이미 도달한 기체. 전이 비행 중 지상 이륙 보류에 쓴다

    # ---- 고장 처리 ----
    def _retire_check(self, bb, t):
        health = bb.get('health', {})
        for d in self.pool:
            if d not in ROLE['retired'] and health.get(d, 'OK') != 'OK':
                ROLE['retired'][d] = t
                why = bb.get('health_reason', {}).get(d, '')
                print(f'[SEARCH] {d} 퇴역: {health.get(d)}' + (f' ({why})' if why else '')
                      + '. 구역은 착륙 확인 후에만 다른 드론이 이어받는다', flush=True)
            if d in ROLE['retired']:
                self._handle_retired(bb, d, t)

    def _handle_retired(self, bb, d, t):
        """퇴역 기체의 비상착륙 진행(_retired_phase 공유) + '구역 이양 허용(cleared)' 판정.

        이양은 오직: 착륙 확인 / 이륙 이력 없음 / (timeout 정책일 때만) LOST 타임아웃.
        strict 정책(실기)에서는 공중 이력이 있는 LOST 기체의 구역을 절대 넘기지 않는다."""
        if d in self.cleared:
            return
        ph = _retired_phase(self, bb, d, t, self.h)
        if ph == 'landed':
            self.cleared.add(d)
            print(f'[SEARCH] {d} 착륙 확인 (z={bb["pose"][d]["z"]:.2f} m). 구역 이어받기 허용', flush=True)
            return
        if ph == 'never_flew':
            self.cleared.add(d)
            print(f'[SEARCH] {d} 이륙 이력 없음 → 공중일 수 없어 구역 이어받기 즉시 허용', flush=True)
            return
        if ph == 'lost':
            if _lost_clear_ok(self.h, ROLE['retired'][d], t):
                self.cleared.add(d)
                print(f'[SEARCH WARNING] {d} LOST 로 착륙 미확인. 퇴역 후 '
                      f'{float(self.h.get("lost_clear_timeout", 12.0)):.0f} s 경과 → 내려앉았다고 보고 '
                      f'구역을 넘긴다 (lost_clear_policy: timeout — 시뮬 전용 완화)', flush=True)
            elif t - self._warn_t.get(d, -1e9) >= float(self.h.get('stale_warn_period', 5.0)):
                self._warn_t[d] = t
                print(f'[SEARCH WARNING] {d} LOST + 공중 이력 있음 — 착륙 확인 전까지 구역을 절대 '
                      f'넘기지 않는다 (lost_clear_policy: strict). pose 복귀 대기', flush=True)
            return
        # 신선한 pose 인데 아직 공중: 하강 시간은 기다리고, 그 뒤에도 떠 있으면 주기 경고
        # (land 재전송은 _retired_phase 가 한다)
        settle = _dur_of('land', d) + 2.0
        since_land = t - ROLE['land_sent'].get(d, t)
        if since_land >= settle and t - self._warn_t.get(d, -1e9) >= float(self.h.get('stale_warn_period', 5.0)):
            self._warn_t[d] = t
            print(f'[SEARCH WARNING] {d} 여전히 공중 (z={bb["pose"][d]["z"]:.2f} m). '
                  f'착륙 확인 전까지 구역을 넘기지 않는다', flush=True)

    def _maybe_join_observe(self, bb, t):
        """미션기(여분 기체) 탐색 투입 판정. 조건 전부 충족 시 pool 에 1회 합류시킨다."""
        if not self.join_enabled or ROLE['obs_joined']:
            return
        obs = ROLE['observe']
        if obs in SEARCHERS or obs in ROLE['retired']:
            return                                   # 승계된 탐색기 = 여분 기체 없음 / 미션기도 고장
        if bb.get('health', {}).get(obs, 'OK') != 'OK':
            return
        alive = [d for d in SEARCHERS if d not in ROLE['retired']]
        if len(alive) >= len(SEARCHERS):
            return                                   # 정원 충족 — 투입 불필요
        if not bb.get('mission_marker', {}).get('found', False):
            return                                   # 미션 완수 전에는 투입하지 않는다
        # 착륙을 요구하지 않는다. 미션을 마친 미션기는 착륙하지 않고 공중에서 그대로
        # 탐색 진입 지점(빌린 구역의 레인 시작점 = 그 구역 담당의 스폰 상공)으로 이동한다.
        # 착륙→재이륙 전이에서 실기체 이륙 실패가 나던 것을 원천 차단한다.
        # (오래 착륙해 있던 미션기가 나중에 투입되는 경우엔 지상에서 정상 이륙한다.)
        if not (self.cleared - set(alive)):
            return                                   # 원 담당의 착륙이 확인된(클리어) 구역이 있어야 투입
        ROLE['obs_joined'] = True
        self.pool.append(obs)
        self.zone_list[obs], self.cyc[obs] = [], 0
        self.cur_zone[obs], self.idx[obs], self.dirn[obs] = None, 0, 1
        # 배정은 _effective_zones 가 매 tick 고정 홈 번호로 다시 계산한다(래치 없음). 미션기는
        # 홈=가운데·담당 0 이라 합류 즉시 빈 구역을 인접 순으로 흡수한다 — 별도 재배분 불필요.
        print(f'[SEARCH] 탐색기 {len(alive)}/{len(SEARCHERS)}대 → 미션기 {obs} 탐색 투입 '
              f'(전용 고도 {_alt("search", obs):.1f} m, 클리어 구역만 배분)', flush=True)

    def _effective_zones(self, bb):
        """드론 -> 순환할 구역 id 목록. 고정 구역 순번(인덱스) 기준의 인접 배정.

        실시간 pose 를 쓰지 않는다. 각 드론의 '홈 번호'(탐색기=자기 구역 번호, 투입 미션기=가운데)와
        빈 구역의 번호만 비교해, 번호가 가장 붙은(인접한) 생존 드론이 그 빈 구역을 이어받는다.
        동률이면 담당 구역 적은 드론 → 낮은 홈 번호. 홈도 구역도 같은 공간 순서라, 번호 인접
        배정은 자동으로 연속 블록이 되어 한 드론이 남의 구역을 가로지르지 않는다. 순간 pose
        끊김에 안 흔들리므로 래치(taken_by)도, 자기 구역을 사이에 끼우던 트릭도 불필요하다.

        · 살아있는 탐색기는 자기 구역을 항상 유지.
        · 투입 미션기(자기 구역 없음)는 홈=가운데 + 담당 0으로 시작 → 빈 구역을 먼저 흡수한다.
          빈 구역의 원 담당은 이미 고장이라 생존기가 미션기보다 더 붙을 수 없어(기껏 동률),
          동률은 담당 적은 미션기가 이긴다. 그래서 "빈 구역은 미션기가 먼저" 규칙이 자동 성립한다.
        · 빈 구역은 착륙 확인된(cleared) 것만 이양한다.

        예) cf232(중)·cf233(하) 고장, cf231(상) 생존 + cf230 투입:
          홈 cf231=0, cf230=가운데(1). 중(1)→cf230, 하(2)→cf230(홈1 이 홈0 보다 붙음).
          결과 cf231=[상], cf230=[중,하] — 붙어있는 두 구역만 미션기가 맡아 가로지름 없음.
        """
        alive = [d for d in self.pool if d not in ROLE['retired']]
        if not alive:
            return {}
        # 구역 공간 순번: 중심 y 내림차순(상단이 0). 구역 id = 원 담당 드론 이름.
        order = sorted(self.zones, key=lambda z: _zone_center(self.zones[z])[1], reverse=True)
        zidx = {z: i for i, z in enumerate(order)}
        mid = len(order) // 2
        homes = {d: (zidx[d] if d in zidx else mid) for d in alive}
        counts = {d: 0 for d in alive}
        zones_of = {d: [] for d in alive}
        # 1) 살아있는 탐색기는 자기 구역 유지 (구역 id == 드론 이름)
        for d in alive:
            if d in zidx:
                zones_of[d].append(d)
                counts[d] += 1
        # 2) 빈(고장+클리어) 구역을 번호 인접 생존 드론에게 (동률: 담당 적은 → 낮은 홈)
        for z in order:
            if z in alive or z not in self.cleared:
                continue
            zi = zidx[z]
            helper = min(alive, key=lambda d: (abs(homes[d] - zi), counts[d], homes[d]))
            zones_of[helper].append(z)
            counts[helper] += 1
        # 3) 각 드론의 구역을 공간 순번으로 정렬 → 붙어있는 구역만 번호 순 순환 (가로지름 없음)
        return {d: sorted(zones_of[d], key=lambda z: zidx[z]) for d in alive}

    def _apply_assign(self, bb):
        assign = self._effective_zones(bb)
        for d, lst in assign.items():
            if lst != self.zone_list[d]:
                print(f'[SEARCH] {d} 구역 목록 {self.zone_list[d]} → {lst}', flush=True)
                self.zone_list[d] = list(lst)
                # 목록이 바뀌면 맨 앞 구역부터 다시 순환한다. 배정 변화는 드물어(퇴역·클리어·투입
                # 시점) 진행 중 왕복을 끊는 대가가 작고, 중복 항목이 있어 포인터를 새로 잡아야 한다.
                self.cyc[d] = 0
                self.cur_zone[d] = lst[0] if lst else None
                self.idx[d], self.dirn[d] = 0, 1
        bb['zone_assign'] = {d: list(v) for d, v in assign.items()}

    def _step_waypoint(self, bb, d):
        """공중 기체 d 를 현재 구역의 레인 웨이포인트로 보낸다 (도착 시 다음 점/구역으로 전진)."""
        path = self.paths[self.cur_zone[d]]
        gz = _alt('search', d)                        # 남의 구역이라도 내 고도로 난다
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

    def _at_entry(self, bb, d):
        """공중 기체 d 가 자기 구역의 진입점(레인 시작점 = 담당 스폰 상공)에 도달했는가."""
        ex, ey, _ = self.paths[self.cur_zone[d]][0]
        return self._at(bb, d, ex, ey, None, self.tol)

    async def run(self, agent, bb):
        t = bb['now']
        self._retire_check(bb, t)
        self._maybe_join_observe(bb, t)
        self._apply_assign(bb)
        mm = bb.get('mission_marker', {}).get('found', False)

        # 활성 기체 분류. 퇴역(문제 판정) 기체는 이동·이륙 어느 쪽도 시키지 않는다 (여기서 제외).
        movers, grounded = [], []
        for d in self.pool:
            if d in ROLE['retired']:
                continue                     # 문제 기체: 절대 이동·이륙 금지
            if not self.zone_list.get(d):
                continue                     # 배정 구역 없음(투입 직후 등)
            if d == ROLE['observe'] and not mm:
                # 미션 담당은 미션 마커 확정 전에는 탐색에 끌어들이지 않는다 (phase1 과 명령 충돌 방지).
                continue
            p = bb['pose'].get(d)
            if p is None:
                continue
            (movers if p['z'] >= float(TOL['airborne_z']) else grounded).append(d)

        # ── 1단계: 이동 우선 ── 이미 공중인 기체(미션 마치고 전환 중인 담당 포함)를 먼저 이동시킨다.
        # 이륙하는 기체와 공중에서 만나지 않도록, 전환 비행 중인 기체가 진입점에 닿기 전에는
        # 아래 지상 기체 이륙을 보류한다.
        transit_pending = False
        for d in movers:
            self.started_t.setdefault(d, t)      # 공중 기체는 stagger 없이 즉시
            self._step_waypoint(bb, d)
            if d not in self.entered:
                if self._at_entry(bb, d):
                    self.entered.add(d)          # 진입점 도달 → 전이 완료, 이제 지상 이륙 허용
                else:
                    transit_pending = True       # 아직 진입점으로 이동 중

        # ── 2단계: 이륙 ── 이동 중인 기체가 없을 때만, 지상 기체를 stagger 로 순차 이륙.
        if not transit_pending:
            prev_started = None
            for d in grounded:
                if d not in self.started_t:
                    if prev_started is not None and t - prev_started < self.stagger:
                        break
                    self.started_t[d] = t
                    self.entered.add(d)          # 지상 이륙 기체의 진입점 = 자기 스폰, 전이 불필요
                prev_started = self.started_t[d]
                self._takeoff(bb, d)

        bb['search_progress'] = {d: (self.cur_zone[d], self.idx[d])
                                 for d in self.pool
                                 if d not in ROLE['retired'] and self.cur_zone.get(d)}
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
            if _return_satisfied(bb, d):
                continue                     # 이미 착륙했거나, 탐색 전환 중인 미션기(착륙 불필요)
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
        # bb['searchers'] 는 탐색에 투입된 미션기까지 포함한 동적 명단 (UpdateBlackboard 가 기록)
        cands = [d for d in bb.get('searchers', SEARCHERS) if d not in retired
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
