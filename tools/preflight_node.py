#!/usr/bin/env python3
"""실기체 비행 전 안전 점검 노드.

safe_two_cf_square_final.py 의 pre-flight / in-flight 검사를 ROS 노드로 옮긴 것.
기체 수를 고정하지 않고, 검사 결과를 토픽 하나로 알린다.

  /preflight/ready   std_msgs/Bool   (transient_local, 1 Hz)

BT 의 IsReady 조건이 이 토픽을 보고 트리 전체를 막는다. True 가 되기 전에는
어떤 이륙·이동 명령도 나가지 않는다.

검사 순서
  ① 서버 준비    crazyflie_server 의 로깅 초기화 (add_logging 서비스 + kalman 스트림)
  ② 추정기 초기화 kalman.resetEstimation = 1
  ③ 준비 관문     기체마다 아래 셋을 동시에 통과할 때까지 대기
       칼만 수렴   최근 N 샘플의 분산 폭 < threshold
       위치 일치   추정 위치가 기대 초기위치와 tolerance 이내로 stable_time 유지
       슈퍼바이저  TUMBLED/LOCKED 아님, 전원 정상, 무장 가능
  ④ 무장         arm(True) 후 IS_ARMED + CAN_FLY 확인
  ⑤ 비행 중 감시  통신 신선도, 자세 이상, 기체 간 최소거리

④ 까지 통과하면 ready=True. ⑤ 에서 위반이 생기면 ready=False 로 내리고 착륙시킨다.
ready 가 False 가 되면 BT 는 그 tick 부터 명령을 멈추므로, 착륙 명령과 다투지 않는다.

실행:
  python3 tools/preflight_node.py --ros-args \
      -p drones:="['cf230','cf231','cf232','cf233']" \
      -p expected_x:="[-1.0,-1.5,-1.5,-1.5]" \
      -p expected_y:="[ 0.0, 1.5, 0.0,-1.5]"
"""
import math
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool

from crazyflie_interfaces.msg import LogDataGeneric, Status
from crazyflie_interfaces.srv import AddLogging, Arm, Land


class Preflight(Node):

    def __init__(self):
        super().__init__('preflight')

        self.declare_parameter('drones', ['cf230', 'cf231', 'cf232', 'cf233'])
        # 기대 초기위치. crazyflies.yaml 의 initial_position 과 같아야 한다.
        # drones 와 같은 순서로 짝을 이룬다.
        self.declare_parameter('expected_x', [-1.0, -1.5, -1.5, -1.5])
        self.declare_parameter('expected_y', [0.0, 1.5, 0.0, -1.5])

        self.declare_parameter('kalman_history_len', 10)
        self.declare_parameter('kalman_threshold', 0.001)
        self.declare_parameter('pose_tolerance', 0.10)
        self.declare_parameter('pose_stable_time', 2.0)
        self.declare_parameter('server_ready_timeout', 10.0)
        self.declare_parameter('preflight_timeout', 35.0)
        self.declare_parameter('arm_timeout', 4.0)
        self.declare_parameter('pose_max_age', 0.8)
        self.declare_parameter('status_max_age', 1.0)
        self.declare_parameter('min_interdrone_distance', 0.45)
        self.declare_parameter('land_height', 0.05)
        self.declare_parameter('land_duration', 4.0)
        # 비행 중 감시를 끌 수 있게 해 둔다. 지상 시험 때 유용하다.
        self.declare_parameter('monitor_inflight', True)

        g = lambda k: self.get_parameter(k).value          # noqa: E731
        self.names = list(g('drones'))
        ex, ey = list(g('expected_x')), list(g('expected_y'))
        if len(ex) != len(self.names) or len(ey) != len(self.names):
            raise RuntimeError(
                'expected_x/expected_y 길이가 drones 와 다르다: '
                '{} vs {}/{}'.format(len(self.names), len(ex), len(ey)))
        self.expected = {n: (float(ex[i]), float(ey[i])) for i, n in enumerate(self.names)}

        self.hist_len = int(g('kalman_history_len'))
        self.kal_thr = float(g('kalman_threshold'))
        self.pose_tol = float(g('pose_tolerance'))
        self.pose_stable = float(g('pose_stable_time'))
        self.t_server = float(g('server_ready_timeout'))
        self.t_pre = float(g('preflight_timeout'))
        self.t_arm = float(g('arm_timeout'))
        self.pose_max_age = float(g('pose_max_age'))
        self.status_max_age = float(g('status_max_age'))
        self.min_dist = float(g('min_interdrone_distance'))
        self.land_h = float(g('land_height'))
        self.land_d = float(g('land_duration'))
        self.monitor = bool(g('monitor_inflight'))

        self.cb = ReentrantCallbackGroup()
        self.pose = {}      # name -> (PoseStamped, 수신시각)
        self.status = {}    # name -> (Status, 수신시각)
        self.kal = {n: [deque(maxlen=self.hist_len) for _ in range(3)] for n in self.names}
        self.pose_good_since = {n: None for n in self.names}
        self.low_power_warned = set()

        for n in self.names:
            self.create_subscription(PoseStamped, '/{}/pose'.format(n),
                                     lambda m, r=n: self._on_pose(r, m), 10,
                                     callback_group=self.cb)
            self.create_subscription(Status, '/{}/status'.format(n),
                                     lambda m, r=n: self._on_status(r, m), 10,
                                     callback_group=self.cb)
            self.create_subscription(LogDataGeneric, '/{}/kalman_variance'.format(n),
                                     lambda m, r=n: self._on_kalman(r, m), 10,
                                     callback_group=self.cb)

        # BT 가 늦게 떠도 마지막 값을 받도록 transient_local.
        self.pub_ready = self.create_publisher(
            Bool, '/preflight/ready',
            QoSProfile(depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.ready = False
        self._publish_ready()
        self.create_timer(1.0, self._publish_ready, callback_group=self.cb)

        self.cli_params = self.create_client(
            SetParameters, '/crazyflie_server/set_parameters', callback_group=self.cb)
        self.cli_arm = {n: self.create_client(Arm, '/{}/arm'.format(n),
                                              callback_group=self.cb)
                        for n in self.names}
        self.cli_land = {n: self.create_client(Land, '/{}/land'.format(n),
                                               callback_group=self.cb)
                         for n in self.names}
        self.cli_addlog = {n: self.create_client(AddLogging, '/{}/add_logging'.format(n),
                                                 callback_group=self.cb)
                           for n in self.names}

    # ── 콜백 ──
    def _on_pose(self, name, msg):
        self.pose[name] = (msg, time.monotonic())

    def _on_status(self, name, msg):
        self.status[name] = (msg, time.monotonic())

    def _on_kalman(self, name, msg):
        if len(msg.values) < 3:
            return
        for axis in range(3):
            self.kal[name][axis].append(float(msg.values[axis]))

    def _publish_ready(self):
        self.pub_ready.publish(Bool(data=bool(self.ready)))

    def _set_ready(self, value, reason=''):
        if self.ready != value:
            self.ready = value
            self.get_logger().info(
                '[READY] {} {}'.format('True' if value else 'False', reason))
        self._publish_ready()

    # ── 관문 ──
    def _kal_ranges(self, name):
        h = self.kal[name]
        if any(len(a) < self.hist_len for a in h):
            return None
        return tuple(max(a) - min(a) for a in h)

    def _pose_ok(self, name):
        entry = self.pose.get(name)
        if not entry or time.monotonic() - entry[1] > self.pose_max_age:
            self.pose_good_since[name] = None
            return False, None
        p = entry[0].pose.position
        ex, ey = self.expected[name]
        err = (abs(p.x - ex), abs(p.y - ey), abs(p.z))
        inside = all(e <= self.pose_tol for e in err)
        now = time.monotonic()
        if inside:
            if self.pose_good_since[name] is None:
                self.pose_good_since[name] = now
        else:
            self.pose_good_since[name] = None
        stable = (self.pose_good_since[name] is not None
                  and now - self.pose_good_since[name] >= self.pose_stable)
        return stable, err

    def _supervisor_ok(self, name):
        entry = self.status.get(name)
        if not entry or time.monotonic() - entry[1] > self.status_max_age:
            return False, 'no fresh /status'
        s = entry[0]
        info = int(s.supervisor_info)
        if info & Status.SUPERVISOR_INFO_IS_TUMBLED:
            return False, 'IS_TUMBLED'
        if info & Status.SUPERVISOR_INFO_IS_LOCKED:
            return False, 'IS_LOCKED'
        if int(s.pm_state) == Status.PM_STATE_SHUTDOWN:
            return False, 'PM_STATE_SHUTDOWN'
        if info & Status.SUPERVISOR_INFO_AUTO_ARM:
            return True, 'auto-arm'
        if not (info & (Status.SUPERVISOR_INFO_CAN_BE_ARMED
                        | Status.SUPERVISOR_INFO_IS_ARMED)):
            return False, 'manual-arm: 무장 불가'
        return True, 'manual-arm'

    def _postarm_ok(self, name):
        entry = self.status.get(name)
        if not entry or time.monotonic() - entry[1] > self.status_max_age:
            return False
        info = int(entry[0].supervisor_info)
        return (bool(info & Status.SUPERVISOR_INFO_IS_ARMED)
                and bool(info & Status.SUPERVISOR_INFO_CAN_FLY)
                and not (info & Status.SUPERVISOR_INFO_IS_TUMBLED)
                and not (info & Status.SUPERVISOR_INFO_IS_LOCKED))

    # ── 단계 ──
    def wait_server_ready(self):
        self.get_logger().info('[1/4] crazyflie_server 로깅 초기화 대기...')
        deadline = time.monotonic() + self.t_server
        while time.monotonic() < deadline:
            svc = all(c.service_is_ready() for c in self.cli_addlog.values())
            stream = all(len(self.kal[n][0]) >= 3 for n in self.names)
            if svc and stream:
                time.sleep(0.5)      # 파라미터 콜백 등록까지 여유
                self.get_logger().info('[1/4] PASS: 서버 준비 완료')
                return True
            time.sleep(0.1)
        self.get_logger().error(
            '[1/4] FAIL: add_logging={} kalman_stream={}. '
            'crazyflies.yaml 의 custom_topics 와 기체 연결을 확인하라.'.format(
                all(c.service_is_ready() for c in self.cli_addlog.values()),
                all(len(self.kal[n][0]) >= 3 for n in self.names)))
        return False

    def reset_estimators(self):
        self.get_logger().info('[2/4] 칼만 추정기 초기화...')
        for n in self.names:
            for a in self.kal[n]:
                a.clear()
            self.pose_good_since[n] = None
        if not self.cli_params.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('[2/4] FAIL: set_parameters 서비스 없음')
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name='all.params.kalman.resetEstimation',
            value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1))]
        fut = self.cli_params.call_async(req)
        deadline = time.monotonic() + 3.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not fut.done() or fut.result() is None:
            self.get_logger().error('[2/4] FAIL: resetEstimation 응답 없음')
            return False
        bad = [r.reason for r in fut.result().results if not r.successful]
        if bad:
            self.get_logger().error('[2/4] FAIL: {}'.format(bad))
            return False
        time.sleep(0.3)
        for n in self.names:            # 초기화 전환 중 샘플 폐기
            for a in self.kal[n]:
                a.clear()
        self.get_logger().info('[2/4] PASS: 추정기 초기화 완료')
        return True

    def wait_preflight(self):
        self.get_logger().info('[3/4] 전 기체 준비 관문 대기...')
        start = time.monotonic()
        last = 0.0
        while time.monotonic() - start < self.t_pre:
            lines, all_ok = [], True
            for n in self.names:
                rng = self._kal_ranges(n)
                kal_ok = rng is not None and all(r < self.kal_thr for r in rng)
                pose_ok, err = self._pose_ok(n)
                sup_ok, sup_why = self._supervisor_ok(n)
                all_ok = all_ok and kal_ok and pose_ok and sup_ok
                lines.append(
                    '  {}: kalman={} {} | pose={} {} | supervisor={} ({})'.format(
                        n, kal_ok,
                        'samples...' if rng is None else
                        '({:.6f},{:.6f},{:.6f})'.format(*rng),
                        pose_ok,
                        'no pose' if err is None else
                        'err=({:.3f},{:.3f},{:.3f})'.format(*err),
                        sup_ok, sup_why))
            if all_ok:
                self.get_logger().info('[3/4] PASS: 전 기체 준비 완료\n' + '\n'.join(lines))
                return True
            now = time.monotonic()
            if now - last >= 1.0:
                self.get_logger().info('\n'.join(lines))
                last = now
            time.sleep(0.05)
        self.get_logger().error(
            '[3/4] FAIL: {:.0f}초 안에 수렴하지 못했다'.format(self.t_pre))
        return False

    def arm_all(self):
        self.get_logger().info('[4/4] 무장 요청...')
        for n, cli in self.cli_arm.items():
            if not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().error('[4/4] FAIL: {} arm 서비스 없음'.format(n))
                return False
            req = Arm.Request()
            req.arm = True
            cli.call_async(req)
        deadline = time.monotonic() + self.t_arm
        while time.monotonic() < deadline:
            if all(self._postarm_ok(n) for n in self.names):
                self.get_logger().info('[4/4] PASS: IS_ARMED + CAN_FLY 확인')
                return True
            time.sleep(0.05)
        for n in self.names:
            req = Arm.Request()
            req.arm = False
            self.cli_arm[n].call_async(req)
        self.get_logger().error('[4/4] FAIL: 무장 확인 실패. 해제했다.')
        return False

    # ── 비행 중 감시 ──
    def check_inflight(self):
        pos = []
        for n in self.names:
            st = self.status.get(n)
            if not st or time.monotonic() - st[1] > self.status_max_age:
                return '{}: /status 끊김'.format(n)
            pe = self.pose.get(n)
            if not pe or time.monotonic() - pe[1] > self.pose_max_age:
                return '{}: /pose 끊김'.format(n)
            info = int(st[0].supervisor_info)
            if info & Status.SUPERVISOR_INFO_IS_TUMBLED:
                return '{}: IS_TUMBLED'.format(n)
            if info & Status.SUPERVISOR_INFO_IS_LOCKED:
                return '{}: IS_LOCKED'.format(n)
            if not (info & Status.SUPERVISOR_INFO_CAN_FLY):
                return '{}: CAN_FLY=False'.format(n)
            if int(st[0].pm_state) == Status.PM_STATE_SHUTDOWN:
                return '{}: PM_STATE_SHUTDOWN'.format(n)
            if int(st[0].pm_state) == Status.PM_STATE_LOW_POWER:
                # 임무를 중단하지는 않는다. 추력 여유가 준다는 경고만 한 번.
                if n not in self.low_power_warned:
                    self.low_power_warned.add(n)
                    self.get_logger().warning(
                        '{}: LOW_POWER (vbat={:.2f}V). 계속 진행하되 추력 여유가 줄었다.'
                        .format(n, float(st[0].battery_voltage)))
            p = pe[0].pose.position
            pos.append((p.x, p.y, p.z))
        for i in range(len(pos)):
            for j in range(i + 1, len(pos)):
                d = math.dist(pos[i], pos[j])
                if d < self.min_dist:
                    return '{}-{} 거리 {:.2f} m < {:.2f} m'.format(
                        self.names[i], self.names[j], d, self.min_dist)
        return None

    def land_all(self, reason):
        self.get_logger().error('[LAND] {} -> 전 기체 착륙'.format(reason))
        for n, cli in self.cli_land.items():
            req = Land.Request()
            req.group_mask = 0
            req.height = self.land_h
            req.duration.sec = int(self.land_d)
            req.duration.nanosec = int((self.land_d - int(self.land_d)) * 1e9)
            cli.call_async(req)

    # ── 본체 ──
    def run(self):
        if not (self.wait_server_ready() and self.reset_estimators()
                and self.wait_preflight() and self.arm_all()):
            self.get_logger().error(
                '점검 실패. ready 를 올리지 않는다. BT 는 시작하지 않는다.')
            return                       # 실패 시 정지 (재시도 없음)

        self._set_ready(True, '전 점검 통과. 이륙 가능.')
        if not self.monitor:
            self.get_logger().info('비행 중 감시 꺼짐 (monitor_inflight=false)')
            return
        self.get_logger().info('비행 중 감시 시작 (최소거리 {:.2f} m)'.format(self.min_dist))
        while rclpy.ok():
            why = self.check_inflight()
            if why:
                # ready=False -> BT 가 그 tick 부터 명령을 멈춘다. 그 뒤 착륙시킨다.
                self._set_ready(False, why)
                self.land_all(why)
                return
            time.sleep(0.1)


def main():
    rclpy.init()
    node = Preflight()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    import threading
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.run()
        while rclpy.ok():            # ready 상태를 계속 발행하며 대기
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
