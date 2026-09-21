#!/usr/bin/env python3
"""
fleet_manager_node.py — 중앙 PC(도메인 33)에서 실행하는 팀 경계 계층 (계약의 위쪽).

역할: BT 와 리모측(relay) 사이를 매개하는 layer 경계.
  - 위쪽(BT 대면): BT 가 이미 쓰는 인터페이스를 그대로 제공 → BT 무수정
      액션 서버  /{robot_id}/navigate_to_pose   (BT 가 직접 호출)
      토픽       /{robot_id}/odom, /{robot_id}/amcl_pose  (BT 가 구독)
  - 아래쪽(리모측 대면): relay 가 노출한 경계 인터페이스로 통과
      액션 클라이언트 /{robot_id}/{iface_ns}/navigate_to_pose
      토픽 구독       /{robot_id}/{iface_ns}/odom, /{robot_id}/{iface_ns}/amcl_pose
  - 추가로 estop 과 status 를 제공 (골 경로 정중앙에 있어 취소가 relay->nav2 까지 전파됨)

나중에 리모 부분을 다른 팀이 맡으면, fleet_manager 위쪽(BT 대면) 계약은 그대로 두고
아래쪽(/{id}/{iface_ns}/...)만 그 팀의 구현으로 갈아끼우면 된다. relay 는 그 임시 구현.

인터페이스
  BT → fleet
    액션 /{id}/navigate_to_pose 호출 (NavigateToPose)
    /fleet/estop            std_msgs/Empty   전 로봇 골 취소
  fleet → BT
    /{id}/odom, /{id}/amcl_pose  (relay 값을 그대로 재노출)
    /fleet/{id}/nav_state   std_msgs/String(JSON)  {robot,state,distance_remaining,stamp} (latched)
    /fleet/status           std_msgs/String(JSON)  1 Hz, 전 로봇 online/상태 요약

전 기체 ROS_DOMAIN_ID=33 동일. 중앙에서 하나만 띄우면 robots 목록의 모든 리모를 한꺼번에 처리.

실행 예
  export ROS_DOMAIN_ID=33
  python3 fleet_manager_node.py --ros-args -p robots:="[limo_a, limo_b]"
"""

import json
import threading
import time
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

_STATUS_NAME = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')

        self.declare_parameter('robots', ['limo_a', 'limo_b'])
        self.declare_parameter('iface_ns', '_raw')
        self.declare_parameter('odom_timeout', 3.0)  # 초, 넘게 odom 없으면 offline

        self.robots = list(self.get_parameter('robots').value)
        self.iface = self.get_parameter('iface_ns').value
        self.odom_timeout = float(self.get_parameter('odom_timeout').value)

        self.cbg = ReentrantCallbackGroup()
        self._r = {}
        for rid in self.robots:
            self._setup_robot(rid)

        self.sub_estop = self.create_subscription(
            Empty, '/fleet/estop', self._on_estop, 10, callback_group=self.cbg)
        self.pub_status = self.create_publisher(String, '/fleet/status', QOS_LATCHED)
        self.create_timer(1.0, self._publish_status, callback_group=self.cbg)

        self.get_logger().info(
            f"fleet_manager up. robots={self.robots}, iface_ns='{self.iface}'")

    # ------------------------------------------------------------------ setup
    def _down(self, rid, leaf):
        """리모측(아래) 이름."""
        return f'/{rid}/{self.iface}/{leaf}' if self.iface else f'/{rid}/{leaf}'

    def _setup_robot(self, rid):
        d = {
            'client': ActionClient(
                self, NavigateToPose, self._down(rid, 'navigate_to_pose'),
                callback_group=self.cbg),
            'client_gh': None,          # relay 로 넘긴 골 핸들 (estop 취소용)
            'state': 'IDLE',
            'distance_remaining': None,
            'last_odom_mono': None,
        }
        # 위쪽(BT 대면) 토픽 재노출
        d['pub_odom'] = self.create_publisher(Odometry, f'/{rid}/odom', 10)
        d['pub_amcl'] = self.create_publisher(
            PoseWithCovarianceStamped, f'/{rid}/amcl_pose', QOS_LATCHED)
        d['pub_state'] = self.create_publisher(
            String, f'/fleet/{rid}/nav_state', QOS_LATCHED)

        # 아래쪽(리모측) 토픽 구독 → 위로 재노출 + liveness
        self.create_subscription(
            Odometry, self._down(rid, 'odom'),
            partial(self._on_down_odom, rid), 10, callback_group=self.cbg)
        self.create_subscription(
            PoseWithCovarianceStamped, self._down(rid, 'amcl_pose'),
            partial(self._on_down_amcl, rid), QOS_LATCHED, callback_group=self.cbg)

        # 위쪽(BT 대면) 액션 서버
        d['server'] = ActionServer(
            self, NavigateToPose, f'/{rid}/navigate_to_pose',
            execute_callback=partial(self._execute, rid),
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        self._r[rid] = d
        self._publish_state(rid)

    # ------------------------------------------------------------------ util
    def _publish_state(self, rid, detail=''):
        d = self._r[rid]
        msg = String()
        msg.data = json.dumps({
            'robot': rid,
            'state': d['state'],
            'distance_remaining': d['distance_remaining'],
            'detail': detail,
            'stamp': time.time(),
        })
        d['pub_state'].publish(msg)

    def _is_online(self, rid):
        t = self._r[rid]['last_odom_mono']
        return t is not None and (time.monotonic() - t) < self.odom_timeout

    # ------------------------------------------------------- 아래 → 위 재노출
    def _on_down_odom(self, rid, msg):
        self._r[rid]['last_odom_mono'] = time.monotonic()
        self._r[rid]['pub_odom'].publish(msg)

    def _on_down_amcl(self, rid, msg):
        self._r[rid]['pub_amcl'].publish(msg)

    # ------------------------------------------------- BT 액션 → relay 통과
    def _execute(self, rid, server_gh):
        d = self._r[rid]
        goal = server_gh.request

        if not d['client'].wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[{rid}] relay 액션 서버 없음 -> abort")
            d['state'] = 'ABORTED'
            self._publish_state(rid, 'relay action server unavailable')
            server_gh.abort()
            return NavigateToPose.Result()

        d['state'], d['distance_remaining'] = 'NAVIGATING', None
        self._publish_state(rid)
        self.get_logger().info(
            f"[{rid}] goal -> relay: "
            f"({goal.pose.pose.position.x:.2f}, {goal.pose.pose.position.y:.2f})")

        done = threading.Event()
        state = {'client_gh': None, 'result': None, 'status': None}

        def on_feedback(fb_msg):
            d['distance_remaining'] = round(
                float(fb_msg.feedback.distance_remaining), 3)
            self._publish_state(rid)
            server_gh.publish_feedback(fb_msg.feedback)

        send_future = d['client'].send_goal_async(goal, feedback_callback=on_feedback)

        def on_goal_response(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                state['status'] = GoalStatus.STATUS_ABORTED
                done.set()
                return
            state['client_gh'] = gh
            d['client_gh'] = gh
            gh.get_result_async().add_done_callback(on_result)

        def on_result(rf):
            wrapped = rf.result()
            state['result'] = wrapped.result
            state['status'] = wrapped.status
            done.set()

        send_future.add_done_callback(on_goal_response)

        # estop(client_gh 취소) 또는 BT 취소(server 취소) 모두 여기서 관측
        cancel_sent = False
        while not done.wait(timeout=0.1):
            if (server_gh.is_cancel_requested and not cancel_sent
                    and state['client_gh'] is not None):
                state['client_gh'].cancel_goal_async()
                cancel_sent = True

        d['client_gh'] = None
        status = state['status']
        result = state['result'] if state['result'] is not None else NavigateToPose.Result()
        d['state'] = _STATUS_NAME.get(status, 'ABORTED')
        if d['state'] == 'SUCCEEDED':
            d['distance_remaining'] = 0.0
        self._publish_state(rid)
        self.get_logger().info(f"[{rid}] goal 종료 status={status} -> {d['state']}")

        if status == GoalStatus.STATUS_SUCCEEDED:
            server_gh.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            server_gh.canceled()
        else:
            server_gh.abort()
        return result

    # ------------------------------------------------------------------ estop
    def _on_estop(self, _msg):
        self.get_logger().warn("ESTOP -- 전 로봇 골 취소")
        for rid, d in self._r.items():
            if d['client_gh'] is not None:
                self._publish_state(rid, 'estop')
                d['client_gh'].cancel_goal_async()

    # ------------------------------------------------------------------ status
    def _publish_status(self):
        robots = {}
        for rid, d in self._r.items():
            robots[rid] = {
                'online': self._is_online(rid),
                'state': d['state'],
                'distance_remaining': d['distance_remaining'],
            }
        msg = String()
        msg.data = json.dumps({'stamp': time.time(), 'robots': robots})
        self.pub_status.publish(msg)


def main():
    rclpy.init()
    node = FleetManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
