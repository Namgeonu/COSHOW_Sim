#!/usr/bin/env python3
"""
ns_relay_node.py — 각 LIMO 실기체에서 실행하는 리모측 인터페이스 노드 (팀 경계의 아래쪽).

하는 일: 로봇 내부의 이름 없는 스택을  /{robot_id}/{iface_ns}/...  로 노출한다.
  토픽 재발행 (로봇 → 네트워크)
    /amcl_pose  →  /{robot_id}/{iface_ns}/amcl_pose   (latched)
    /odom       →  /{robot_id}/{iface_ns}/odom
  액션 통과 (양방향)
    네트워크 /{robot_id}/{iface_ns}/navigate_to_pose (서버) ⇄ 로컬 /navigate_to_pose (클라이언트)
    goal / feedback / result / cancel 를 그대로 중계

경계 이름
  iface_ns 기본값 '_raw'. 이 접두가 곧 fleet_manager 와 리모측 사이의 계약 이름이다.
  나중에 리모 팀과 합의된 이름으로 코드 수정 없이 -p iface_ns:=<이름> 으로 바꿀 수 있다.

전제 (도메인 통일 상태에서 이름 충돌을 막는 핵심)
  - 드라이버 + nav2 는 로봇 내부에만 존재하도록 localhost 격리로 실행:
      Foxy/Humble : export ROS_LOCALHOST_ONLY=1
      Iron/Jazzy  : export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  - 이 relay 노드만 격리 없이 실행 → loopback 으로 로컬 스택과 통신,
    네트워크로는 /{robot_id}/{iface_ns}/... 만 노출. 전 기체 ROS_DOMAIN_ID=33 동일.

두 리모는 완전히 동일한 파일을 실행한다. 다른 것은 robot_id 파라미터뿐.
  리모 A:  python3 ns_relay_node.py --ros-args -p robot_id:=limo_a
  리모 B:  python3 ns_relay_node.py --ros-args -p robot_id:=limo_b

실행 예 (LIMO A)
  export ROS_DOMAIN_ID=33
  export ROS_LOCALHOST_ONLY=1          # 벤더 스택 격리용 (드라이버/nav2 띄우는 터미널)
  python3 ns_relay_node.py --ros-args -p robot_id:=limo_a
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

QOS_LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class NsRelay(Node):
    def __init__(self):
        super().__init__('ns_relay')

        self.declare_parameter('robot_id', 'limo_a')
        self.declare_parameter('iface_ns', '_raw')
        self.declare_parameter('local_amcl_topic', 'amcl_pose')
        self.declare_parameter('local_odom_topic', 'odom')
        self.declare_parameter('local_nav_action', 'navigate_to_pose')

        rid = self.get_parameter('robot_id').value
        iface = self.get_parameter('iface_ns').value
        prefix = f'/{rid}/{iface}' if iface else f'/{rid}'
        self.rid = rid

        self.cbg = ReentrantCallbackGroup()

        # ---- 토픽 재발행: 로컬 → 경계 이름 ----
        local_amcl = self.get_parameter('local_amcl_topic').value
        local_odom = self.get_parameter('local_odom_topic').value

        self.pub_amcl = self.create_publisher(
            PoseWithCovarianceStamped, f'{prefix}/amcl_pose', QOS_LATCHED)
        self.create_subscription(
            PoseWithCovarianceStamped, local_amcl,
            lambda m: self.pub_amcl.publish(m), 10, callback_group=self.cbg)

        self.pub_odom = self.create_publisher(Odometry, f'{prefix}/odom', 10)
        self.create_subscription(
            Odometry, local_odom,
            lambda m: self.pub_odom.publish(m), 10, callback_group=self.cbg)

        # ---- 액션 통과 ----
        local_action = self.get_parameter('local_nav_action').value
        self.local_client = ActionClient(
            self, NavigateToPose, local_action, callback_group=self.cbg)
        self.server = ActionServer(
            self, NavigateToPose, f'{prefix}/navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=lambda _req: GoalResponse.ACCEPT,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self.cbg)

        self.get_logger().info(
            f"[{rid}] relay up. local amcl='{local_amcl}', odom='{local_odom}', "
            f"action='{local_action}'  ->  network prefix='{prefix}'")

    # -------------------------------------------------- 액션 통과 실행 콜백
    def _execute(self, server_gh):
        goal = server_gh.request

        if not self.local_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[{self.rid}] 로컬 nav2 액션 서버 없음 -> abort")
            server_gh.abort()
            return NavigateToPose.Result()

        done = threading.Event()
        state = {'client_gh': None, 'result': None, 'status': None}

        def on_feedback(fb_msg):
            server_gh.publish_feedback(fb_msg.feedback)

        send_future = self.local_client.send_goal_async(
            goal, feedback_callback=on_feedback)

        def on_goal_response(fut):
            gh = fut.result()
            if gh is None or not gh.accepted:
                state['status'] = GoalStatus.STATUS_ABORTED
                done.set()
                return
            state['client_gh'] = gh
            result_future = gh.get_result_async()

            def on_result(rf):
                wrapped = rf.result()
                state['result'] = wrapped.result
                state['status'] = wrapped.status
                done.set()

            result_future.add_done_callback(on_result)

        send_future.add_done_callback(on_goal_response)

        cancel_sent = False
        while not done.wait(timeout=0.1):
            if (server_gh.is_cancel_requested and not cancel_sent
                    and state['client_gh'] is not None):
                self.get_logger().info(f"[{self.rid}] cancel -> 로컬 nav2 골 취소")
                state['client_gh'].cancel_goal_async()
                cancel_sent = True

        status = state['status']
        result = state['result'] if state['result'] is not None else NavigateToPose.Result()
        if status == GoalStatus.STATUS_SUCCEEDED:
            server_gh.succeed()
        elif status == GoalStatus.STATUS_CANCELED:
            server_gh.canceled()
        else:
            server_gh.abort()
        self.get_logger().info(f"[{self.rid}] 액션 종료 status={status}")
        return result


def main():
    rclpy.init()
    node = NsRelay()
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
