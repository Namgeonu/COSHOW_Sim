"""Pose topic visualization for crazyflie_sim (COSHOW).

실물 crazyflie_server는 firmware_logging 설정으로 {name}/pose (PoseStamped)를
10Hz 발행하지만, sim 서버에는 이 발행이 없다 (TF만 있음).
mission_controller가 pose 스트림으로 도착 판정을 하는 설계이므로,
이 플러그인이 실물과 동일한 토픽/타입/주기로 그 갭을 메꾼다.

server.yaml 예시:
  sim:
    visualizations:
      pose:
        enabled: true
        frequency: 10   # Hz (실물 crazyflies.yaml의 pose frequency와 맞출 것)
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from ..sim_data_types import Action, State


class Visualization:
    """실물 서버의 {name}/pose 발행을 시뮬에서 재현한다."""

    def __init__(
        self,
        node: Node,
        params: dict,
        names: list[str],
        states: list[State],
        reference_frames: list[str] = None,
    ):
        self.node = node
        self.names = names
        self.reference_frames = reference_frames if reference_frames else ['world'] * len(names)
        self.frequency = float(params.get('frequency', 10))
        self.period = 1.0 / self.frequency
        self.last_pub_t = -1e9

        # 실물 서버와 동일하게 서버 노드 아래 상대 이름으로 발행 -> /cf_a/pose 등
        self.pubs = {
            name: node.create_publisher(PoseStamped, f'{name}/pose', 10)
            for name in names
        }

    def step(self, t, states: list[State], states_desired: list[State], actions: list[Action]):
        if t - self.last_pub_t < self.period:
            return
        self.last_pub_t = t

        sec = math.floor(t)
        nanosec = int((t - sec) * 1e9)
        for name, state, frame in zip(self.names, states, self.reference_frames):
            msg = PoseStamped()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.header.frame_id = frame
            msg.pose.position.x = float(state.pos[0])
            msg.pose.position.y = float(state.pos[1])
            msg.pose.position.z = float(state.pos[2])
            # State.quat = [w, x, y, z]
            msg.pose.orientation.w = float(state.quat[0])
            msg.pose.orientation.x = float(state.quat[1])
            msg.pose.orientation.y = float(state.quat[2])
            msg.pose.orientation.z = float(state.quat[3])
            self.pubs[name].publish(msg)

    def shutdown(self):
        pass
