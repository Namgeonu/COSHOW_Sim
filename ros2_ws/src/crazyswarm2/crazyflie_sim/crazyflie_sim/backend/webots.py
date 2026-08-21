"""Webots physics backend for crazyflie_sim (COSHOW, CrazyChoir 이식판).

RPM 변환 경로를 버리고, SIL의 control_t(thrust/roll/pitch/yaw 원출력)를 직접 받아
CrazyChoir(OPT4SMART)의 검증된 모터 명령 로직을 그대로 적용한다:
  - 믹싱: power_distribution_quadrotor.c 방식
  - cmd_yaw = -control.yaw  (Webots 좌표계 보정, CrazyChoir가 발견한 핵심)
  - scaling = 800
  - 모터 부호: m1(-) m2(+) m3(-) m4(+)

이 방식은 서버가 backend.step()에 SIL 객체(cfs)를 넘겨줘야 한다.
(crazyflie_server.py 231행 수정 필요 - 함께 제공)

전제 PROTO: CrazyChoir와 동일 (thrust ±4e-05 [m1,m3 음 / m2,m4 양], torque 2.4e-06 균일)
전제 server.yaml: controller pid
"""

from __future__ import annotations

import math
import os
import sys

import numpy as np
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from ..sim_data_types import Action, State

SCALING = 800.0   # CrazyChoir와 동일 (PWM/rad 스케일)


def _rotmat_to_quat(m):
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = m
    tr = r00 + r11 + r22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return [0.25 * s, (r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s]
    if r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2
        return [(r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s]
    if r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2
        return [(r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s]
    s = math.sqrt(1.0 + r22 - r00 - r11) * 2
    return [(r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s]


class Backend:
    """Webots 물리 백엔드 (CrazyChoir 모터 명령 이식)."""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)

        webots_home = os.environ.get('WEBOTS_HOME', '/usr/local/webots')
        py_path = os.path.join(webots_home, 'lib', 'controller', 'python')
        if py_path not in sys.path:
            sys.path.append(py_path)
        try:
            from controller import Supervisor  # noqa: PLC0415
        except ImportError as e:
            raise RuntimeError(
                'Webots controller 모듈을 찾을 수 없습니다. WEBOTS_HOME 등 설정 확인.') from e

        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())

        self._nodes = {}
        self._custom_fields = {}
        for name, state in zip(names, states):
            n = self.robot.getFromDef(name)
            if n is None:
                raise RuntimeError(f'월드에서 DEF "{name}"를 찾지 못했습니다.')
            n.getField('translation').setSFVec3f(
                [float(state.pos[0]), float(state.pos[1]), max(float(state.pos[2]), 0.015)])
            n.resetPhysics()
            self._nodes[name] = n
            self._custom_fields[name] = n.getField('customData')

        self.robot.step(self.timestep)
        self.node.get_logger().info(
            f'[webots backend] connected (CrazyChoir-port, /{SCALING}). '
            f'basicTimeStep={self.timestep}ms, robots={names}')

    def time(self) -> float:
        return self.robot.getTime()

    def set_controllers(self, cfs):
        """서버가 SIL 객체(control_t 접근용)를 등록."""
        self._cfs = cfs

    def step(self, states_desired, actions):
        self._dbg = getattr(self, '_dbg', 0) + 1
        do_dbg = (self._dbg % 25 == 0)

        # 서버의 self.cfs를 최초 1회 자동 등록 (서버 수정 불필요)
        if not hasattr(self, '_cfs'):
            self._cfs = getattr(self.node, 'cfs', None)
        cfs = self._cfs

        # ① SIL control_t (thrust/roll/pitch/yaw) -> CrazyChoir 믹싱 -> customData
        for i, name in enumerate(self.names):
            if cfs is not None:
                ctrl = list(cfs.values())[i].control
                cmd_thrust = float(ctrl.thrust)
                cmd_roll = float(ctrl.roll)
                cmd_pitch = float(ctrl.pitch)
                cmd_yaw = -float(ctrl.yaw)   # CrazyChoir 핵심: yaw 반전
            else:
                cmd_thrust = cmd_roll = cmd_pitch = cmd_yaw = 0.0

            # power_distribution_quadrotor.c mixing
            p1 = cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
            p2 = cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
            p3 = cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
            p4 = cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw
            # 모터 부호 (CrazyChoir): m1(-) m2(+) m3(-) m4(+)
            w = [-p1 / SCALING, p2 / SCALING, -p3 / SCALING, p4 / SCALING]

            self._custom_fields[name].setSFString(
                f'{w[0]:.4f},{w[1]:.4f},{w[2]:.4f},{w[3]:.4f}')

            if do_dbg and i == 0:
                n0 = self._nodes[name]
                o = n0.getOrientation()
                v6 = n0.getVelocity()
                roll = math.degrees(math.atan2(o[7], o[8]))
                pitch = math.degrees(-math.asin(max(-1.0, min(1.0, o[6]))))
                yawd = math.degrees(math.atan2(o[3], o[0]))
                self.node.get_logger().info(
                    f'[dbg] t={self.time():.2f} cmd=[T{cmd_thrust:.0f},R{cmd_roll:.1f},'
                    f'P{cmd_pitch:.1f},Y{cmd_yaw:.1f}] '
                    f'pos={[round(float(vv), 3) for vv in n0.getPosition()]} '
                    f'rpy=[{roll:.1f},{pitch:.1f},{yawd:.1f}] wz={v6[5]:.2f}')

        # ② 물리 스텝
        if self.robot.step(self.timestep) == -1:
            self.node.get_logger().warn('[webots backend] Webots가 종료되었습니다.')

        # ③ 상태 회수 -> SIL 피드백 (CrazyChoir와 동일: attitude/gyro 방식)
        next_states = []
        for name in self.names:
            n = self._nodes[name]
            state = State()
            state.pos = np.array(n.getPosition())
            R = np.array(n.getOrientation()).reshape(3, 3)
            state.quat = np.array(_rotmat_to_quat(n.getOrientation()))
            vel6 = n.getVelocity()
            state.vel = np.array(vel6[0:3])
            state.omega = R.T @ np.array(vel6[3:6])
            next_states.append(state)

        # ④ /clock
        clock_message = Clock()
        t = self.time()
        clock_message.clock.sec = int(t)
        clock_message.clock.nanosec = int((t - int(t)) * 1e9)
        self.clock_publisher.publish(clock_message)

        return next_states

    def shutdown(self):
        pass
