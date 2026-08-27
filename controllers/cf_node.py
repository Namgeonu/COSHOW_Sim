"""COSHOW 방향2 1단계: 드론별 제어 노드 (webots_ros2_driver 플러그인).

CrazyChoir 뼈대 + 우리 검증 자산(CrazyflieSIL, dbg22 믹싱, 시정수)을 결합.
각 드론이 별도 프로세스로 이 플러그인을 실행 → cffirmware static 독립.

매 스텝:
  1. 센서(GPS/InertialUnit/Gyro) 읽기 → State(pos, vel, quat, omega)
  2. sil.setState(state)           # 물리 상태를 firmware에
  3. sil.getSetpoint()             # planner에서 현시점 목표
  4. sil.executeController()       # PID (이 프로세스 static → 독립!)
  5. 믹싱(yaw반전, /800) + 시정수   # dbg22 로직 + Gazebo 시정수
  6. 자기 모터 setVelocity

1단계 범위: cf_a 단독 비행 검증. goTo/takeoff를 노드가 직접 서비스로 받음(서버 없이).
서버 중계는 2단계.

월드 설정: cf_a controller "<extern>", 이 플러그인을 webots_ros2_driver로 로드.
PYTHONPATH 필요:
  - <COSHOW>/crazyflie-firmware/build           (cffirmware)
  - <COSHOW>/ros2_ws/install/crazyflie_sim/lib/python3.12/site-packages  (CrazyflieSIL)
"""
import math
import os
import socket
import struct
import sys

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time as RosTime
from rclpy.node import Node

# --- 경로 설정 (환경변수 or 기본값) ---
_COSHOW = os.path.expanduser('~/COSHOW')
_FW = os.path.join(_COSHOW, 'crazyflie-firmware', 'build')
import glob as _glob
_sim_cand = _glob.glob(os.path.join(_COSHOW, 'ros2_ws', 'install', 'crazyflie_sim',
                                    'lib', 'python3.*', 'site-packages'))
_SIM = _sim_cand[0] if _sim_cand else os.path.join(
    _COSHOW, 'ros2_ws', 'install', 'crazyflie_sim', 'lib',
    f'python{sys.version_info.major}.{sys.version_info.minor}', 'site-packages')
for _p in (_FW, _SIM):
    if _p not in sys.path:
        sys.path.append(_p)

import cffirmware  # noqa: E402
from crazyflie_sim.crazyflie_sil import CrazyflieSIL  # noqa: E402
from crazyflie_sim.sim_data_types import State  # noqa: E402
from crazyflie_interfaces.srv import Takeoff, GoTo, Land  # noqa: E402
from geometry_msgs.msg import PoseStamped  # noqa: E402

SCALING = 800.0        # dbg22와 동일 (control -> 각속도)
TAU_UP = 0.0125        # 모터 시정수 (Gazebo, 가속)
TAU_DOWN = 0.025       # 모터 시정수 (Gazebo, 감속)


def _quat_from_euler(roll, pitch, yaw):
    """xyz 오일러(rad) -> 쿼터니언 [w,x,y,z] (CrazyflieSIL State 규약)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]


class CfNode:
    """webots_ros2_driver가 로드하는 로봇별 제어 플러그인."""

    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0
        self.robot_name = self.robot.getName()   # 예: "cf_a"

        # --- ROS 노드 ---
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node(f'{self.robot_name}_ctrl')
        ns = self.robot_name

        # --- 모터 (dbg22 부호: m1(-) m2(+) m3(-) m4(+)) ---
        self.motors = []
        for i in (1, 2, 3, 4):
            m = self.robot.getDevice(f'm{i}_motor')
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
            self.motors.append(m)
        self.filtered = [0.0, 0.0, 0.0, 0.0]   # 시정수 필터 상태

        # --- 센서 ---
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        self.imu = self.robot.getDevice('inertial_unit')   # 우리 PROTO 이름
        self.imu.enable(self.timestep)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)
        # --- 카메라 (방향 검증용, enable만) ---
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(self.timestep * 16)   # 2ms*16=32ms (~31fps)
        self.cam_interval = 16
        self._cam_tick = 0
        # --- UDP 스트리밍 (AI-deck 프로토콜 호환) ---
        # 드론 이름 -> deck 포트 (cf_a=6001 ... cf_d=6004)
        deck_port = 6000 + (ord(self.robot_name[-1]) - ord('a') + 1)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_sock.bind(('0.0.0.0', deck_port))
        self.udp_sock.setblocking(False)
        self.udp_client = None   # FER 프로브 보낸 뷰어 주소

        self.past_pos = None
        self.past_t = self.robot.getTime()

        # --- 초기 위치 (GPS 첫 읽기 전엔 0) ---
        init_pos = [0.0, 0.0, 0.0]

        # --- 우리 검증된 SIL 인스턴스 1개 (이 프로세스 static 독립) ---
        # init_pos는 임시(0,0,0). 첫 step에서 GPS 실제 위치로 SIL을 재초기화한다.
        self.sil = CrazyflieSIL(
            self.robot_name, init_pos, 'pid', self.robot.getTime)
        self._pos_initialized = False   # 첫 GPS로 SIL 위치 보정 여부

        # --- 내부 인터페이스 (A-1: 서버가 바깥 이름을 가지고, 여기로 전달) ---
        # 바깥 /{ns}/takeoff 는 coshow_server 가 제공 -> /{ns}/_impl/takeoff 로 전달됨.
        # (서버 없이 직접 테스트하려면 아래 _impl 을 떼면 됨)
        self.node.create_service(Takeoff, f'/{ns}/_impl/takeoff', self._takeoff_cb)
        self.node.create_service(GoTo, f'/{ns}/_impl/go_to', self._goto_cb)
        self.node.create_service(Land, f'/{ns}/_impl/land', self._land_cb)
        # pose 는 드라이버가 직접 발행 (실물 서버와 동일한 /{ns}/pose 이름)
        self.pose_pub = self.node.create_publisher(PoseStamped, f'/{ns}/pose', 10)

        self.node.get_logger().info(
            f'[{self.robot_name}] cf_node init OK '
            f'(SIL+sensors+PID+mixing+tau, /{SCALING}, dt={self.dt})')

    # ---- 서비스 콜백: SIL planner에 목표 ----
    def _takeoff_cb(self, request, response):
        duration = request.duration.sec + request.duration.nanosec / 1e9
        self.sil.takeoff(request.height, duration, request.group_mask)
        self.node.get_logger().info(
            f'[{self.robot_name}] takeoff h={request.height} d={duration}')
        return response

    def _goto_cb(self, request, response):
        duration = request.duration.sec + request.duration.nanosec / 1e9
        self.sil.goTo([request.goal.x, request.goal.y, request.goal.z],
                      request.yaw, duration, request.relative, request.group_mask)
        self.node.get_logger().info(
            f'[{self.robot_name}] goTo '
            f'({request.goal.x},{request.goal.y},{request.goal.z}) d={duration}')
        return response

    def _land_cb(self, request, response):
        duration = request.duration.sec + request.duration.nanosec / 1e9
        self.sil.land(request.height, duration, request.group_mask)
        return response

    # ---- 매 스텝 ----
    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        # 1. 센서 읽기 → State
        pos = np.array(self.gps.getValues())
        # InertialUnit getQuaternion: [x, y, z, w] (Webots 규약)
        # CrazyflieSIL State.quat: [w, x, y, z] (setState에서 quat[0]=w)
        q_webots = self.imu.getQuaternion()    # [x, y, z, w]
        quat = np.array([q_webots[3], q_webots[0], q_webots[1], q_webots[2]])  # -> [w,x,y,z]
        rpy = self.imu.getRollPitchYaw()       # 디버그용 (rad)
        omega = np.array(self.gyro.getValues())  # body frame 각속도 (Gyro)
        vel = np.array(self.gps.getSpeedVector())  # Webots가 직접 주는 속도 (dbg22 getVelocity 대체)
        t = self.robot.getTime()

        state = State()
        state.pos = pos
        state.vel = vel
        state.quat = quat
        state.omega = omega

        # 첫 스텝: SIL의 cmdHl_pos를 실제 GPS 위치로 보정 (takeoff 기준점)
        if not self._pos_initialized:
            self.sil.cmdHl_pos = cffirmware.mkvec(
                float(pos[0]), float(pos[1]), float(pos[2]))
            self.sil.state.position.x = float(pos[0])
            self.sil.state.position.y = float(pos[1])
            self.sil.state.position.z = float(pos[2])
            self._pos_initialized = True
            self.node.get_logger().info(
                f'[{self.robot_name}] SIL 위치 보정: '
                f'({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})')

        # 2. SIL에 물리 상태 반영
        self.sil.setState(state)

        # 3. planner에서 현시점 setpoint
        self.sil.getSetpoint()

        # 4. PID 실행 (이 프로세스 static → 독립)
        self.sil.executeController()

        # 5. dbg22 믹싱 + 시정수
        ctrl = self.sil.control
        cmd_thrust = float(ctrl.thrust)
        cmd_roll = float(ctrl.roll)
        cmd_pitch = float(ctrl.pitch)
        cmd_yaw = -float(ctrl.yaw)   # dbg22 핵심: yaw 반전

        p1 = cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        p2 = cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        p3 = cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        p4 = cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw
        target = [-p1 / SCALING, p2 / SCALING, -p3 / SCALING, p4 / SCALING]

        # 시정수 1차 필터 (각속도에, up/down 구분)
        for i in range(4):
            tau = TAU_UP if abs(target[i]) > abs(self.filtered[i]) else TAU_DOWN
            alpha = min(self.dt / tau, 1.0)
            self.filtered[i] += (target[i] - self.filtered[i]) * alpha
            self.motors[i].setVelocity(self.filtered[i])

        # --- 디버그 로그 (25스텝마다, dbg22 형식) ---
        self._dbg = getattr(self, '_dbg', 0) + 1
        if self._dbg % 25 == 0:
            self.node.get_logger().info(
                f'[dbg] t={t:.2f} '
                f'cmd=[T{cmd_thrust:.0f},R{cmd_roll:.1f},P{cmd_pitch:.1f},Y{cmd_yaw:.1f}] '
                f'pos=[{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}] '
                f'rpy=[{math.degrees(rpy[0]):.1f},{math.degrees(rpy[1]):.1f},{math.degrees(rpy[2]):.1f}] '
                f'wz={omega[2]:.2f} '
                f'w=[{self.filtered[0]:.2f},{self.filtered[1]:.2f},{self.filtered[2]:.2f},{self.filtered[3]:.2f}]')

        # 6. pose 발행
        ps = PoseStamped()
        # Webots 시뮬 시각으로 스탬프 (/clock 과 동일 기준, gt_odom 과 같은 패턴)
        _t = self.robot.getTime()
        ps.header.stamp = RosTime(sec=int(_t), nanosec=int((_t - int(_t)) * 1e9))
        ps.header.frame_id = 'world'
        ps.pose.position.x = float(pos[0])
        ps.pose.position.y = float(pos[1])
        ps.pose.position.z = float(pos[2])
        q = state.quat
        ps.pose.orientation.w = float(q[0])
        ps.pose.orientation.x = float(q[1])
        ps.pose.orientation.y = float(q[2])
        ps.pose.orientation.z = float(q[3])
        self.pose_pub.publish(ps)
        # --- UDP: FER/BYE 폴링 ---
        while True:
            try:
                data, addr = self.udp_sock.recvfrom(64)
            except BlockingIOError:
                break
            except OSError:
                break
            if data == b'FER':
                if self.udp_client != addr:
                    self.node.get_logger().info(
                        f'[{self.robot_name}] stream client {addr}')
                self.udp_client = addr
            elif data == b'BYE':
                self.udp_client = None
        # --- UDP: cam_interval 틱마다 JPEG 프레임 전송 ---
        self._cam_tick += 1
        if self._cam_tick >= self.cam_interval:
            self._cam_tick = 0
            if self.udp_client is not None:
                img = self.camera.getImage()
                if img is not None:
                    w = self.camera.getWidth()
                    h = self.camera.getHeight()
                    bgra = np.frombuffer(img, np.uint8).reshape((h, w, 4))
                    gray = cv2.cvtColor(bgra, cv2.COLOR_BGRA2GRAY)
                    ok, jpg = cv2.imencode('.jpg', gray)
                    if ok:
                        buf = jpg.tobytes()
                        cpx = b'\x00\x00\x00\x00'
                        hdr = struct.pack('<BHHBBI', 0xBC, w, h, 1, 1, len(buf))
                        chunk = 1000
                        first = buf[:chunk - len(hdr)]
                        try:
                            self.udp_sock.sendto(cpx + hdr + first, self.udp_client)
                            off = len(first)
                            while off < len(buf):
                                part = buf[off:off + chunk]
                                self.udp_sock.sendto(cpx + part, self.udp_client)
                                off += len(part)
                        except OSError:
                            pass
