#!/usr/bin/env python3
"""ArUco 검출 노드 (드론 1대 담당).

cf_node가 UDP로 쏘는 JPEG 스트림(AI-deck 프로토콜)을 수신해서
ArUco 마커를 검출하고, 검출 시점의 드론 pose와 함께
/{drone}/marker_detections (coshow_interfaces/MarkerDetections)로 발행한다.

실행 (드론당 1프로세스):
  python3 aruco_detector_node.py --ros-args -p drone:=cf230
옵션:
  -p display:=true   # OpenCV 창으로 영상+검출 표시 (기본 false)
"""
import copy
import socket
import struct
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool
from coshow_interfaces.msg import MarkerDetection, MarkerDetections

# ── 카메라 역투영: 마커 픽셀 → 월드 좌표 (하방 카메라) ──
#
# 마커의 "화면상 크기"로 거리를 재는 방식이라 지형 높이를 몰라도 된다.
# 한 변 _MARKER_SIZE_M 인 마커가 깊이 Z 에 있으면 화면에 f*size/Z 픽셀로 보이므로,
# 거꾸로 Z = f*size/size_px 로 깊이가 나온다. 그 깊이의 광선 위 점이 곧 마커다.
#
# 예전에는 광선을 z=0 평면과 교차시켰다. 마커가 바닥에 있을 때만 맞는 가정이라,
# 건물 지붕(0.37~0.93 m)에 올리자 광선이 지붕을 지나쳐 바닥까지 내려가
# 최대 0.9 m 까지 빗나갔다. 크기 기반은 그 가정 자체가 없다.
_MARKER_SIZE_M = 0.2                  # 월드의 마커 한 변 (Box size 0.2 x 0.2)
_IMG_W, _IMG_H = 320.0, 220.0
_FOV_H = np.radians(87.0)
_FOV_V = 2 * np.arctan((_IMG_H / _IMG_W) * np.tan(_FOV_H / 2))
_FX = _IMG_W / (2 * np.tan(_FOV_H / 2))
_FY = _IMG_H / (2 * np.tan(_FOV_V / 2))
_CX, _CY = _IMG_W / 2, _IMG_H / 2
_CAM_STATIC = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])  # Rx(180): 하방


def _quat_to_R(qw, qx, qy, qz):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx*qx+qy*qy)]])


def backproject_marker(cx_px, cy_px, size_px, pose):
    """마커 픽셀·크기와 드론 pose 로 마커의 월드 좌표 (x, y, z) 계산.

    size_px 는 네 변 길이의 평균. 지형 높이를 가정하지 않으므로 마커가
    바닥에 있든 건물 지붕에 있든 같은 식으로 동작한다. 실패 시 None.

    한계: 마커가 카메라를 정면으로 마주본다고 본다. 크게 기울면 화면상
    크기가 줄어 실제보다 멀게 나온다. 하방 카메라 + 수평 지붕이라
    기울기는 드론 자세만큼(보통 10도 안팎)이고 그때 오차는 2% 미만이다.
    """
    if size_px <= 1e-6:
        return None
    p = pose.pose.position
    o = pose.pose.orientation
    nx = (cx_px - _CX) / _FX
    ny = (cy_px - _CY) / _FY
    depth = _FX * _MARKER_SIZE_M / size_px        # 광축 방향 거리
    pc = np.array([-ny, nx, 1.0]) * depth         # 카메라 좌표계에서의 마커 위치
    R = _quat_to_R(o.w, o.x, o.y, o.z) @ _CAM_STATIC
    v = R @ pc                                    # 월드 기준 상대 변위
    return float(p.x + v[0]), float(p.y + v[1]), float(p.z + v[2])



CPX_HEADER_SIZE = 4
IMG_HEADER_MAGIC = 0xBC
IMG_HEADER_SIZE = 11
MAGIC = b"FER"
GOODBYE = b"BYE"

# 드론 이름 -> (cf_node deck 포트, 수신 포트)
PORT_MAP = {
    'cf230': (6001, 5001),
    'cf231': (6002, 5002),
    'cf232': (6003, 5003),
    'cf233': (6004, 5004),
}


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        # --- 파라미터 ---
        self.declare_parameter('drone', 'cf230')
        self.declare_parameter('display', False)
        self.declare_parameter('deck_ip', '127.0.0.1')
        self.drone = self.get_parameter('drone').value
        self.display = self.get_parameter('display').value
        deck_ip = self.get_parameter('deck_ip').value

        if self.drone not in PORT_MAP:
            raise ValueError(f'unknown drone: {self.drone}')
        deck_port, listen_port = PORT_MAP[self.drone]
        self.deck_addr = (deck_ip, deck_port)

        # --- ArUco 검출기 (OpenCV 5.0 새 API) ---
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.detector = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters())

        # --- UDP 수신 소켓 ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', listen_port))
        self.sock.setblocking(False)

        # --- 프레임 재조립 상태 ---
        self.buffer = bytearray()
        self.expected_size = 0
        self.receiving = False
        self.width = 0
        self.height = 0
        self.fmt = 0
        self.frame_count = 0
        self.last_probe = 0.0

        # --- pose 구독 (최신값 보관) ---
        self.latest_pose = None
        self.create_subscription(
            PoseStamped, f'/{self.drone}/pose', self._pose_cb, 10)

        # --- 검출 결과 발행 ---
        self.pub = self.create_publisher(
            MarkerDetections, f'/{self.drone}/marker_detections', 10)

        # --- 대시보드용 주석 영상 발행 (관람 대시보드가 /aideck/{cf}/... 를 구독) ---
        # 실기 aideck_aruco_node 와 같은 토픽 이름을 써서 대시보드는 시뮬/실기 구분 없이 동작한다.
        self.declare_parameter('publish_image', True)   # 대시보드 송출 on/off
        self.declare_parameter('jpeg_quality', 70)
        self.publish_image = bool(self.get_parameter('publish_image').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        if self.publish_image:
            self.img_pub = self.create_publisher(
                CompressedImage, f'/aideck/{self.drone}/image_annotated/compressed', 10)
            self.fps_pub = self.create_publisher(Float32, f'/aideck/{self.drone}/fps', 10)
            self.ok_pub = self.create_publisher(Bool, f'/aideck/{self.drone}/stream_ok', 10)
            self._fps_ema = 0.0
            self._last_frame_t = None
            self.create_timer(1.0, self._publish_stream_health)   # fps·stream_ok 1 Hz

        # --- 메인 루프 타이머 (10ms마다 소켓 폴링) ---
        self.create_timer(0.01, self._poll)

        self.get_logger().info(
            f'[{self.drone}] aruco_detector: deck={deck_ip}:{deck_port} '
            f'listen={listen_port} display={self.display}')

    def _pose_cb(self, msg):
        self.latest_pose = msg

    def _poll(self):
        # 주기적 FER 프로브 (cf_node에게 스트리밍 요청)
        now = time.time()
        if now - self.last_probe >= 1.0:
            try:
                self.sock.sendto(MAGIC, self.deck_addr)
            except OSError:
                pass
            self.last_probe = now

        # 쌓인 패킷 전부 처리
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            except OSError:
                break
            self._handle_packet(data)

    def _handle_packet(self, data):
        if len(data) <= CPX_HEADER_SIZE:
            return
        payload = data[CPX_HEADER_SIZE:]

        if payload and payload[0] == IMG_HEADER_MAGIC:
            if len(payload) < IMG_HEADER_SIZE:
                return
            _, w, h, depth, fmt, size = struct.unpack(
                '<BHHBBI', payload[:IMG_HEADER_SIZE])
            self.buffer = bytearray(payload[IMG_HEADER_SIZE:])
            self.expected_size = size
            self.receiving = True
            self.width, self.height, self.fmt = w, h, fmt
            return

        if not self.receiving:
            return
        self.buffer.extend(payload)
        if len(self.buffer) < self.expected_size:
            return

        # 프레임 완성
        image_bytes = bytes(self.buffer[:self.expected_size])
        self.receiving = False
        self.frame_count += 1
        self._process_frame(image_bytes)

    def _process_frame(self, image_bytes):
        if self.fmt == 0:
            frame = np.frombuffer(image_bytes, dtype=np.uint8)
            try:
                frame = frame.reshape((self.height, self.width))
            except ValueError:
                return
        else:
            frame = cv2.imdecode(
                np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_UNCHANGED)
        if frame is None:
            return

        gray = frame if frame.ndim == 2 else cv2.cvtColor(
            frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        # --- 검출 결과 msg 구성 ---
        msg = MarkerDetections()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.drone
        msg.drone = self.drone
        if self.latest_pose is not None:
            # 아래에서 위치를 덮어쓰므로 원본(self.latest_pose)을 건드리지 않게 복사한다.
            msg.drone_pose = copy.deepcopy(self.latest_pose)

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                pts = marker_corners.reshape((4, 2))
                center = pts.mean(axis=0)
                edges = [np.linalg.norm(pts[(i + 1) % 4] - pts[i])
                         for i in range(4)]
                det = MarkerDetection()
                det.id = int(marker_id)
                det.cx = float(center[0])
                det.cy = float(center[1])
                det.size_px = float(np.mean(edges))
                # 역투영: 마커 실제 좌표 (드론 pose 있을 때만).
                # z 는 메시지에 담을 자리가 없어 버린다. BT 는 x·y 만 쓰고
                # 포착 고도는 config 의 altitudes.capture 로 따로 정한다.
                if self.latest_pose is not None:
                    proj = backproject_marker(det.cx, det.cy, det.size_px,
                                              self.latest_pose)
                    if proj is not None:
                        det.world_x, det.world_y = proj[0], proj[1]
                msg.markers.append(det)

        # drone_pose 는 이름 그대로 "검출 시점의 드론 위치" 로 둔다.
        # 마커 위치는 markers[i].world_x/world_y 에 마커별로 실려 있고,
        # 어느 마커가 타겟인지는 target_id 를 아는 BT 가 고른다.
        #
        # 예전에는 여기서 markers 가 정확히 1개일 때만 drone_pose 를 마커 좌표로
        # 덮어써 보냈다 (BT 를 안 고치려는 우회책). 한 프레임에 마커가 2개 이상이면
        # 어느 것인지 몰라 덮어쓰기를 건너뛰었고, 그러면 같은 필드가 말없이
        # "드론 위치" 로 되돌아가 BT 가 그것을 마커 위치로 믿었다.
        self.pub.publish(msg)

        # --- 주석 영상 만들기 (대시보드 송출 또는 로컬 창 표시 시) ---
        if self.publish_image or self.display:
            disp = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(disp, corners)
            hh, ww = disp.shape[:2]
            cv2.drawMarker(disp, (ww // 2, hh // 2), (0, 255, 255),
                           cv2.MARKER_CROSS, 18, 1)
            for m in msg.markers:
                px, py = int(m.cx), int(m.cy)
                cv2.circle(disp, (px, py), 4, (0, 0, 255), -1)
                cv2.putText(disp, f"ID:{m.id} ({m.world_x:.2f},{m.world_y:.2f})",
                            (px + 6, py - 6), cv2.FONT_HERSHEY_SIMPLEX,
                            0.45, (0, 0, 255), 1)
            if self.publish_image:
                ok, buf = cv2.imencode('.jpg', disp,
                                       [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if ok:
                    out = CompressedImage()
                    out.header.stamp = msg.header.stamp
                    out.header.frame_id = self.drone
                    out.format = 'jpeg'
                    out.data = buf.tobytes()
                    self.img_pub.publish(out)
                    now = time.time()
                    if self._last_frame_t is not None:
                        dt = now - self._last_frame_t
                        if dt > 0:
                            inst = 1.0 / dt
                            self._fps_ema = inst if self._fps_ema == 0 else 0.8 * self._fps_ema + 0.2 * inst
                    self._last_frame_t = now
            if self.display:
                cv2.imshow(f'aruco {self.drone}', cv2.resize(disp, None, fx=0.7, fy=0.7,
                           interpolation=cv2.INTER_AREA))
                cv2.waitKey(1)

        if msg.markers:
            found = ', '.join(f'id={m.id}({m.size_px:.0f}px)'
                              for m in msg.markers)
            self.get_logger().info(
                f'[{self.drone}] frame={self.frame_count} {found}')


    def _publish_stream_health(self):
        # fps 와 stream_ok(최근 프레임이 흐르는가) 를 대시보드에 보고.
        alive = self._last_frame_t is not None and (time.time() - self._last_frame_t) < 2.0
        self.fps_pub.publish(Float32(data=float(self._fps_ema if alive else 0.0)))
        self.ok_pub.publish(Bool(data=bool(alive)))

    def destroy_node(self):
        try:
            self.sock.sendto(GOODBYE, self.deck_addr)
        except OSError:
            pass
        self.sock.close()
        if self.display:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
