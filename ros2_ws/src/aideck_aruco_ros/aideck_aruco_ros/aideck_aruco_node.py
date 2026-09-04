#!/usr/bin/env python3
"""ROS 2 node version of ``multi_aruco_udp_viewer.py``.

Receives one or more AI-Deck UDP video streams, runs ArUco detection and
publishes the result on ROS topics instead of printing to the terminal.

Per drone ``<name>`` (relative to the node namespace)::

    <name>/image_raw                  sensor_msgs/Image           (mono8 / bgr8)
    <name>/image_annotated            sensor_msgs/Image           (bgr8, markers drawn)
    <name>/image_annotated/compressed sensor_msgs/CompressedImage (optional, jpeg)
    <name>/markers_text               std_msgs/String             (compact terminal view)
    <name>/fps                        std_msgs/Float32
    <name>/stream_ok                  std_msgs/Bool               (False while stalled)

Plus, in the GLOBAL namespace so the COSHOW behaviour tree finds it unchanged::

    /<name>/marker_detections         coshow_interfaces/MarkerDetections

That message carries, per marker, the pixel centre AND the marker's ground
position in the map frame (``world_x`` / ``world_y``), back-projected from the
pixel and the drone's pose.  The drone pose comes from ``/<name>/pose``
(published by crazyswarm2), which this node subscribes to.

The upstream ``vision_msgs/Detection2DArray`` and ``geometry_msgs/PointStamped``
outputs were dropped: they carried pixel coordinates only, which
``marker_detections`` already includes, and ``marker_center`` picked the largest
marker while the behaviour tree has to pick by target id.

``cv_bridge`` is deliberately NOT used: on this machine it is built
against numpy 1.x while numpy 2.2 is installed, so ``cv2_to_imgmsg()``
raises ``KeyError``.  The Image messages are filled in by hand, which is
exactly what cv_bridge does internally.

Run without building::

    python3 aideck_aruco_node.py --ros-args -p drones:="['drone1,192.168.1.125,5001']"

Run from the built package::

    ros2 run aideck_aruco_ros aideck_aruco_node --ros-args -p ...
"""

import array
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from coshow_interfaces.msg import MarkerDetection, MarkerDetections
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String

try:  # installed as a package
    from aideck_aruco_ros.udp_stream import (
        DroneStream,
        StreamSettings,
        overlay_status,
        parse_drone,
    )
except ImportError:  # run directly: python3 aideck_aruco_node.py
    import os
    import sys

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from udp_stream import (  # noqa: F401
        DroneStream,
        StreamSettings,
        overlay_status,
        parse_drone,
    )


# ── 카메라 역투영: 마커 픽셀 -> 지면(z=0) 월드 좌표 (하방 카메라) ──
#
# AI-deck (Himax HM01B0-MNA) 규격:
#   유효 픽셀 320x320, QVGA 윈도우 출력 324x244, 수평/수직 시야각 87°.
#   324x244 는 정사각 센서를 세로로 잘라낸 창이다. 광학은 그대로이므로 초점거리는
#   같고 수직 화각만 줄어든다 -> 87° 가 아니라 약 71°. 아래 _FOV_V 가 그 관계다.
#
# 주의: 규격표의 대각 115° 는 87°x87° 정사각의 핀홀 계산값(107°)과 맞지 않는다.
#   광학 왜곡이 있다는 뜻이고, 이 코드는 왜곡 없는 핀홀을 가정하므로 화면
#   가장자리에서 오차가 커진다. 우선 그대로 쓰고 실측 후 필요하면 보정한다.
_IMG_W, _IMG_H = 324.0, 244.0
_FOV_H = np.radians(87.0)
_FOV_V = 2 * np.arctan((_IMG_H / _IMG_W) * np.tan(_FOV_H / 2))
_FX = _IMG_W / (2 * np.tan(_FOV_H / 2))
_FY = _IMG_H / (2 * np.tan(_FOV_V / 2))
_CX, _CY = _IMG_W / 2, _IMG_H / 2
_CAM_STATIC = np.array(
    [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
)  # Rx(180): 하방


def quat_to_R(qw, qx, qy, qz):
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )


def backproject_marker(cx_px, cy_px, pose):
    """마커 픽셀과 드론 pose(위치+자세)로 마커의 지면 월드 좌표 (x,y) 계산.

    드론이 기울어져 있어도 자세를 반영해 광선을 회전시키므로 마커의 실제 위치가
    나온다. 광선이 지면을 향하지 않거나 뒤쪽에서 만나면 None.
    """
    p = pose.pose.position
    o = pose.pose.orientation
    nx = (cx_px - _CX) / _FX
    ny = (cy_px - _CY) / _FY
    pc = np.array([-ny, nx, 1.0])
    pc /= np.linalg.norm(pc)
    R = quat_to_R(o.w, o.x, o.y, o.z) @ _CAM_STATIC
    uE = R @ pc
    if uE[2] >= -1e-9:
        return None
    k = -p.z / uE[2]
    if k < 0:
        return None
    return float(p.x + k * uE[0]), float(p.y + k * uE[1])


def to_stamp(wall_time):
    """float seconds -> builtin_interfaces/Time fields."""
    sec = int(wall_time)
    nanosec = int(round((wall_time - sec) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    return sec, nanosec


def to_uint8_array(payload):
    """Wrap raw bytes for a ROS ``uint8[]`` field.

    Assigning ``bytes`` straight to ``msg.data`` makes rclpy's generated
    setter validate every single element while ``__debug__`` is true -
    measured at 16 ms for a 324x244 mono8 frame and 42 ms for bgr8.  That
    blocks the GIL long enough to starve the UDP receive thread and the
    stream starts losing packets.  Handing it an ``array.array('B')``
    hits the setter's fast path (typecode check only) and is ~1500x
    faster for identical bytes.
    """
    return array.array("B", payload)


def image_center(width, height):
    """Geometric centre of the image in pixels.

    Taken from the decoded frame's actual shape - nothing is hard coded.
    For the AI-Deck's 324x244 stream this gives (162.0, 122.0).
    """
    return width / 2.0, height / 2.0


def polygon_area(corners):
    """Shoelace area of the marker quad in pixels^2."""
    pts = np.asarray(corners, dtype=np.float64).reshape((-1, 2))
    x = pts[:, 0]
    y = pts[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2.0)


def format_corners(corners):
    return " ".join("({:.1f},{:.1f})".format(c[0], c[1]) for c in corners)


def parameter_as_bool(value):
    if isinstance(value, bool):
        return value

    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")

    return bool(value)


def fill_image_msg(msg, frame, sec, nanosec, frame_id):
    """Hand-rolled cv2 ndarray -> sensor_msgs/Image (no cv_bridge)."""
    if frame.dtype != np.uint8:
        frame = cv2.convertScaleAbs(frame)

    if frame.ndim == 2:
        encoding = "mono8"
        channels = 1
    elif frame.shape[2] == 3:
        encoding = "bgr8"
        channels = 3
    elif frame.shape[2] == 4:
        encoding = "bgra8"
        channels = 4
    else:
        raise ValueError("unsupported frame shape {}".format(frame.shape))

    frame = np.ascontiguousarray(frame)

    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = int(frame.shape[1] * channels)
    msg.data = to_uint8_array(frame.tobytes())
    return msg


class DroneChannel(object):
    """Publishers + local view state for one drone."""

    def __init__(self, node, stream, frame_id, image_qos, data_qos, options):
        name = stream.stream_name
        self.stream = stream
        self.frame_id = frame_id
        self.options = options

        self.pub_raw = (
            node.create_publisher(Image, "{}/image_raw".format(name), image_qos)
            if options["publish_raw"]
            else None
        )
        self.pub_annotated = (
            node.create_publisher(Image, "{}/image_annotated".format(name), image_qos)
            if options["publish_annotated"]
            else None
        )
        self.pub_compressed = (
            node.create_publisher(
                CompressedImage,
                "{}/image_annotated/compressed".format(name),
                image_qos,
            )
            if options["publish_compressed"]
            else None
        )
        self.pub_markers_text = node.create_publisher(
            String, "{}/markers_text".format(name), data_qos
        )
        self.pub_fps = node.create_publisher(Float32, "{}/fps".format(name), data_qos)
        self.pub_ok = node.create_publisher(Bool, "{}/stream_ok".format(name), data_qos)

        # ── COSHOW 연동 ──
        # 드론 pose 를 구독해 역투영에 쓰고, 마커의 지면 좌표까지 담아 발행한다.
        # 이름 앞의 "/" 는 의도한 것: 노드 namespace 와 무관하게 crazyswarm2 /
        # 행동트리가 쓰는 전역 이름(/cf231/pose, /cf231/marker_detections)에 맞춘다.
        self.latest_pose = None
        node.create_subscription(
            PoseStamped,
            "/{}/pose".format(name),
            self._on_pose,
            10,
        )
        self.pub_coshow = node.create_publisher(
            MarkerDetections, "/{}/marker_detections".format(name), data_qos
        )

    def _on_pose(self, msg):
        self.latest_pose = msg

        self.window = "AI-Deck UDP ArUco - {}".format(name)
        self.latest_display = None
        self.latest_fps = 0.0
        self.shape_logged = False
        self.latest_found = []
        self.window_created = False


class AideckArucoNode(Node):
    def __init__(self):
        super().__init__("aideck_aruco_node")

        self.declare_parameter("drones", [""])
        self.declare_parameter("dictionary", "DICT_4X4_50")
        self.declare_parameter("target_id", -1)
        self.declare_parameter("detect_every", 1)
        self.declare_parameter("probe_period", 1.0)
        self.declare_parameter("timeout", 0.2)
        self.declare_parameter("stall_timeout", 3.0)
        self.declare_parameter("publish_raw", True)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("publish_compressed", False)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("display", False)
        self.declare_parameter("display_scale", 1.6)
        self.declare_parameter("display_rate", 30.0)
        self.declare_parameter("poll_rate", 100.0)
        self.declare_parameter("image_qos", "sensor_data")
        self.declare_parameter("frame_id_template", "{name}_camera")
        self.declare_parameter("log_detections", False)
        self.declare_parameter("publish_empty_text", False)
        # Detector experiment: per-frame detection diagnostics on rosout.
        self.declare_parameter("debug_detection", True)
        self.declare_parameter("queue_depth", 4)
        self.declare_parameter("rcvbuf_bytes", 0)

        specs = [
            spec
            for spec in self.get_parameter("drones").get_parameter_value().string_array_value
            if spec.strip()
        ]
        if not specs:
            raise RuntimeError(
                "No drones configured. Set the 'drones' parameter, e.g. "
                "-p drones:=\"['drone1,192.168.1.125,5001']\""
            )

        configs = [parse_drone(spec) for spec in specs]

        ports = [config["listen_port"] for config in configs]
        if len(ports) != len(set(ports)):
            raise RuntimeError("Each drone must use a unique listen port")

        names = [config["name"] for config in configs]
        if len(names) != len(set(names)):
            raise RuntimeError("Each drone must use a unique name")

        target_id = self.get_parameter("target_id").value
        self.target_id = None if target_id is None or target_id < 0 else int(target_id)
        self.display_enabled = parameter_as_bool(self.get_parameter("display").value)
        self.display_scale = float(self.get_parameter("display_scale").value)
        self.log_detections = parameter_as_bool(
            self.get_parameter("log_detections").value
        )
        self.publish_empty_text = parameter_as_bool(
            self.get_parameter("publish_empty_text").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.debug_detection = parameter_as_bool(
            self.get_parameter("debug_detection").value
        )

        options = {
            "publish_raw": parameter_as_bool(self.get_parameter("publish_raw").value),
            "publish_annotated": parameter_as_bool(
                self.get_parameter("publish_annotated").value
            ),
            "publish_compressed": parameter_as_bool(
                self.get_parameter("publish_compressed").value
            ),
        }
        need_annotated = (
            options["publish_annotated"]
            or options["publish_compressed"]
            or self.display_enabled
        )

        settings = StreamSettings(
            dictionary=self.get_parameter("dictionary").value,
            target_id=self.target_id,
            detect_every=int(self.get_parameter("detect_every").value),
            probe_period=float(self.get_parameter("probe_period").value),
            timeout=float(self.get_parameter("timeout").value),
            stall_timeout=float(self.get_parameter("stall_timeout").value),
            need_annotated=need_annotated,
            rcvbuf_bytes=int(self.get_parameter("rcvbuf_bytes").value),
            queue_depth=int(self.get_parameter("queue_depth").value),
        )

        image_qos = self._make_image_qos()
        data_qos = QoSProfile(depth=10)

        self.stop_event = threading.Event()
        self.shutdown_requested = False
        self.channels = []
        self.streams = []

        template = self.get_parameter("frame_id_template").value
        for config in configs:
            stream = DroneStream(config, settings, self.stop_event)
            self.streams.append(stream)
            self.channels.append(
                DroneChannel(
                    self,
                    stream,
                    template.format(name=config["name"]),
                    image_qos,
                    data_qos,
                    options,
                )
            )

        self.get_logger().info("OpenCV     : {}".format(cv2.__version__))
        self.get_logger().info("Dictionary : {}".format(settings.dictionary))
        self.get_logger().info(
            "Target ID  : {}".format(
                self.target_id if self.target_id is not None else "all"
            )
        )
        self.get_logger().info(
            "Display    : {}".format("on" if self.display_enabled else "off")
        )
        for channel in self.channels:
            self.get_logger().info(
                "Topics     : {}/image_raw, {}/markers, {}/marker_center".format(
                    channel.stream.stream_name,
                    channel.stream.stream_name,
                    channel.stream.stream_name,
                )
            )

        for stream in self.streams:
            stream.start()

        poll_rate = max(1.0, float(self.get_parameter("poll_rate").value))
        self.create_timer(1.0 / poll_rate, self.on_poll)
        self.create_timer(1.0, self.on_status)
        if self.display_enabled:
            display_rate = max(1.0, float(self.get_parameter("display_rate").value))
            self.create_timer(1.0 / display_rate, self.on_display)

    # ------------------------------------------------------------------
    def _make_image_qos(self):
        mode = str(self.get_parameter("image_qos").value).lower()
        if mode == "reliable":
            return QoSProfile(depth=5, reliability=QoSReliabilityPolicy.RELIABLE)
        if mode not in ("sensor_data", "best_effort"):
            self.get_logger().warn(
                "Unknown image_qos '{}', falling back to sensor_data".format(mode)
            )
        return qos_profile_sensor_data

    # ------------------------------------------------------------------
    # timers
    # ------------------------------------------------------------------
    def on_poll(self):
        for channel in self.channels:
            stream = channel.stream

            while True:
                try:
                    level, message = stream.events.get_nowait()
                except Exception:
                    break
                logger = self.get_logger()
                if level == "error":
                    logger.error(message)
                elif level == "warn":
                    logger.warn(message)
                else:
                    logger.info(message)

            while True:
                try:
                    record = stream.frames.get_nowait()
                except Exception:
                    break
                self.publish_record(channel, record)

    def publish_record(self, channel, record):
        sec, nanosec = to_stamp(record["stamp"])
        frame_id = channel.frame_id

        if channel.pub_raw is not None:
            channel.pub_raw.publish(
                fill_image_msg(Image(), record["frame"], sec, nanosec, frame_id)
            )

        annotated = record["annotated"]
        if annotated is not None and channel.pub_annotated is not None:
            channel.pub_annotated.publish(
                fill_image_msg(Image(), annotated, sec, nanosec, frame_id)
            )

        if annotated is not None and channel.pub_compressed is not None:
            ok, buf = cv2.imencode(
                ".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            )
            if ok:
                msg = CompressedImage()
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nanosec
                msg.header.frame_id = frame_id
                msg.format = "jpeg"
                msg.data = to_uint8_array(buf.tobytes())
                channel.pub_compressed.publish(msg)

        found = record["found"]

        # Only publish a detection array on frames that were actually run
        # through the detector (detect_every > 1 skips some frames).
        if record["detected"]:
            # COSHOW 행동트리가 구독하는 메시지. 마커마다 픽셀 좌표와 함께
            # 역투영한 지면 좌표(world_x/world_y)를 담는다. 트리는 target_id 와
            # 맞는 마커를 골라 그 좌표를 쓰므로, 한 프레임에 마커가 여러 개
            # 잡혀도 문제되지 않는다.
            detections = MarkerDetections()
            detections.header.stamp.sec = sec
            detections.header.stamp.nanosec = nanosec
            detections.header.frame_id = frame_id
            detections.drone = record["name"]
            pose = channel.latest_pose
            if pose is not None:
                detections.drone_pose = pose

            for marker in found:
                det = MarkerDetection()
                det.id = int(marker["id"])
                det.cx = float(marker["center_x"])
                det.cy = float(marker["center_y"])
                det.size_px = float(marker["size_px"])
                # pose 가 아직 없거나 광선이 지면을 안 향하면 0 으로 남는다.
                # 트리는 0 인 프레임을 버리고 다음 프레임에 다시 시도한다.
                if pose is not None:
                    proj = backproject_marker(det.cx, det.cy, pose)
                    if proj is not None:
                        det.world_x, det.world_y = proj
                detections.markers.append(det)

            channel.pub_coshow.publish(detections)
            self.publish_markers_text(channel, record, found)

            if found:
                if self.log_detections:
                    for marker in found:
                        self.get_logger().info(
                            "{}: Found frame={} id={} center=({:.1f},{:.1f}) "
                            "size_px={:.1f} fps={:.2f}".format(
                                record["name"],
                                record["frame_count"],
                                marker["id"],
                                marker["center_x"],
                                marker["center_y"],
                                marker["size_px"],
                                record["fps"],
                            )
                        )

        if self.debug_detection:
            self.log_detection_debug(channel, record, found)

        channel.latest_fps = record["fps"]
        channel.latest_found = found
        if self.display_enabled and annotated is not None:
            channel.latest_display = annotated

    def log_detection_debug(self, channel, record, found):
        """Per-frame detector diagnostics (runs on the ROS thread, not the
        UDP receive thread, so it cannot stall packet reception)."""
        frame = record["frame"]
        width = record["width"]
        height = record["height"]
        u0, v0 = image_center(width, height)
        logger = self.get_logger()

        if not channel.shape_logged:
            channel.shape_logged = True
            logger.info(
                "{}: decoded frame shape={} dtype={} ndim={} -> image_center=({:.1f},{:.1f})".format(
                    record["name"], frame.shape, frame.dtype, frame.ndim, u0, v0
                )
            )

        head = "{}: f={} t={:.3f}".format(
            record["name"], record["frame_count"], record["stamp"]
        )

        if not record["detected"]:
            return

        if not found:
            logger.info(
                "{} NO-MARKER img={}x{} img_center=({:.1f},{:.1f}) fps={:.2f}".format(
                    head, width, height, u0, v0, record["fps"]
                )
            )
            return

        for marker in found:
            uc = marker["center_x"]
            vc = marker["center_y"]
            logger.info(
                "{} DETECT id={} center=({:.1f},{:.1f}) img_center=({:.1f},{:.1f}) "
                "du={:+.1f} dv={:+.1f} size_px={:.1f} area_px={:.1f} "
                "img={}x{} fps={:.2f} corners={}".format(
                    head,
                    marker["id"],
                    uc,
                    vc,
                    u0,
                    v0,
                    uc - u0,
                    vc - v0,
                    marker["size_px"],
                    polygon_area(marker["corners"]),
                    width,
                    height,
                    record["fps"],
                    format_corners(marker["corners"]),
                )
            )

    def publish_markers_text(self, channel, record, found):
        if not found and not self.publish_empty_text:
            return

        u0, v0 = image_center(record["width"], record["height"])

        parts = [
            "frame={}".format(record["frame_count"]),
            "count={}".format(len(found)),
            "fps={:.2f}".format(record["fps"]),
            "img={}x{}".format(record["width"], record["height"]),
            "img_center=({:.1f},{:.1f})".format(u0, v0),
        ]

        for marker in found:
            parts.append(
                "id={} u={:.1f} v={:.1f} du={:+.1f} dv={:+.1f} "
                "size={:.1f} area={:.1f} theta={:.2f}".format(
                    marker["id"],
                    marker["center_x"],
                    marker["center_y"],
                    marker["center_x"] - u0,
                    marker["center_y"] - v0,
                    marker["size_px"],
                    polygon_area(marker["corners"]),
                    marker["theta"],
                )
            )

        msg = String()
        msg.data = " | ".join(parts)
        channel.pub_markers_text.publish(msg)

    def on_status(self):
        for channel in self.channels:
            stream = channel.stream
            fps = Float32()
            fps.data = float(channel.latest_fps)
            channel.pub_fps.publish(fps)

            ok = Bool()
            ok.data = not stream.stalled and stream.last_frame_wall_time is not None
            channel.pub_ok.publish(ok)

    def on_display(self):
        for channel in self.channels:
            if channel.latest_display is None:
                continue

            display = channel.latest_display.copy()
            self.draw_center_overlay(display, channel.latest_found)
            overlay_status(
                display,
                channel.stream.stream_name,
                channel.stream.frame_count,
                channel.latest_fps,
                channel.latest_found,
                channel.stream.stalled,
            )

            if self.display_scale != 1.0:
                display = cv2.resize(
                    display,
                    None,
                    fx=self.display_scale,
                    fy=self.display_scale,
                    interpolation=cv2.INTER_NEAREST,
                )

            cv2.imshow(channel.window, display)
            channel.window_created = True

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.get_logger().info("Quit key pressed")
            self.shutdown_requested = True

    def draw_center_overlay(self, display, found):
        """Image centre cross, marker centre and du/dv. Marker boundary and id
        are already drawn by udp_stream.draw_markers()."""
        height, width = display.shape[0], display.shape[1]
        u0, v0 = image_center(width, height)
        iu0, iv0 = int(round(u0)), int(round(v0))

        # image centre: cyan cross
        cv2.line(display, (iu0 - 8, iv0), (iu0 + 8, iv0), (255, 255, 0), 1, cv2.LINE_AA)
        cv2.line(display, (iu0, iv0 - 8), (iu0, iv0 + 8), (255, 255, 0), 1, cv2.LINE_AA)

        if not found:
            return

        for marker in found:
            uc = marker["center_x"]
            vc = marker["center_y"]
            iuc, ivc = int(round(uc)), int(round(vc))

            # marker centre: magenta dot + line from image centre
            cv2.line(display, (iu0, iv0), (iuc, ivc), (255, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(display, (iuc, ivc), 4, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.putText(
                display,
                "du={:+.0f} dv={:+.0f}".format(uc - u0, vc - v0),
                (min(iuc + 8, width - 90), max(ivc - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )

    # ------------------------------------------------------------------
    def stop_streams(self):
        # Plain print: on Ctrl+C rclpy has already invalidated the
        # context, so get_logger() would fail here.
        for stream in self.streams:
            print(
                "{}: frames={} partial_frames_dropped={} last_error={}".format(
                    stream.stream_name,
                    stream.frame_count,
                    stream.dropped_frames,
                    stream.last_error,
                )
            )

        self.stop_event.set()
        for stream in self.streams:
            stream.close()
        for stream in self.streams:
            stream.join(timeout=1.0)
        if self.display_enabled:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except cv2.error:
                pass


def main(args=None):
    rclpy.init(args=args)

    node = None
    status = 0
    try:
        node = AideckArucoNode()

        # rclpy only installs a SIGINT handler.  Handle SIGTERM too
        # (ros2 launch shutdown, systemd, plain `kill`) so the streams
        # still get their "BYE" and the deck stops transmitting.
        def on_sigterm(_signum, _frame):
            node.shutdown_requested = True

        signal.signal(signal.SIGTERM, on_sigterm)

        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C / external shutdown is a normal exit, not a failure.
        pass
    except (RuntimeError, ValueError, OSError) as error:
        print("aideck_aruco_node: {}".format(error))
        status = 1
    finally:
        if node is not None:
            node.stop_streams()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Stopped")

    return status


if __name__ == "__main__":
    raise SystemExit(main())
