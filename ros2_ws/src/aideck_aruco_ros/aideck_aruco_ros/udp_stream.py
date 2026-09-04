#!/usr/bin/env python3
"""AI-Deck UDP frame receiver + ArUco detection.

The wire protocol handling in this module is a 1:1 port of
``multi_aruco_udp_viewer.py`` (udp-link-diagnostic).  Nothing about the
network behaviour was changed:

* one UDP socket per drone, bound to ``0.0.0.0:<listen_port>``
* ``b"FER"`` probe sent from that same socket every ``probe_period``
  (the ESP32 registers the *source* ip:port of that packet as its stream
  target - see ``aideck-esp-firmware-udp/main/wifi.c``)
* 4 byte CPX header is stripped, ``0xBC`` marks an image header
  ``<BHHBBI`` = magic, width, height, depth, format, size
* ``b"BYE"`` sent on close so the deck clears its target

The only difference from the original is the *output*: instead of
printing to the terminal and keeping a single "latest display" frame,
each completed frame is pushed onto a ``queue.Queue`` so that the ROS
thread can publish it.  All ROS specific code lives in
``aideck_aruco_node.py`` - this module has no rclpy dependency.
"""

import queue
import socket
import struct
import threading
import time

import cv2
import numpy as np


CPX_HEADER_SIZE = 4
IMG_HEADER_MAGIC = 0xBC
IMG_HEADER_SIZE = 11
MAGIC = b"FER"
GOODBYE = b"BYE"

# Sanity bound on the image size announced in the image header.  A
# corrupt header could otherwise make us allocate/wait for nonsense.
MAX_IMAGE_BYTES = 4 * 1024 * 1024


def parse_drone(definition):
    """Parse ``NAME,IP,LISTEN_PORT[,DECK_PORT]`` (same format as the viewer)."""
    parts = [part.strip() for part in definition.split(",")]
    if len(parts) not in (3, 4):
        raise ValueError(
            "Invalid drone spec '{}'. Use NAME,IP,LISTEN_PORT[,DECK_PORT]".format(
                definition
            )
        )

    return {
        "name": parts[0],
        "deck_ip": parts[1],
        "listen_port": int(parts[2]),
        "deck_port": int(parts[3]) if len(parts) == 4 else 5000,
    }


def create_aruco_detector(dictionary_name):
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("This OpenCV build does not include cv2.aruco")

    if not hasattr(cv2.aruco, dictionary_name):
        raise RuntimeError("Unknown ArUco dictionary: {}".format(dictionary_name))

    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    else:
        detector = None

    return detector, dictionary, parameters


def detect_markers(detector, dictionary, parameters, gray):
    if detector is not None:
        return detector.detectMarkers(gray)

    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def decode_frame(image_bytes, width, height, fmt):
    if fmt == 0:
        if width <= 0 or height <= 0 or len(image_bytes) < width * height:
            return None
        frame = np.frombuffer(image_bytes, dtype=np.uint8, count=width * height)
        return frame.reshape((height, width))

    return cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_UNCHANGED)


def frame_to_gray(frame):
    if frame.ndim == 2:
        return frame

    if frame.shape[2] == 4:
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)

    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def analyse_markers(corners, ids, target_id):
    """Turn raw detector output into plain dicts (no OpenCV types)."""
    found = []
    if ids is None or len(ids) == 0:
        return found, [], []

    selected_corners = []
    selected_ids = []

    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        if target_id is not None and marker_id != target_id:
            continue

        pts = np.asarray(marker_corners, dtype=np.float64).reshape((4, 2))
        center = pts.mean(axis=0)
        edge_lengths = [
            float(np.linalg.norm(pts[(i + 1) % 4] - pts[i])) for i in range(4)
        ]
        size_px = float(np.mean(edge_lengths))

        # Orientation of the marker's top edge (corner0 -> corner1) in
        # image coordinates, radians.  Image y grows downwards.
        top_edge = pts[1] - pts[0]
        theta = float(np.arctan2(top_edge[1], top_edge[0]))

        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)

        found.append(
            {
                "id": marker_id,
                "center_x": float(center[0]),
                "center_y": float(center[1]),
                "size_px": size_px,
                "theta": theta,
                "bbox_w": float(maxs[0] - mins[0]),
                "bbox_h": float(maxs[1] - mins[1]),
                "corners": [[float(p[0]), float(p[1])] for p in pts],
            }
        )
        selected_corners.append(marker_corners)
        selected_ids.append([marker_id])

    return found, selected_corners, selected_ids


def draw_markers(frame, selected_corners, selected_ids):
    """Return a BGR copy of ``frame`` with the selected markers drawn."""
    if frame.ndim == 2:
        display = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif frame.shape[2] == 4:
        display = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    else:
        display = frame.copy()

    if selected_corners:
        cv2.aruco.drawDetectedMarkers(
            display,
            selected_corners,
            np.array(selected_ids, dtype=np.int32),
        )

    return display


def overlay_status(display, name, frame_count, fps, found, stalled):
    status = (
        "Found {}".format(",".join(str(marker["id"]) for marker in found))
        if found
        else "No marker"
    )
    color = (0, 255, 0) if found else (0, 0, 255)

    if stalled:
        status = "Stream stalled"
        color = (0, 165, 255)

    cv2.putText(
        display,
        "{} f={} fps={:.1f}".format(name, frame_count, fps),
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 0),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        display,
        status,
        (8, 44),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        color,
        2,
        cv2.LINE_AA,
    )


class StreamSettings(object):
    """Plain settings container (replaces argparse ``args``)."""

    def __init__(
        self,
        dictionary="DICT_4X4_50",
        target_id=None,
        detect_every=1,
        probe_period=1.0,
        timeout=0.2,
        stall_timeout=3.0,
        need_annotated=True,
        rcvbuf_bytes=0,
        queue_depth=4,
    ):
        self.dictionary = dictionary
        self.target_id = target_id
        self.detect_every = detect_every
        self.probe_period = probe_period
        self.timeout = timeout
        self.stall_timeout = stall_timeout
        self.need_annotated = need_annotated
        self.rcvbuf_bytes = rcvbuf_bytes
        self.queue_depth = queue_depth


class DroneStream(threading.Thread):
    """One UDP stream: receive, reassemble, decode, detect, enqueue.

    Events are pushed to ``self.frames`` (bounded, drop-oldest) and
    ``self.events`` (log lines the ROS node re-emits through rosout).
    Nothing in this class touches rclpy, so it stays safe to run in a
    background thread.
    """

    def __init__(self, config, settings, stop_event):
        super().__init__(daemon=True, name="udp-{}".format(config["name"]))
        self.stream_name = config["name"]
        self.deck_addr = (config["deck_ip"], config["deck_port"])
        self.listen_port = config["listen_port"]
        self.settings = settings
        self.stop_event = stop_event

        self.detector, self.dictionary, self.parameters = create_aruco_detector(
            settings.dictionary
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if settings.rcvbuf_bytes > 0:
            try:
                self.sock.setsockopt(
                    socket.SOL_SOCKET, socket.SO_RCVBUF, settings.rcvbuf_bytes
                )
            except OSError:
                pass
        try:
            self.sock.bind(("0.0.0.0", self.listen_port))
        except OSError as error:
            self.sock.close()
            raise OSError(
                "{}: could not bind UDP listen port {}. "
                "Stop any existing udp_viewer/multi_aruco_udp_viewer/"
                "aideck_aruco_node using this port, or choose another "
                "listen_port. Original error: {}".format(
                    self.stream_name,
                    self.listen_port,
                    error,
                )
            ) from error
        self.sock.settimeout(settings.timeout)

        self.buffer = bytearray()
        self.expected_size = 0
        self.packet_count = 0
        self.frame_count = 0
        self.dropped_frames = 0
        self.last_frame_time = None
        self.last_frame_wall_time = None
        self.receiving = False
        self.width = 0
        self.height = 0
        self.depth = 0
        self.fmt = 0
        self.stalled = False
        self.last_error = None

        self.frames = queue.Queue(maxsize=max(1, settings.queue_depth))
        self.events = queue.Queue(maxsize=256)

    # ------------------------------------------------------------------
    # plumbing
    # ------------------------------------------------------------------
    def log(self, level, message):
        try:
            self.events.put_nowait((level, "{}: {}".format(self.stream_name, message)))
        except queue.Full:
            pass

    def publish_frame(self, record):
        """Drop-oldest so a slow consumer can never stall the socket read."""
        while True:
            try:
                self.frames.put_nowait(record)
                return
            except queue.Full:
                try:
                    self.frames.get_nowait()
                except queue.Empty:
                    pass

    def close(self):
        try:
            self.sock.sendto(GOODBYE, self.deck_addr)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass

    # ------------------------------------------------------------------
    # thread body - identical control flow to the original viewer
    # ------------------------------------------------------------------
    def run(self):
        self.log(
            "info",
            "target={}:{} listen=0.0.0.0:{}".format(
                self.deck_addr[0], self.deck_addr[1], self.listen_port
            ),
        )

        last_probe = 0.0
        last_wait_log = time.time()

        while not self.stop_event.is_set():
            now = time.time()
            if now - last_probe >= self.settings.probe_period:
                try:
                    self.sock.sendto(MAGIC, self.deck_addr)
                except OSError as error:
                    self.last_error = str(error)
                    self.log("warn", "probe send failed: {}".format(error))
                last_probe = now

            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                self.handle_timeout(last_wait_log)
                if time.time() - last_wait_log >= self.settings.stall_timeout:
                    last_wait_log = time.time()
                continue
            except OSError as error:
                if not self.stop_event.is_set():
                    self.last_error = str(error)
                    self.log("error", "socket error: {}".format(error))
                break

            self.handle_packet(data)

    def handle_timeout(self, last_wait_log):
        now = time.time()
        if self.last_frame_wall_time is None:
            if now - last_wait_log >= self.settings.stall_timeout:
                self.log(
                    "warn",
                    "STREAM WAIT no UDP frames for {:.1f}s target={}:{}".format(
                        self.settings.stall_timeout,
                        self.deck_addr[0],
                        self.deck_addr[1],
                    ),
                )
            return

        gap = now - self.last_frame_wall_time
        if gap >= self.settings.stall_timeout and not self.stalled:
            self.stalled = True
            self.log(
                "warn",
                "STREAM STALL no UDP frames for {:.1f}s frames={}".format(
                    gap, self.frame_count
                ),
            )

    def handle_packet(self, data):
        if len(data) <= CPX_HEADER_SIZE:
            return

        payload = data[CPX_HEADER_SIZE:]

        if payload and payload[0] == IMG_HEADER_MAGIC:
            if len(payload) < IMG_HEADER_SIZE:
                self.log("warn", "STREAM WARN incomplete image header")
                return

            _, width, height, depth, fmt, size = struct.unpack(
                "<BHHBBI", payload[:IMG_HEADER_SIZE]
            )

            if size == 0 or size > MAX_IMAGE_BYTES:
                self.log("warn", "STREAM WARN bogus image size {}".format(size))
                self.receiving = False
                return

            if self.receiving:
                # A new frame started before the previous one completed.
                self.dropped_frames += 1

            self.buffer = bytearray(payload[IMG_HEADER_SIZE:])
            self.expected_size = size
            self.packet_count = 1
            self.receiving = True
            self.width = width
            self.height = height
            self.depth = depth
            self.fmt = fmt
            return

        if not self.receiving:
            return

        self.buffer.extend(payload)
        self.packet_count += 1

        if len(self.buffer) < self.expected_size:
            return

        image_bytes = bytes(self.buffer[: self.expected_size])
        self.receiving = False
        self.frame_count += 1

        frame = decode_frame(image_bytes, self.width, self.height, self.fmt)
        if frame is None:
            self.log(
                "warn",
                "STREAM WARN image decode failed frame={}".format(self.frame_count),
            )
            return

        now = time.time()
        fps = 0.0
        if self.last_frame_time is not None:
            dt = now - self.last_frame_time
            fps = 1.0 / dt if dt > 0 else 0.0
        self.last_frame_time = now
        self.last_frame_wall_time = now

        if self.stalled:
            self.log("info", "STREAM RECOVERED frame={}".format(self.frame_count))
            self.stalled = False

        found = []
        annotated = None
        detected = False

        detect_every = self.settings.detect_every
        if detect_every <= 1 or self.frame_count % detect_every == 0:
            detected = True
            gray = frame_to_gray(frame)
            corners, ids, _ = detect_markers(
                self.detector, self.dictionary, self.parameters, gray
            )
            found, selected_corners, selected_ids = analyse_markers(
                corners, ids, self.settings.target_id
            )
            if self.settings.need_annotated:
                annotated = draw_markers(frame, selected_corners, selected_ids)
        elif self.settings.need_annotated:
            annotated = draw_markers(frame, [], [])

        self.publish_frame(
            {
                "name": self.stream_name,
                "stamp": now,
                "frame": frame,
                "annotated": annotated,
                "found": found,
                "detected": detected,
                "fps": fps,
                "frame_count": self.frame_count,
                "dropped_frames": self.dropped_frames,
                "stalled": self.stalled,
                "width": frame.shape[1],
                "height": frame.shape[0],
            }
        )
