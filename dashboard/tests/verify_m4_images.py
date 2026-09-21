#!/usr/bin/env python3
"""Real ROS/JPEG evidence for §4.6; no Webots, hardware or detector UDP socket.

Run in the ROS Humble test container with OpenCV >= 4.7, ROS_DOMAIN_ID=85
and ROS_LOCALHOST_ONLY=1. Only the detector's existing UDP socket is replaced;
real OpenCV decoding/detection/annotation and all ROS publications execute.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import cv2
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32
from coshow_interfaces.msg import MarkerDetections

from dashboard.tests.test_sim_images import NoNetworkSocket, load_detector, marker_frame


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'dashboard/REPORTS/evidence'


def fixture(control):
    module = load_detector()
    module.socket = SimpleNamespace(socket=lambda *_: NoNetworkSocket(), AF_INET=2, SOCK_DGRAM=2)
    rclpy.init(args=[])
    node = module.ArucoDetectorNode()
    node.width, node.height, node.fmt = 320, 220, 1
    ok, encoded = cv2.imencode('.jpg', marker_frame(), [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok
    payload = encoded.tobytes()

    def inject():
        if control.exists() and control.read_text() == 'stream':
            node._process_frame(payload)

    node.create_timer(.1, inject, clock=Clock(clock_type=ClockType.STEADY_TIME))
    print('FIXTURE READY: real simulated detector; socket replaced; JPEG input at 10 Hz when enabled', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '85'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    control = ROOT / 'dashboard/run/m4_images_control'
    control.parent.mkdir(parents=True, exist_ok=True)
    control.write_text('paused')
    rclpy.init(args=[])
    observer = Node('m4_images_observer')
    samples = {'images': [], 'fps': [], 'ok': [], 'detections': []}

    def receive(key):
        return lambda message: samples[key].append((time.monotonic(), message))

    prefix = '/aideck/cf230/'
    observer.create_subscription(CompressedImage, prefix + 'image_annotated/compressed',
                                 receive('images'), qos_profile_sensor_data)
    observer.create_subscription(Float32, prefix + 'fps', receive('fps'), 10)
    observer.create_subscription(Bool, prefix + 'stream_ok', receive('ok'), 10)
    observer.create_subscription(MarkerDetections, '/cf230/marker_detections', receive('detections'), 10)

    def wait_for(predicate, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(observer, timeout_sec=.05)
            if predicate():
                return
        raise AssertionError('ROS condition timed out: sample counts ' + str({k: len(v) for k, v in samples.items()}))

    log_path = EVIDENCE / 'M4_images_fixture.log'
    with log_path.open('w') as log:
        command = [sys.executable, str(Path(__file__).resolve()), '--fixture', str(control)]
        log.write('$ ' + ' '.join(command) + '\n')
        log.flush()
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        try:
            wait_for(lambda: samples['ok'] and samples['ok'][-1][1].data is False)
            assert not samples['images']
            assert samples['fps'][-1][1].data == 0.0
            assert observer.count_publishers('/clock') == 0
            print('PASS startup with no frame: stream_ok=false, fps=0; no /clock publisher', flush=True)
            control.write_text('stream')
            wait_for(lambda: len(samples['images']) >= 20 and samples['ok'][-1][1].data
                     and samples['fps'][-1][1].data > 0.0)
            image = samples['images'][-1][1]
            decoded = cv2.imdecode(np.frombuffer(bytes(image.data), np.uint8), cv2.IMREAD_COLOR)
            assert image.format == 'jpeg' and decoded.shape == (220, 320, 3)
            assert any(marker.id == 3 for _, msg in samples['detections'] for marker in msg.markers)
            assert decoded[110, 160, 1] > decoded[110, 160, 0] + 80
            print('PASS actual compressed subscription: {} JPEGs, 320x220 BGR, yellow annotation, marker ID 3'.format(
                len(samples['images'])), flush=True)

            cli_log = EVIDENCE / 'M4_images_ros_cli.log'
            with cli_log.open('w') as output:
                for command, expected in (
                    (['ros2', 'topic', 'list', '-t'], 'sensor_msgs/msg/CompressedImage'),
                    (['ros2', 'topic', 'info', '-v', prefix + 'image_annotated/compressed'], 'BEST_EFFORT'),
                    (['ros2', 'topic', 'info', '-v', prefix + 'fps'], 'std_msgs/msg/Float32'),
                    (['ros2', 'topic', 'info', '-v', prefix + 'stream_ok'], 'std_msgs/msg/Bool'),
                    (['ros2', 'topic', 'echo', '--once', prefix + 'fps'], 'data:'),
                    (['ros2', 'topic', 'echo', '--once', prefix + 'stream_ok'], 'data: true'),
                ):
                    result = subprocess.run(command, capture_output=True, text=True, timeout=15)
                    output.write('$ ' + ' '.join(command) + '\n' + result.stdout + result.stderr)
                    output.write('exit={}\n'.format(result.returncode))
                    output.flush()
                    assert result.returncode == 0 and expected in result.stdout, result.stdout + result.stderr

            # Consume queued messages, then stop production and observe the 1 Hz timers.
            wait_for(lambda: len(samples['images']) >= 30)
            active_fps = [round(msg.data, 3) for _, msg in samples['fps'] if msg.data > 0.0]
            control.write_text('paused')
            stopped = time.monotonic()
            stop_ok_index = len(samples['ok'])
            stop_fps_index = len(samples['fps'])
            wait_for(lambda: len(samples['ok']) > stop_ok_index
                     and samples['ok'][-1][1].data is False, timeout=5.0)
            stale_at = samples['ok'][-1][0]
            last_frame_at = samples['images'][-1][0]
            age = stale_at - last_frame_at
            assert 2.8 <= age <= 4.2, age
            post_fps = samples['fps'][stop_fps_index:]
            assert post_fps[-1][1].data == 0.0
            post_ok = samples['ok'][stop_ok_index:]
            for timed in (post_ok, post_fps):
                gaps = [b[0] - a[0] for a, b in zip(timed, timed[1:])]
                assert len(gaps) >= 2 and all(.8 <= gap <= 1.2 for gap in gaps), gaps
            result = subprocess.run(['ros2', 'topic', 'echo', '--once', prefix + 'stream_ok'],
                                    capture_output=True, text=True, timeout=8)
            with cli_log.open('a') as output:
                output.write('$ ros2 topic echo --once ' + prefix + 'stream_ok # input stopped\n')
                output.write(result.stdout + result.stderr + 'exit={}\n'.format(result.returncode))
            assert result.returncode == 0 and 'data: false' in result.stdout
            report = {
                'opencv': cv2.__version__, 'python': sys.version.split()[0],
                'images_received': len(samples['images']),
                'active_fps_samples': active_fps,
                'stream_false_after_last_frame_s': round(age, 3),
                'stream_false_after_pause_s': round(stale_at - stopped, 3),
                'heartbeat_gaps_s': [round(b[0] - a[0], 3) for a, b in zip(post_ok, post_ok[1:])],
                'jpeg_shape': list(decoded.shape), 'detector_udp_sockets_opened': 0,
                'scope': 'synthetic JPEG input through real detector -> real ROS subscribers and ROS CLI; no Webots/hardware',
            }
            print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
            print('PASS: real ROS JPEG/fps/stream_ok and CLI; steady 1 Hz heartbeat, 3 s stale threshold', flush=True)
        finally:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=5)
            print('CHILD STOP: rc={}'.format(child.returncode), flush=True)
            control.unlink(missing_ok=True)
            observer.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    if args.fixture:
        fixture(args.fixture)
    else:
        main()
