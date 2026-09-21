"""Exercise the real simulated detector without opening its UDP transport.

ROS and OpenCV are existing detector dependencies. The lightweight dashboard
test image may omit OpenCV; the M4 integration container installs it explicitly.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import time

import pytest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = np = None
import rclpy
from rclpy.clock import ClockType
from rclpy.publisher import Publisher
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped


ROOT = Path(__file__).resolve().parents[2]


class NoNetworkSocket:
    """Only substitutes the pre-existing simulator socket; no real socket opens."""
    def bind(self, address):
        self.address = address

    def setblocking(self, flag):
        pass

    def sendto(self, data, address):
        pass

    def recvfrom(self, size):
        raise BlockingIOError

    def close(self):
        pass


def load_detector():
    path = ROOT / 'nodes/aruco_detector_node.py'
    spec = importlib.util.spec_from_file_location('sim_detector_m4_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def marker_frame():
    frame = np.full((220, 320), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    frame[30:110, 30:110] = cv2.aruco.generateImageMarker(dictionary, 3, 80)
    return frame


@pytest.fixture
def detector(monkeypatch, request):
    if cv2 is None:
        pytest.skip('real simulated detector tests require its existing OpenCV dependency')
    module = load_detector()
    module.socket = SimpleNamespace(socket=lambda *_: NoNetworkSocket(),
                                    AF_INET=2, SOCK_DGRAM=2)
    current = [100.0]
    module.time = SimpleNamespace(time=time.time, monotonic=lambda: current[0])
    args = ['--ros-args']
    for key, value in getattr(request, 'param', {}).items():
        args += ['-p', '{}:={}'.format(key, str(value).lower())]
    rclpy.init(args=args)
    messages = []
    publish = Publisher.publish

    def record(publisher, message):
        messages.append((publisher.topic_name, message))
        publish(publisher, message)

    monkeypatch.setattr(Publisher, 'publish', record)
    node = module.ArucoDetectorNode()
    node.width, node.height, node.fmt = 320, 220, 0
    try:
        yield SimpleNamespace(node=node, module=module, current=current,
                              messages=messages)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def received(detector, suffix):
    return [message for topic, message in detector.messages if topic.endswith(suffix)]


def test_image_topics_default_on_with_sensor_data_qos_and_steady_heartbeat(detector):
    node = detector.node
    topics = {publisher.topic_name: publisher for publisher in node.publishers}
    for suffix in ('image_annotated/compressed', 'fps', 'stream_ok'):
        name = '/aideck/cf230/' + suffix
        assert name in topics, 'missing image publication topic ' + name
    qos = topics['/aideck/cf230/image_annotated/compressed'].qos_profile
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE
    assert qos.depth == 5
    stats = [timer for timer in node.timers if timer.timer_period_ns == 1_000_000_000]
    assert len(stats) == 1
    assert stats[0].clock.clock_type == ClockType.STEADY_TIME


def test_real_marker_detection_and_full_size_annotated_jpeg_without_display(detector, monkeypatch):
    node = detector.node
    assert node.display is False
    quality = []
    encode = cv2.imencode

    def record(extension, frame, parameters):
        quality.append((extension, parameters, frame.copy()))
        return encode(extension, frame, parameters)

    monkeypatch.setattr(cv2, 'imencode', record)
    pose = PoseStamped()
    pose.pose.position.z = 1.0
    pose.pose.orientation.w = 1.0
    node._pose_cb(pose)
    node._process_frame(marker_frame().tobytes())
    images = received(detector, '/compressed')
    assert len(images) == 1, 'headless simulated frame must publish one JPEG'
    detections = received(detector, '/marker_detections')
    assert [marker.id for marker in detections[0].markers] == [3]
    assert detections[0].drone_pose == pose
    assert node.latest_pose == pose
    assert images[0].header == detections[0].header
    assert images[0].format == 'jpeg'
    assert bytes(images[0].data).startswith(b'\xff\xd8')
    decoded = cv2.imdecode(np.frombuffer(bytes(images[0].data), np.uint8), cv2.IMREAD_COLOR)
    assert decoded.shape == (220, 320, 3)
    assert quality[0][:2] == ('.jpg', [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert quality[0][2].shape == (220, 320, 3)
    # The original display's yellow center cross is present in the BGR output.
    assert quality[0][2][110, 160].tolist() == [0, 255, 255]


def test_fps_uses_received_frames_and_stream_expires_at_three_seconds(detector):
    node = detector.node
    assert hasattr(node, '_publish_image_stats'), 'missing image heartbeat'
    detector.current[0] = 101.0
    node._publish_image_stats()
    assert received(detector, '/fps')[-1].data == 0.0
    assert received(detector, '/stream_ok')[-1].data is False
    for _ in range(4):
        node._process_frame(marker_frame().tobytes())
    detector.current[0] = 102.0
    node._publish_image_stats()
    assert received(detector, '/fps')[-1].data == 4.0
    assert received(detector, '/stream_ok')[-1].data is True
    detector.current[0] = 103.999
    node._publish_image_stats()
    assert received(detector, '/stream_ok')[-1].data is True
    detector.current[0] = 104.0
    node._publish_image_stats()
    assert received(detector, '/fps')[-1].data == 0.0
    assert received(detector, '/stream_ok')[-1].data is False


@pytest.mark.parametrize('detector', [{'image_topic_prefix': '/preview/'}], indirect=True)
def test_topic_prefix_is_configurable(detector):
    names = {publisher.topic_name for publisher in detector.node.publishers}
    assert '/preview/cf230/image_annotated/compressed' in names
    assert not any(name.startswith('/aideck/') for name in names)


@pytest.mark.parametrize('detector', [{'publish_images': False, 'display': True}], indirect=True)
def test_disabling_images_keeps_original_display_and_detection(detector, monkeypatch):
    shown = []
    monkeypatch.setattr(cv2, 'imshow', lambda name, frame: shown.append((name, frame)))
    monkeypatch.setattr(cv2, 'waitKey', lambda delay: None)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: None)
    assert detector.node.get_parameter('publish_images').value is False
    detector.node._process_frame(marker_frame().tobytes())
    assert len(received(detector, '/marker_detections')) == 1
    assert not received(detector, '/compressed')
    assert not any(publisher.topic_name.startswith('/aideck/')
                   for publisher in detector.node.publishers)
    assert len(shown) == 1
    assert shown[0][0] == 'aruco cf230'
    assert shown[0][1].shape == (154, 224, 3)


def test_bad_frame_does_not_refresh_stream_or_publish_image(detector):
    assert hasattr(detector.node, '_publish_image_stats'), 'missing image heartbeat'
    detector.node._process_frame(b'bad')
    detector.current[0] = 101.0
    detector.node._publish_image_stats()
    assert not received(detector, '/compressed')
    assert received(detector, '/stream_ok')[-1].data is False
    assert received(detector, '/fps')[-1].data == 0.0


def test_jpeg_encode_failure_keeps_detection_and_display(detector, monkeypatch):
    shown = []
    detector.node.display = True
    monkeypatch.setattr(cv2, 'imshow', lambda name, frame: shown.append(frame))
    monkeypatch.setattr(cv2, 'waitKey', lambda delay: None)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: None)
    attempts = []

    def fail(*args):
        attempts.append(args)
        raise cv2.error('intentional encoder failure')

    monkeypatch.setattr(cv2, 'imencode', fail)
    detector.node._process_frame(marker_frame().tobytes())
    assert len(attempts) == 1, 'image publication must try the JPEG encoder'
    assert len(received(detector, '/marker_detections')) == 1
    assert not received(detector, '/compressed')
    assert len(shown) == 1


@pytest.mark.parametrize('before,after', [
    ('det.id = int(marker_id)', 'det.id = 0'),
    ('self.pub.publish(msg)', 'pass'),
    ("cv2.imshow(f'aruco {self.drone}', disp)", 'pass'),
    ("self.sock.bind(('0.0.0.0', listen_port))", "self.sock.bind(('127.0.0.1', listen_port))"),
    ('(0, 255, 255)', '(255, 0, 255)'),
    ('k = -p.z / uE[2]', 'k = p.z / uE[2]'),
])
def test_scope_rejects_detection_display_transport_and_projection_changes(before, after):
    from dashboard.tests import verify_m1_scope as scope
    assert hasattr(scope, 'check_sim_image_additions'), 'missing simulated detector AST protection'
    original = scope.original('nodes/aruco_detector_node.py')
    current = (ROOT / 'nodes/aruco_detector_node.py').read_text()
    scope.check_sim_image_additions(original, current)
    assert before in current
    with pytest.raises(AssertionError):
        scope.check_sim_image_additions(original, current.replace(before, after, 1))
