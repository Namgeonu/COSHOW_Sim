"""Camera startup regressions without opening an AI Deck UDP socket.

The Docker harness has no OpenCV. Compile the unchanged production
DroneChannel class body in isolation, using real ROS message/QoS types.
The launch module is imported and its real LaunchDescription is evaluated.
"""
import ast
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32, String
from coshow_interfaces.msg import MarkerDetections


ROOT = Path(__file__).resolve().parents[2]
AIDECK = ROOT / 'ros2_ws/src/aideck_aruco_ros'


@pytest.fixture
def channel():
    source = AIDECK / 'aideck_aruco_ros/aideck_aruco_node.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef)
               and item.name == 'DroneChannel')
    module = ast.Module(body=[cls], type_ignores=[])
    namespace = dict(Image=Image, CompressedImage=CompressedImage, String=String,
                     Float32=Float32, Bool=Bool, PoseStamped=PoseStamped,
                     MarkerDetections=MarkerDetections)
    exec(compile(module, str(source), 'exec'), namespace)
    rclpy.init()
    node = Node('test_aideck_channel')
    instance = namespace['DroneChannel'](
        node, SimpleNamespace(stream_name='cf231'), 'cf231_camera',
        qos_profile_sensor_data, QoSProfile(depth=10),
        {'publish_raw': True, 'publish_annotated': True, 'publish_compressed': True})
    yield instance
    node.destroy_node()
    rclpy.shutdown()


def test_display_state_is_ready_before_first_pose(channel):
    assert channel.window == 'AI-Deck UDP ArUco - cf231'
    assert channel.latest_display is None
    assert channel.latest_fps == 0.0
    assert channel.shape_logged is False
    assert channel.latest_found == []
    assert channel.window_created is False
    assert channel.pub_compressed.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT


def test_pose_callback_only_updates_pose(channel):
    display, detections = object(), [{'id': 3}]
    channel.latest_display = display
    channel.latest_fps = 10.0
    channel.shape_logged = True
    channel.latest_found = detections
    channel.window_created = True
    pose = PoseStamped()
    channel._on_pose(pose)
    assert channel.latest_pose is pose
    assert channel.latest_display is display
    assert channel.latest_fps == 10.0
    assert channel.shape_logged is True
    assert channel.latest_found is detections
    assert channel.window_created is True


@pytest.mark.parametrize('override,expected', [(None, True), ('false', False)])
def test_launch_compressed_default_overrides_yaml_and_is_configurable(monkeypatch, override, expected):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import Node as LaunchNode
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    path = AIDECK / 'launch/aideck_aruco.launch.py'
    spec = importlib.util.spec_from_file_location('aideck_launch_m1_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package: str(AIDECK))
    parameters = []

    class RecordingNode(LaunchNode):
        def __init__(self, **kwargs):
            parameters.extend(kwargs['parameters'])
            super().__init__(**kwargs)

    monkeypatch.setattr(module, 'Node', RecordingNode)
    description = module.generate_launch_description()
    context = LaunchContext()
    if override is not None:
        context.launch_configurations['publish_compressed'] = override
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    assert parameters[0].perform(context) == str(AIDECK / 'config/drones.yaml')
    evaluated = evaluate_parameters(context, normalize_parameters(parameters))
    assert evaluated[-1]['publish_compressed'] is expected
    assert 'image_qos' not in parameters[-1]
