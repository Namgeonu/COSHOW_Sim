"""Adapter contracts exercised with real Humble types and DDS endpoints."""
import asyncio
import copy
import importlib
import json
import math
import threading
import time
import uuid
from types import SimpleNamespace

import pytest


def adapter_module():
    try:
        return importlib.import_module('dashboard.ros_io')
    except ModuleNotFoundError as exc:
        pytest.fail('The configured ROS adapter has not been implemented: {}'.format(exc))


@pytest.fixture
def config():
    prefix = '/adapter_test_' + uuid.uuid4().hex[:8]
    topics = {
        'pose_template': prefix + '/{cf}/position',
        'status_template': prefix + '/{cf}/health',
        'odom_template': prefix + '/{limo}/position',
        'limo_status_template': prefix + '/{limo}/health',
        'camera_template': prefix + '/camera/{cf}/jpeg',
        'camera_fps_template': prefix + '/camera/{cf}/rate',
        'camera_ok_template': prefix + '/camera/{cf}/available',
        'detections_template': prefix + '/{cf}/markers',
        'mission_state': prefix + '/narrative',
        'preflight_ready': prefix + '/ready',
        'preflight_status': prefix + '/checks',
    }
    types = {'topics': {
        'pose_template': 'geometry_msgs/msg/PoseStamped',
        'status_template': 'crazyflie_interfaces/msg/Status',
        'odom_template': 'nav_msgs/msg/Odometry',
        'limo_status_template': 'crazyflie_interfaces/msg/Status',
        'camera_template': 'sensor_msgs/msg/CompressedImage',
        'camera_fps_template': 'std_msgs/msg/Float32',
        'camera_ok_template': 'std_msgs/msg/Bool',
        'detections_template': 'coshow_interfaces/msg/MarkerDetections',
        'mission_state': 'std_msgs/msg/String',
        'preflight_ready': 'std_msgs/msg/Bool',
        'preflight_status': 'std_msgs/msg/String',
    }, 'services': {'land_template': 'crazyflie_interfaces/srv/Land',
                    'arm_template': 'crazyflie_interfaces/srv/Arm'},
        'actions': {'nav_template': 'nav2_msgs/action/NavigateToPose'}}
    robots = {
        'observer': {'kind': 'drone', 'role': 'observer', 'fleet_id': 'A'},
        'seeker': {'kind': 'drone', 'role': 'seeker', 'fleet_id': 'B'},
        'reserve': {'kind': 'drone', 'role': None, 'fleet_id': 'C'},
        'carrier': {'kind': 'limo', 'role': 'carrier', 'fleet_id': 'D'},
        'reserve_car': {'kind': 'limo', 'role': None, 'fleet_id': 'E'},
    }
    return SimpleNamespace(
        raw={'topics': topics, 'types': types,
             'services': {'land_template': prefix + '/{cf}/descend',
                          'arm_template': prefix + '/{cf}/enable'},
             'actions': {'nav_template': prefix + '/{limo}/travel'},
             'nodes': {'server': 'fixture_' + uuid.uuid4().hex[:8]}},
        robots=robots, drones=['observer', 'seeker'], limos=['carrier'], radio_counts={})


def test_interface_expansion_monitors_spares_without_creating_spare_control(config):
    specs, errors = adapter_module().interface_specs(config)
    statuses = [s['robot'] for s in specs if s['key'] == 'status_template']
    assert statuses == ['observer', 'seeker', 'reserve']
    odometry = [s['robot'] for s in specs if s['key'] == 'odom_template']
    assert odometry == ['carrier', 'reserve_car']
    assert len([s for s in specs if s['key'] == 'camera_template']) == 2
    assert {s['robot'] for s in specs if s['kind'] == 'services'} == set(config.drones)
    assert {s['robot'] for s in specs if s['kind'] == 'actions'} == set(config.limos)
    assert all(s['name'].startswith('/adapter_test_') for s in specs)


def test_unknown_configured_interface_is_not_silently_ignored(config):
    config.raw['topics']['new_unsupported_topic'] = '/unexpected'
    specs, errors = adapter_module().interface_specs(config)
    assert specs and any(e['key'] == 'new_unsupported_topic' for e in errors)


@pytest.fixture
def ros_types():
    return pytest.importorskip('rosidl_runtime_py.utilities')


def test_real_status_bits_and_pose_fields_ignore_header_time(ros_types):
    decode = adapter_module().decode
    status = ros_types.get_message('crazyflie_interfaces/msg/Status')()
    status.battery_voltage = 3.875
    status.rssi = 65
    status.supervisor_info = (status.SUPERVISOR_INFO_IS_ARMED |
                              status.SUPERVISOR_INFO_CAN_FLY |
                              status.SUPERVISOR_INFO_IS_TUMBLED)
    status.pm_state = status.PM_STATE_LOW_POWER
    assert decode('status', status) == {
        'battery_v': 3.875, 'rssi': 65, 'armed': True,
        'can_fly': True, 'tumbled': True, 'low_power': True}
    status.supervisor_info = 0
    assert decode('status', status)['armed'] is False
    for typename in ['geometry_msgs/msg/PoseStamped', 'nav_msgs/msg/Odometry']:
        message = ros_types.get_message(typename)()
        pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        pose.position.x, pose.position.y, pose.position.z = 1.0, 2.0, 3.0
        pose.orientation.z = math.sin(0.4)
        pose.orientation.w = math.cos(0.4)
        message.header.stamp.sec = 2147483647
        assert decode('pose', message) == pytest.approx(
            {'x': 1.0, 'y': 2.0, 'z': 3.0, 'yaw': 0.8})
        pose.position.x = float('nan')
        with pytest.raises(ValueError):
            decode('pose', message)


def test_decodes_detection_ids_and_rejects_non_object_json(ros_types):
    decode = adapter_module().decode
    message = ros_types.get_message('coshow_interfaces/msg/MarkerDetections')()
    marker_type = ros_types.get_message('coshow_interfaces/msg/MarkerDetection')
    message.markers = [marker_type(id=3), marker_type(id=13)]
    assert decode('detections', message) == [3, 13]
    string_type = ros_types.get_message('std_msgs/msg/String')
    for value in ['null', '[]', '42', 'bad json', '{"age": NaN}']:
        with pytest.raises(ValueError):
            decode('mission', string_type(data=value))


def test_camera_and_limo_conversions_use_real_configurable_message_fields(ros_types):
    decode = adapter_module().decode
    float_type = ros_types.get_message('std_msgs/msg/Float32')
    bool_type = ros_types.get_message('std_msgs/msg/Bool')
    status_type = ros_types.get_message('crazyflie_interfaces/msg/Status')
    assert decode('camera_fps', float_type(data=12.5)) == 12.5
    assert decode('camera_ok', bool_type(data=True)) is True
    assert decode('limo_status', status_type(battery_voltage=24.25)) == {'battery_v': 24.25}
    with pytest.raises(ValueError):
        decode('camera_fps', float_type(data=float('inf')))
    with pytest.raises(ValueError):
        decode('ready', float_type(data=1.0))


class Store:
    def __init__(self):
        self.values, self.errors, self.calls = {}, {}, []
        self.lock = threading.RLock()

    def _put(self, key, value):
        with self.lock:
            self.values[key] = value
            self.calls.append((key, value, time.monotonic(), threading.get_ident()))

    def receive(self, name, channel, value):
        self._put((name, channel), value)

    def mission(self, value):
        self._put('mission', value)

    def preflight(self, value):
        self._put('preflight', value)

    def ready(self, value):
        self._put('ready', value)

    def nodes(self, value):
        self._put('nodes', value)

    def unavailable(self, name, channel, reason):
        with self.lock:
            self.errors[(name, channel)] = reason


@pytest.fixture
def world(config, ros_types):
    import rclpy
    from rclpy.action import ActionServer
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                          qos_profile_sensor_data)

    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node(config.raw['nodes']['server'], context=context)
    specs, errors = adapter_module().interface_specs(config)
    publishers, services, actions = {}, [], []
    for spec in specs:
        if spec['kind'] == 'topics':
            typename = ros_types.get_message(spec['type'])
            if spec['channel'] in ('mission', 'preflight', 'ready'):
                qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            elif spec['channel'] == 'detections':
                qos = QoSProfile(depth=10)
            else:
                qos = qos_profile_sensor_data
            publishers[(spec['robot'], spec['channel'])] = (
                node.create_publisher(typename, spec['name'], qos), typename)
        elif spec['kind'] == 'services':
            typename = ros_types.get_service(spec['type'])
            services.append(node.create_service(typename, spec['name'], lambda req, res: res))
        else:
            typename = ros_types.get_action(spec['type'])
            actions.append(ActionServer(node, typename, spec['name'],
                                        execute_callback=lambda handle: typename.Result()))
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    stopped = threading.Event()

    def spin():
        while not stopped.is_set():
            executor.spin_once(timeout_sec=0.02)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    yield SimpleNamespace(node=node, publishers=publishers, specs=specs)
    stopped.set()
    thread.join(timeout=2)
    executor.shutdown()
    for action in actions:
        action.destroy()
    node.destroy_node()
    context.shutdown()


async def wait_until(predicate, tick=None, seconds=3):
    deadline = time.monotonic() + seconds
    while not predicate() and time.monotonic() < deadline:
        if tick is not None:
            tick()
        await asyncio.sleep(0.03)
    assert predicate(), 'ROS condition was not reached before the deadline'


def test_late_latched_json_and_best_effort_camera_on_actual_ros(config, world):
    async def exercise():
        store = Store()
        for channel, value in [('mission', '{"phase":"search"}'),
                               ('preflight', '{"ready":true}'), ('ready', True)]:
            publisher, message_type = world.publishers[(None, channel)]
            publisher.publish(message_type(data=value))
        await asyncio.sleep(0.1)
        io = adapter_module().ROSIO(config, store)
        await io.start()
        try:
            await wait_until(lambda: all(k in store.values for k in ('mission', 'preflight', 'ready')))
            assert store.values['ready'] is True
            assert store.values['mission']['phase'] == 'search'
            publisher, message_type = world.publishers[('observer', 'frame')]
            frame = message_type(format='jpeg', data=b'\xff\xd8\xff\xd9')
            await wait_until(lambda: ('observer', 'frame') in store.values,
                             lambda: publisher.publish(frame))
            assert store.values[('observer', 'frame')] == b'\xff\xd8\xff\xd9'
            await wait_until(lambda: store.values.get(('carrier', 'nav_ready')) is True)
            with store.lock:
                assert {call[3] for call in store.calls} == {io.thread.ident}
            assert not any(key[0] == 'reserve' for key in io.service_clients)
            assert set(io.action_clients) == {'carrier'}
        finally:
            await io.close()
        assert not io.thread.is_alive()

    asyncio.run(exercise())


def test_real_store_receives_latest_frame_while_asyncio_loop_is_blocked(config, world):
    from dashboard.state import TelemetryStore

    async def exercise():
        store = TelemetryStore(config)
        io = adapter_module().ROSIO(config, store)
        await io.start()
        publisher, message_type = world.publishers[('observer', 'frame')]
        try:
            await wait_until(lambda: publisher.get_subscription_count() > 0)
            # Deliberately keep the asyncio loop busy: ROS must still record the
            # receipt time and replace the one latest-frame slot on its own thread.
            started = time.monotonic()
            deadline = started + 0.5
            while time.monotonic() < deadline:
                publisher.publish(message_type(format='jpeg', data=b'latest'))
                time.sleep(0.02)
            assert store.latest_frames()['observer'][1] == b'latest'
            with store.lock:
                received_at = store.data['observer']['frame'][1]
            assert started <= received_at < time.monotonic()
            assert len(store.latest_frames()) == 1
        finally:
            await io.close()

    asyncio.run(exercise())


@pytest.mark.parametrize('channel,payload', [
    ('mission', {'led': None}),
    ('mission', {'led': {'observer': []}}),
    ('mission', {'cmd': []}),
    ('mission', {'cmd': {'observer': None}}),
    ('mission', {'cmd': {'observer': {'goal': ['go_to', 1.0, 2.0]}}}),
    ('mission', {'cmd': {'observer': {'t': 'yesterday'}}}),
    ('mission', {'phase': []}),
    ('mission', {'target_confirmed': 'false'}),
    ('mission', {'preflight_ready': 1}),
    ('mission', {'target_id': True}),
    ('mission', {'search_progress': {'observer': []}}),
    ('mission', {'search_progress': {'observer': 1.5}}),
    ('mission', {'missing_pose': [4]}),
    ('mission', {'P_N': ['1', 2]}),
    ('mission', {'P_N': {'x': '1', 'y': 2}}),
    ('mission', {'finder': {}}),
    ('mission', {'rescue_done_t': -1}),
    ('preflight', {'drones': ['observer']}),
    ('preflight', {'drones': {'observer': []}}),
    ('preflight', {'drones': {'observer': {'kal_ok': 'true'}}}),
    ('preflight', {'drones': {'observer': {'battery_v': '3.8'}}}),
    ('preflight', {'drones': {'observer': {'pose_err': [0, 'bad', 0]}}}),
    ('preflight', {'drones': {'observer': {'kal_range': [0, 0]}}}),
    ('preflight', {'stages': {'result': 'pass'}}),
    ('preflight', {'stages': [None]}),
    ('preflight', {'stages': [{'result': False}]}),
    ('preflight', {'stage': '3'}),
    ('preflight', {'ready': 1}),
    ('preflight', {'abort_reason': []}),
    ('preflight', {'t': 10 ** 400}),
])
def test_rejects_unsafe_nested_protocol_shape(channel, payload, ros_types):
    string_type = ros_types.get_message('std_msgs/msg/String')
    with pytest.raises(ValueError):
        adapter_module().decode(channel, string_type(data=json.dumps(payload)))


def test_known_optional_nulls_and_unknown_optional_keys_remain_compatible(ros_types):
    string_type = ros_types.get_message('std_msgs/msg/String')
    payloads = {
        'mission': {'phase': 'search', 'led': {'observer': 'blue'},
                    'cmd': {'observer': {'kind': 'go_to', 'goal': [1.0, 2.0, 0.4], 't': 1}},
                    'P_N': {'x': 1.0, 'y': 2.0, 'z': 0.4}, 'finder': None,
                    'mission_marker_id': None, 'target_id': None,
                    'target_confirmed': False, 'target_confirm_note': None,
                    'search_progress': {'seeker': 1}, 'missing_pose': [],
                    'extension': {'future': [None, 'supported']}},
        'preflight': {'stage': 0, 'ready': False, 'abort_reason': None,
                      'stages': [{'name': 'future_gate', 'result': 'pending'}],
                      'drones': {'observer': {'kal_ok': False, 'battery_v': None,
                                               'pose_err': None, 'kal_range': None}},
                      'extension': ['supported']},
    }
    for channel, payload in payloads.items():
        assert adapter_module().decode(channel, string_type(data=json.dumps(payload))) == payload
    payloads['mission']['P_N'] = None
    assert adapter_module().decode('mission', string_type(data=json.dumps(payloads['mission'])))['P_N'] is None


def test_real_store_keeps_valid_state_after_nested_json_and_numeric_errors(config, world):
    from dashboard.state import TelemetryStore

    async def exercise():
        store = TelemetryStore(config)
        io = adapter_module().ROSIO(config, store)
        await io.start()
        try:
            mission_pub, string_type = world.publishers[(None, 'mission')]
            preflight_pub, _ = world.publishers[(None, 'preflight')]
            good_mission = string_type(data='{"phase":"search","led":{"observer":"blue"}}')
            good_preflight = string_type(data='{"ready":false,"drones":{"observer":{"kal_ok":true}}}')
            await wait_until(lambda: store.snapshot()['mission'] is not None,
                             lambda: mission_pub.publish(good_mission))
            await wait_until(lambda: store.snapshot()['preflight'] is not None,
                             lambda: preflight_pub.publish(good_preflight))
            for publisher, payload in [(mission_pub, '{"led":null}'),
                                       (preflight_pub, '{"drones":["observer"]}'),
                                       (mission_pub, '{"cmd":{"observer":{"goal":["bad"]}}}')]:
                before = len(store.snapshot()['events'])
                await wait_until(lambda: len(store.snapshot()['events']) > before,
                                 lambda: publisher.publish(string_type(data=payload)))
            status_pub, status_type = world.publishers[('reserve', 'status')]
            before = len(store.snapshot()['events'])
            await wait_until(lambda: len(store.snapshot()['events']) > before,
                             lambda: status_pub.publish(status_type(battery_voltage=float('nan'))))
            valid_status = status_type(battery_voltage=3.875, rssi=55)
            await wait_until(lambda: store.snapshot()['robots']['reserve']['rssi'] == 55,
                             lambda: status_pub.publish(valid_status))
            state = store.snapshot()
            assert state['mission']['phase'] == 'search'
            assert state['robots']['observer']['led'] == 'blue'
            assert state['preflight']['drones']['observer']['kal_ok'] is True
            assert state['robots']['reserve']['battery_v'] == 3.875
            assert io.thread.is_alive()
        finally:
            await io.close()

    asyncio.run(exercise())


def test_missing_type_and_bad_message_do_not_stop_other_channels(config, world):
    async def exercise():
        changed = copy.deepcopy(config)
        changed.raw['types']['topics']['limo_status_template'] = 'absent_pkg/msg/NoStatus'
        store = Store()
        io = adapter_module().ROSIO(changed, store)
        await io.start()
        try:
            await wait_until(lambda: ('carrier', 'limo_status') in store.errors)
            bad_pub, string_type = world.publishers[(None, 'mission')]
            await wait_until(lambda: (None, 'mission') in store.errors,
                             lambda: bad_pub.publish(string_type(data='invalid')))
            status_pub, status_type = world.publishers[('reserve', 'status')]
            status = status_type(battery_voltage=3.9, rssi=57)
            await wait_until(lambda: ('reserve', 'status') in store.values,
                             lambda: status_pub.publish(status))
            assert store.values[('reserve', 'status')]['rssi'] == 57
            await wait_until(lambda: 'mission' in store.values,
                             lambda: bad_pub.publish(string_type(data='{"phase":"done"}')))
        finally:
            await io.close()

    asyncio.run(exercise())


def test_check_config_real_servers_pass_and_typo_does_not_self_publish(config, world):
    module = adapter_module()
    rows = module.check_config(config, timeout=3)
    assert rows and all(row['ok'] for row in rows), rows
    changed = copy.deepcopy(config)
    changed.raw['topics']['pose_template'] += '_misspelled'
    rows = module.check_config(changed, timeout=0.4)
    failures = [row for row in rows if not row['ok']]
    assert len(failures) == len(config.drones)
    assert all(row['key'] == 'pose_template' for row in failures)


def test_check_config_reports_type_mismatch_and_missing_package(config, world):
    module = adapter_module()
    changed = copy.deepcopy(config)
    changed.raw['types']['topics']['pose_template'] = 'std_msgs/msg/String'
    rows = module.check_config(changed, timeout=0.3)
    failures = [row for row in rows if not row['ok']]
    assert len(failures) == len(config.drones)
    assert all('geometry_msgs/msg/PoseStamped' in row['actual_types'] for row in failures)
    changed.raw['types']['topics']['pose_template'] = 'absent_pkg/msg/NoPose'
    rows = module.check_config(changed, timeout=0.3)
    failures = [row for row in rows if not row['ok']]
    assert len(failures) == len(config.drones)
    assert all('정보 없음' in row['detail'] for row in failures)


def test_check_config_subscribers_and_clients_never_count_as_servers(config, ros_types):
    async def exercise():
        module = adapter_module()
        store = Store()
        io = module.ROSIO(config, store)
        await io.start()
        try:
            # An observation-only graph contains every topic subscription and
            # service/action client name, but no data publishers or servers.
            rows = await asyncio.to_thread(module.check_config, config, 0.3)
            assert rows and not any(row['ok'] for row in rows), rows
            assert not any(row['actual_types'] for row in rows)
        finally:
            await io.close()

    asyncio.run(exercise())
