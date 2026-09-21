"""Pure phase derivation and tick-thread mission publication contracts."""
import copy
import asyncio
import json
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


@pytest.mark.parametrize('phase', [
    'waiting_poses', 'blocked_preflight', 'observe', 'handover', 'search',
    'capture', 'rescue_dispatch', 'rescue', 'return', 'done',
])
def test_phase_contract_and_purity(bt_nodes, home_bb, monkeypatch, phase):
    bb = home_bb
    monkeypatch.setitem(bt_nodes.C, 'preflight', {'required': True})
    if phase != 'observe':
        bb['mission_marker'] = {'found': True, 'id': 13}
    if phase == 'waiting_poses':
        bb['missing_pose'] = ['cf230']
        bb['preflight_ready'] = False
    elif phase == 'blocked_preflight':
        bb['preflight_ready'] = False
    elif phase == 'handover':
        bb['pose'][bt_nodes.C['observe_drone']]['z'] = 0.5
    elif phase in ('capture', 'rescue_dispatch', 'rescue', 'return', 'done'):
        bb['target_marker'] = {'found': True}
        bb['P_N'] = {'x': 0.0, 'y': 0.0, 'z': 0.8}
        bb['target_confirmed'] = phase != 'capture'
        if phase == 'rescue':
            # The display cannot perform IsRescue's elapsed-time latch.
            bb['limo_arrived']['limo_b'] = {'goal': (0.0, 0.0), 't': -1000.0}
        if phase in ('return', 'done'):
            bb['rescue_done_t'] = 90.0
        if phase == 'return':
            bb['pose'][bt_nodes.C['observe_drone']]['z'] = 0.5
    before = copy.deepcopy(bb)
    assert bt_nodes._phase(bb) == phase
    assert bb == before


def test_preflight_is_optional(bt_nodes, home_bb, monkeypatch):
    monkeypatch.setitem(bt_nodes.C, 'preflight', {'required': False})
    home_bb['preflight_ready'] = False
    assert bt_nodes._phase(home_bb) == 'observe'


@pytest.mark.parametrize('confirmed,expected', [(False, 'capture'), (True, 'rescue_dispatch')])
def test_target_latch_takes_priority_over_observer_movement(bt_nodes, home_bb, confirmed, expected):
    home_bb.update(mission_marker={'found': True}, target_marker={'found': True},
                   target_confirmed=confirmed, P_N={'x': 0.0, 'y': 0.0})
    home_bb['pose'][bt_nodes.C['observe_drone']]['z'] = 0.5
    assert bt_nodes._phase(home_bb) == expected


@pytest.mark.parametrize('offset,expected', [(0.0, 'rescue'), (1e-9, 'rescue_dispatch')])
def test_rescue_uses_same_inclusive_radius_as_is_rescue(bt_nodes, home_bb, offset, expected):
    home_bb.update(mission_marker={'found': True}, target_marker={'found': True},
                   target_confirmed=True, P_N={'x': 0.0, 'y': 0.0})
    radius = float(bt_nodes.TOL['limo_at']) + 0.1
    home_bb['limo_arrived']['limo_b'] = {'goal': (radius + offset, 0.0), 't': 99.0}
    assert bt_nodes._phase(home_bb) == expected


def test_rescue_requires_matching_arrival_record_not_current_pose(bt_nodes, home_bb):
    home_bb.update(mission_marker={'found': True}, target_marker={'found': True},
                   target_confirmed=True, P_N={'x': 0.0, 'y': 0.0})
    home_bb['pose']['limo_b'] = {'x': 0.0, 'y': 0.0}
    assert bt_nodes._phase(home_bb) == 'rescue_dispatch'
    home_bb['limo_arrived'] = {}
    assert bt_nodes._phase(home_bb) == 'rescue_dispatch'


@pytest.mark.parametrize('robot,field,extra', [
    ('cf231', 'z', 0.0), ('cf231', 'z', 1e-9),
    ('cf231', 'x', 0.0), ('cf231', 'x', 1e-9),
    ('limo_a', 'goal', 0.0), ('limo_a', 'goal', 1e-9),
])
def test_done_matches_existing_mission_complete_boundaries(bt_nodes, home_bb, robot, field, extra):
    bb = home_bb
    bb.update(mission_marker={'found': True}, target_marker={'found': True},
              target_confirmed=True, rescue_done_t=50.0)
    if field == 'z':
        bb['pose'][robot]['z'] = float(bt_nodes.TOL['landed_z']) + extra
    elif field == 'x':
        bb['pose'][robot]['x'] += float(bt_nodes.TOL['drone_at']) * 2 + extra
    else:
        bx, by = bt_nodes.LIMOS[robot]['base']
        bb['limo_arrived'][robot]['goal'] = (bx + float(bt_nodes.TOL['limo_at']) + extra, by)
    complete = bt_nodes.IsMissionComplete('complete', None)._check(None, bb)
    assert (bt_nodes._phase(bb) == 'done') == (complete == bt_nodes.Status.SUCCESS)


@pytest.fixture
def updater(bt_nodes, monkeypatch):
    rclpy.init()
    node = Node('test_dashboard_mission')
    monkeypatch.setattr(bt_nodes, '_install_emergency_land', lambda node: None)
    published = []
    publishers = {}
    create_publisher = node.create_publisher

    def capture(msg_type, topic, qos):
        pub = create_publisher(msg_type, topic, qos)
        publishers[topic] = pub
        send = pub.publish

        def publish(msg):
            published.append(json.loads(msg.data))
            send(msg)

        monkeypatch.setattr(pub, 'publish', publish)
        return pub

    monkeypatch.setattr(node, 'create_publisher', capture)
    tick = bt_nodes.UpdateBlackboard('update', SimpleNamespace(ros_bridge=SimpleNamespace(node=node)))
    clock = [100.0]
    monkeypatch.setattr(bt_nodes, 'now', lambda: clock[0])
    yield tick, clock, published, publishers
    node.destroy_node()
    rclpy.shutdown()


def test_mission_publishes_missing_pose_and_recovers_each_tick(bt_nodes, updater):
    tick, clock, messages, pubs = updater
    bb = {}
    assert tick._predicate(None, bb) is False
    assert messages[-1]['phase'] == 'waiting_poses'
    assert messages[-1]['missing_pose'] == list(bt_nodes.DRONES) + list(bt_nodes.LIMOS)
    for name, cfg in bt_nodes.DRONES.items():
        tick._pose[name] = {'x': cfg['base'][0], 'y': cfg['base'][1], 'z': 0.0}
    for name, cfg in bt_nodes.LIMOS.items():
        tick._pose[name] = {'x': cfg['base'][0], 'y': cfg['base'][1], 'yaw': 0.0}
    clock[0] += 0.1
    assert tick._predicate(None, bb) is True
    assert bb['missing_pose'] == []
    assert messages[-1]['phase'] == 'observe'
    assert messages[-1]['missing_pose'] == []
    qos = pubs['/coshow/mission_state'].qos_profile
    assert qos.depth == 1
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_mission_excludes_only_top_level_time_from_change_detection(updater):
    tick, clock, messages, pubs = updater
    bb = {}
    tick._predicate(None, bb)
    for t in (100.1, 100.5, 100.99):
        clock[0] = t
        tick._predicate(None, bb)
    assert len(messages) == 1
    clock[0] = 101.0
    tick._predicate(None, bb)
    assert len(messages) == 2
    bb['cmd']['cf231'] = {'kind': 'go_to', 'goal': [1.0, 0.0, 0.4], 't': 101.05}
    clock[0] = 101.1
    tick._predicate(None, bb)
    assert len(messages) == 3
    # In-place nested mutations must be detected, including command timestamps.
    bb['cmd']['cf231']['t'] = 101.15
    clock[0] = 101.2
    tick._predicate(None, bb)
    assert len(messages) == 4
    assert messages[-1]['cmd']['cf231']['t'] == 101.15
    assert messages[-1]['t'] == 101.2


def test_mission_schema_and_integer_progress(updater):
    tick, clock, messages, pubs = updater
    bb = {'mission_marker': {'found': True, 'id': 13},
          'search_progress': {'cf231': 1}, 'led': {'cf231': 'blue'}}
    tick._predicate(None, bb)
    assert set(messages[-1]) == {
        'phase', 't', 'mission_marker_id', 'target_id', 'finder', 'P_N',
        'target_confirmed', 'target_confirm_note', 'search_progress', 'missing_pose',
        'preflight_required', 'preflight_ready', 'cmd', 'led', 'rescue_done_t',
    }
    assert messages[-1]['search_progress'] == {'cf231': 1}
    assert messages[-1]['mission_marker_id'] == 13
    assert messages[-1]['target_confirm_note'] is None


@pytest.mark.parametrize('action_name,attrs,goal', [
    ('DroneGoTo', {'x': 0.0, 'y': 0.0, 'z': 1.0}, [0.0, 0.0, 1.0]),
    ('DroneTakeoff', {'height': 1.0}, [1.0]),
    ('DroneLand', {}, []),
])
def test_drone_service_command_goal_contains_only_numbers(
        bt_nodes, updater, monkeypatch, action_name, attrs, goal):
    tick, clock, messages, pubs = updater
    bb = {'cmd': {}, 'pose': {}, 'now': clock[0]}
    calls = []
    client = SimpleNamespace(wait_for_service=lambda **kw: True,
                             call_async=calls.append)
    action = getattr(bt_nodes, action_name)(
        'numeric_goal', SimpleNamespace(ros_bridge=tick.ros), robot='cf230', **attrs)
    monkeypatch.setattr(action, '_client_for', lambda robot: client)
    assert asyncio.run(action.run(None, bb)) == bt_nodes.Status.RUNNING
    assert asyncio.run(action.run(None, bb)) == bt_nodes.Status.RUNNING
    assert len(calls) == 1, 'telemetry changes must not defeat service deduplication'
    tick._predicate(None, bb)
    command = messages[-1]['cmd']['cf230']
    assert command['goal'] == goal
    assert all(isinstance(value, (float, int)) for value in command['goal'])


@pytest.mark.parametrize('failure', ['phase', 'serialize', 'publish'])
@pytest.mark.parametrize('poses_available', [False, True])
def test_mission_telemetry_errors_do_not_interrupt_control_ticks(
        bt_nodes, updater, monkeypatch, failure, poses_available):
    tick, clock, messages, pubs = updater
    bb = {}
    warnings = []
    monkeypatch.setattr(tick.ros.node, 'get_logger',
                        lambda: SimpleNamespace(warning=warnings.append))
    if poses_available:
        for name, cfg in dict(bt_nodes.DRONES, **bt_nodes.LIMOS).items():
            tick._pose[name] = {'x': cfg['base'][0], 'y': cfg['base'][1], 'z': 0.0}

    def broken(*args, **kwargs):
        raise RuntimeError('injected telemetry failure')

    with monkeypatch.context() as failing:
        if failure == 'phase':
            failing.setattr(bt_nodes, '_phase', broken)
        elif failure == 'serialize':
            bb['led'] = {'cf230': object()}
        else:
            failing.setattr(pubs['/coshow/mission_state'], 'publish', broken)
        for _ in range(3):
            assert tick._predicate(None, bb) is poses_available
            clock[0] += 0.1
    assert len(warnings) == 1
    bb['led'] = {}
    assert tick._predicate(None, bb) is poses_available
    assert messages[-1]['phase'] == ('observe' if poses_available else 'waiting_poses')


def test_mission_warning_failure_cannot_interrupt_control_ticks(updater, monkeypatch):
    tick, clock, messages, pubs = updater
    warnings = []

    def broken_publish(message):
        raise RuntimeError('publisher unavailable')

    def broken_warning(message):
        warnings.append(message)
        raise RuntimeError('logger unavailable')

    monkeypatch.setattr(pubs['/coshow/mission_state'], 'publish', broken_publish)
    monkeypatch.setattr(tick.ros.node, 'get_logger',
                        lambda: SimpleNamespace(warning=broken_warning))
    for _ in range(2):
        assert tick._predicate(None, {}) is False
    assert len(warnings) == 1
