"""Safety gates consume observations, never ROS or process side effects."""
import copy
from pathlib import Path
from types import SimpleNamespace

import pytest

from dashboard.checklist import evaluate, external_observation


@pytest.fixture
def setup():
    cfg = SimpleNamespace(
        raw={'freshness_s': {'pose': .8, 'status': 1., 'odom': 1.},
             'battery_v': {'warn': 3.75, 'block': 3.6}, 'camera_min_fps': 5.,
             'external_node_grace_s': 20., 'fleet': {'max_per_radio': 5},
             'nodes': {'server': 'radio_server', 'aideck': '/vision/deck',
                       'preflight': '/safety/check', 'bt': '/mission/tree'},
             'commands': {'env': {'SDL_VIDEODRIVER': 'dummy'}}},
        bt={'coshow': {'preflight': {'required': True}, 'emergency_land_on_exit': True,
                      'drones': {'reader': {'base': [0., 0.]}},
                      'limos': {'carrier': {'base': [1., 1.]}},
                      'searchers': ['reader'],
                      'search': {'lane_spacing': .4, 'lane_axis': 'y',
                                 'zones': {'reader': {'x': [0., 1.], 'y': [0., 1.]}}},
                      'tolerances': {'landed_z': .06, 'drone_at': .15, 'limo_at': .3},
                      'durations': {'land': 8.}, 'observe_point': {'x': 0., 'y': 0.},
                      'rescue_sec': 10.},
            'bt_runner': {'bt_visualiser': {'enabled': False}}},
        bt_path=Path('/example/bt.yaml'), errors=[], dashboard_ok=True, field_ok=True,
        field={}, robots={
            'reader': {'kind': 'drone', 'role': 'reader', 'fleet_id': 'D1', 'ip': '127.0.0.1'},
            'backup': {'kind': 'drone', 'role': None, 'fleet_id': 'D2', 'ip': '127.0.0.2'},
            'carrier': {'kind': 'limo', 'role': 'carrier', 'fleet_id': 'L1', 'ip': '127.0.0.3'},
            'reserve': {'kind': 'limo', 'role': None, 'fleet_id': 'L2', 'ip': '127.0.0.4'}})
    state = {
        'run': {'state': 'READY'},
        'robots': {
            'reader': {'status_age': .1, 'pose_age': .1, 'battery_v': 4.,
                       'armed': True, 'can_fly': True,
                       'camera': {'stream_ok': True, 'fps': 5., 'frame_age': .1},
                       'ping': {'ok': True, 'ip': '127.0.0.1', 'rtt_ms': 2.}},
            'backup': {'status_age': None, 'pose_age': None, 'battery_v': None,
                       'armed': False, 'ping': {'ok': False, 'ip': '127.0.0.2'}},
            'carrier': {'pose_age': .1, 'nav_ready': True, 'battery_v': None,
                        'ping': {'ok': True, 'ip': '127.0.0.3'}},
            'reserve': {'pose_age': None, 'battery_v': None,
                        'ping': {'ok': False, 'ip': '127.0.0.4'}}},
        'preflight': {'ready': True, 'abort_reason': None, 'age': .1,
                      'drones': {'reader': {'kal_ok': True, 'pose_ok': True, 'sup_ok': True}}},
        'mission': None,
        'stack': {'crazyflie_server': 'up', 'aideck': 'up', 'roster_hash': 'a',
                  'applied_hash': 'a', 'radio_counts': {'0': 5}}}
    context = {
        'nodes': [('radio_server', '/'), ('deck', '/vision'), ('check', '/safety')],
        'processes': {'preflight': {'alive': True, 'exited_at': None},
                      'bt': {'alive': False, 'exited_at': None}},
        'receipts': {'mission': [], 'preflight_status': [], 'preflight_ready': []},
        'env': {'SDL_VIDEODRIVER': 'dummy'}, 'ping_available': True, 'orphans': []}
    return state, cfg, context


def rows(setup):
    state, cfg, context = setup
    return {row['id']: row for row in evaluate(state, cfg, context, 100.)}


def test_healthy_roles_allow_start_despite_disconnected_spares(setup):
    items = rows(setup)
    assert all(row['ok'] for row in items.values() if row['blocking'])
    assert items['backup.pose']['status'] == 'skipped'
    assert items['backup.camera']['status'] == 'skipped'
    assert items['reserve.nav']['status'] == 'skipped'
    assert items['carrier.battery']['status'] == 'unavailable'
    assert not items['carrier.battery']['blocking']


@pytest.mark.parametrize('field,threshold,row', [
    ('pose_age', .8, 'pose'), ('status_age', 1., 'radio')])
@pytest.mark.parametrize('offset,expected', [(-.000001, True), (0., False), (.000001, False)])
def test_freshness_boundary_is_strict(setup, field, threshold, row, offset, expected):
    setup[0]['robots']['reader'][field] = threshold + offset
    assert rows(setup)['reader.' + row]['ok'] is expected


@pytest.mark.parametrize('voltage,ok,status', [
    (None, False, 'unavailable'), (3.599, False, 'fail'),
    (3.6, True, 'warning'), (3.749, True, 'warning'), (3.75, True, 'pass')])
def test_voltage_block_inclusive_warn_below_threshold(setup, voltage, ok, status):
    setup[0]['robots']['reader']['battery_v'] = voltage
    row = rows(setup)['reader.battery']
    assert row['ok'] is ok
    assert row['status'] == status


@pytest.mark.parametrize('fault', ['dead', 'stale', 'aborted', 'not_ready'])
def test_live_monitor_overrides_past_arm_success(setup, fault):
    state, cfg, context = setup
    state['preflight']['stages'] = [{'result': 'pass'}] * 4
    if fault == 'dead':
        context['processes']['preflight']['alive'] = False
    elif fault == 'stale':
        state['preflight']['age'] = 2.
    elif fault == 'aborted':
        state['preflight']['abort_reason'] = 'position lost'
    else:
        state['preflight']['ready'] = False
    assert rows(setup)['reader.armed']['ok'] is True
    assert rows(setup)['reader.monitor']['ok'] is False


def test_missing_ip_and_malformed_telemetry_fail_without_exceptions(setup):
    state, cfg, context = setup
    state['robots']['reader']['ping'] = {'ip': None, 'ok': None}
    cfg.robots['reader']['ip'] = None
    state['robots']['reader']['camera']['fps'] = float('nan')
    state['robots']['reader']['status_age'] = -1.
    state['robots']['reader']['battery_v'] = float('inf')
    items = rows(setup)
    for key in ('ping', 'camera', 'radio', 'battery'):
        assert items['reader.' + key]['ok'] is False


def test_spare_arm_warns_without_becoming_control_target(setup):
    setup[0]['robots']['backup'].update(armed=True, status_age=.1)
    row = rows(setup)['backup.spare_unarmed']
    assert row['ok'] is False and row['blocking'] is False
    assert row['status'] == 'warning'


def test_config_and_static_flags_block_with_absolute_path(setup):
    state, cfg, context = setup
    cfg.dashboard_ok = cfg.field_ok = False
    cfg.errors = ['invalid configuration']
    cfg.bt['coshow']['preflight']['required'] = False
    cfg.bt['coshow']['emergency_land_on_exit'] = False
    cfg.bt['bt_runner']['bt_visualiser']['enabled'] = True
    context['env'] = {}
    context['ping_available'] = False
    context['orphans'] = [{'pid': 123}]
    items = rows(setup)
    for key in ('dashboard_config', 'field_config', 'config_error_0', 'preflight_required',
                'emergency_land', 'bt_environment', 'ping_tool', 'orphans'):
        assert items['global.' + key]['blocking'] and not items['global.' + key]['ok']
    assert str(cfg.bt_path) in items['global.preflight_required']['detail']


def test_stack_hash_requires_known_match_and_external_stack_is_advisory(setup):
    state, cfg, context = setup
    state['stack']['applied_hash'] = None
    assert not rows(setup)['global.roster']['ok']
    state['stack']['crazyflie_server'] = 'external'
    row = rows(setup)['global.roster']
    assert row['status'] == 'skipped' and not row['blocking']
    state['stack']['radio_counts']['0'] = 6
    assert not rows(setup)['global.radios']['ok']


@pytest.mark.parametrize('alive,exited_at,count,external', [
    (True, None, 0, False), (True, None, 1, False), (True, None, 2, True),
    (False, None, 0, False), (False, None, 1, True),
    (False, 80., 1, False), (False, 79.99, 1, True), (False, 90., 2, True)])
def test_external_name_count_and_grace(setup, alive, exited_at, count, external):
    state, cfg, context = setup
    context['nodes'] = [('tree', '/mission')] * count + [('tree', '/different')]
    context['processes']['bt'] = {'alive': alive, 'exited_at': exited_at}
    result = external_observation('bt', cfg, context, 100.)
    assert result['external'] is external
    assert result['active'] is False
    if not alive and exited_at == 80.:
        assert '잔류' in result['detail']


@pytest.mark.parametrize('receipts,active', [
    ([97., 99.], False), ([97.001, 99.], True), ([98.], False),
    ([100.1, 100.2], False), ([80., 81.], False)])
def test_external_activity_uses_recent_receipts_not_graph_or_stamps(setup, receipts, active):
    state, cfg, context = setup
    context['receipts']['mission'] = receipts
    result = external_observation('bt', cfg, context, 100.)
    assert result['active'] is active
    assert result['external'] is active


def test_receipts_from_own_run_do_not_survive_exit_and_preflight_falls_back(setup):
    state, cfg, context = setup
    context['processes']['preflight'] = {'alive': False, 'exited_at': 99.}
    context['receipts']['preflight_status'] = [98., 99.]
    context['receipts']['preflight_ready'] = [99.1, 99.8]
    result = external_observation('preflight', cfg, context, 100.)
    assert result['active'] is True
    context['processes']['preflight']['alive'] = True
    assert external_observation('preflight', cfg, context, 100.)['active'] is False


def test_evaluation_does_not_mutate_inputs(setup):
    before = copy.deepcopy(setup)
    first = rows(setup)
    second = rows(setup)
    assert first == second
    assert setup == before


@pytest.mark.parametrize('location,key,row', [
    ('drones', 'reader', 'reader.base_config'),
    ('search', 'zones', 'reader.search_config'),
    ('tolerances', 'landed_z', 'global.bt_tolerances'),
    ('durations', 'land', 'global.bt_durations')])
def test_missing_bt_setting_identifies_only_the_affected_item(setup, location, key, row):
    cfg = setup[1]
    del cfg.bt['coshow'][location][key]
    item = rows(setup)[row]
    assert item['blocking'] and not item['ok']
    assert item['status'] == 'unavailable'
    assert '설정 없음' in item['detail'] and str(cfg.bt_path) in item['detail']
    assert rows(setup)['carrier.base_config']['ok'] is True
