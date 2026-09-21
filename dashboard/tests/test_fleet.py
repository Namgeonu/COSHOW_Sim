"""M6 fleet generation and owned stack control regressions; no hardware."""
import asyncio
import copy
import hashlib
import importlib
import importlib.util
from pathlib import Path
import shlex
import sys
import time

import pytest
import yaml

from dashboard.config import load_config
from dashboard.checklist import evaluate
from dashboard.state import TelemetryStore


ROOT = Path(__file__).resolve().parents[2]


def fleet_module():
    assert importlib.util.find_spec('dashboard.fleet') is not None, 'M6 fleet backend missing'
    return importlib.import_module('dashboard.fleet')


@pytest.fixture
def fleet_cfg(tmp_path):
    source = load_config(mock=True)
    raw = copy.deepcopy(source.raw)
    raw['roster_file'] = str(tmp_path / 'run/roster.yaml')
    raw['crazyflies_template'] = str(tmp_path / 'crazyflies.template.yaml')
    raw['aideck_template'] = str(tmp_path / 'drones.template.yaml')
    raw['commands']['bt_cwd'] = str(ROOT / 'bt')
    raw['commands']['bt'] = 'python3 main.py --config ' + shlex.quote(str(tmp_path / 'bt.yaml'))
    template = yaml.safe_load((ROOT / 'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml').read_text())
    for role, item in zip(source.drones, raw['fleet']['drones']):
        template['robots'][role] = dict(enabled=True, uri=item['uri'], type='cf21', initial_position=[99, 99, 0])
    template['sentinel'] = {'preserve': ['operator-owned', 42]}
    aideck = yaml.safe_load((ROOT / 'ros2_ws/src/aideck_aruco_ros/config/drones.yaml').read_text())
    for name, value in (('dashboard.yaml', raw), ('field.yaml', source.field), ('bt.yaml', source.bt),
                        ('crazyflies.template.yaml', template), ('drones.template.yaml', aideck)):
        (tmp_path / name).write_text(yaml.safe_dump(value, sort_keys=False))
    cfg = load_config(tmp_path / 'dashboard.yaml')
    assert not cfg.errors, cfg.errors
    return cfg


def test_concurrent_roster_saves_reject_stale_expected_hash_before_any_write(fleet_cfg):
    fleet = fleet_module()
    manager = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
    original_hash = manager.generated.roster_hash
    first, stale = dict(fleet_cfg.roster), dict(fleet_cfg.roster)
    first[fleet_cfg.drones[0]] = fleet_cfg.raw['fleet']['drones'][6]['id']
    stale[fleet_cfg.drones[1]] = fleet_cfg.raw['fleet']['drones'][7]['id']
    async def exercise():
        return await asyncio.gather(manager.save_roster(first, expected_hash=original_hash),
                                    manager.save_roster(stale, expected_hash=original_hash),
                                    return_exceptions=True)
    saved, rejected = asyncio.run(exercise())
    assert saved == first
    assert isinstance(rejected, ValueError) and '로스터' in str(rejected)
    assert manager.cfg.roster == first
    assert yaml.safe_load(manager.roster_path.read_text()) == first
    assert manager.generated.roster_hash != original_hash
    assert manager.generated.files == fleet.generate_config(fleet_cfg, first).files


def test_generates_three_files_with_only_template_robots_replaced(fleet_cfg):
    fleet = fleet_module()
    generated = fleet.generate_config(fleet_cfg)
    assert set(generated.files) == {'crazyflies.generated.yaml', 'drones.generated.yaml', 'preflight.generated.yaml'}
    template_path = fleet_cfg.resolve(fleet_cfg.raw['crazyflies_template'])
    original_bytes = template_path.read_bytes()
    template = yaml.safe_load(original_bytes)
    generated_cf = yaml.safe_load(generated.files['crazyflies.generated.yaml'])
    assert {k: v for k, v in generated_cf.items() if k != 'robots'} == {
        k: v for k, v in template.items() if k != 'robots'}
    assert len(generated_cf['robots']) == 10
    assert all(row['enabled'] is True and row['type'] == 'cf21' for row in generated_cf['robots'].values())
    assert template_path.read_bytes() == original_bytes
    assert len(fleet_cfg.robots) == 14
    for role in fleet_cfg.drones:
        robot = generated_cf['robots'][role]
        assert robot['initial_position'] == fleet_cfg.bt['coshow']['drones'][role]['base'] + [0.0]
        physical = next(d for d in fleet_cfg.raw['fleet']['drones'] if d['id'] == fleet_cfg.roster[role])
        assert robot['uri'] == physical['uri']
    positions = [v['initial_position'] for k, v in generated_cf['robots'].items() if k not in fleet_cfg.drones]
    assert len({tuple(p) for p in positions}) == 6
    area = fleet_cfg.field['spare_area']
    assert all(area['x'][0] <= p[0] <= area['x'][1] and area['y'][0] <= p[1] <= area['y'][1]
               and p[2] == 0.0 for p in positions)


def test_camera_roles_only_and_preflight_tokens_come_from_bt(fleet_cfg):
    generated = fleet_module().generate_config(fleet_cfg)
    camera = yaml.safe_load(generated.files['drones.generated.yaml'])
    original = yaml.safe_load(fleet_cfg.resolve(fleet_cfg.raw['aideck_template']).read_text())
    key = next(iter(original))
    rows = camera[key]['ros__parameters']['drones']
    assert rows == ['{},{},{}'.format(role, fleet_cfg.robots[role]['ip'], 5001 + index)
                    for index, role in enumerate(fleet_cfg.drones)]
    original[key]['ros__parameters']['drones'] = rows
    assert camera == original
    tokens = generated.preflight_argv
    assert yaml.safe_load(generated.files['preflight.generated.yaml']) == {'argv': tokens}
    assert next(token for token in tokens if token.startswith('drones:=')) == 'drones:=[' + ','.join(fleet_cfg.drones) + ']'
    for axis, label in ((0, 'expected_x'), (1, 'expected_y')):
        text = next(token.split(':=', 1)[1] for token in tokens if token.startswith(label + ':='))
        assert yaml.safe_load(text) == [fleet_cfg.bt['coshow']['drones'][role]['base'][axis] for role in fleet_cfg.drones]


def test_combined_hash_is_deterministic_and_changes_for_any_generated_input(fleet_cfg):
    fleet = fleet_module()
    first = fleet.generate_config(fleet_cfg)
    assert first.roster_hash == fleet.generate_config(fleet_cfg).roster_hash
    assert len(first.roster_hash) == 64
    assert first.roster_hash == hashlib.sha256(b''.join(first.files[name] for name in sorted(first.files))).hexdigest()
    fleet_cfg.bt['coshow']['drones'][fleet_cfg.drones[0]]['base'][0] += .25
    second = fleet.generate_config(fleet_cfg)
    assert second.roster_hash != first.roster_hash
    assert second.files['preflight.generated.yaml'] != first.files['preflight.generated.yaml']
    assert second.files['crazyflies.generated.yaml'] != first.files['crazyflies.generated.yaml']


@pytest.mark.parametrize('problem', ['duplicate_role', 'missing_role', 'unknown_id', 'missing_uri',
                                    'duplicate_uri', 'six_on_radio', 'missing_camera_ip', 'small_grid'])
def test_generation_rejects_invalid_assignment_or_unusable_inventory(fleet_cfg, problem):
    fleet = fleet_module()
    role = fleet_cfg.drones[0]
    inventory = fleet_cfg.raw['fleet']['drones']
    if problem == 'duplicate_role':
        fleet_cfg.roster[fleet_cfg.drones[1]] = fleet_cfg.roster[role]
    elif problem == 'missing_role':
        fleet_cfg.roster.pop(role)
    elif problem == 'unknown_id':
        fleet_cfg.roster[role] = 'unknown'
    elif problem == 'missing_uri':
        inventory[0]['uri'] = None
    elif problem == 'duplicate_uri':
        inventory[-1]['uri'] = inventory[0]['uri']
    elif problem == 'six_on_radio':
        for index, drone in enumerate(inventory[:6]):
            drone['uri'] = 'radio://0/80/2M/{:010X}'.format(index + 1)
    elif problem == 'missing_camera_ip':
        inventory[0]['aideck_ip'] = None
    elif problem == 'small_grid':
        fleet_cfg.field['spare_area'] = {'x': [0, .1], 'y': [0, .1], 'pitch': .4}
    with pytest.raises(ValueError):
        fleet.generate_config(fleet_cfg)


@pytest.mark.parametrize('failure', ['readonly_roster', 'parent_is_file', 'missing_config', 'malformed_path'])
def test_fleet_initialization_preserves_degraded_state_on_configuration_io_failure(fleet_cfg, tmp_path, monkeypatch, failure):
    fleet = fleet_module()
    if failure == 'readonly_roster':
        def denied(path, data):
            raise PermissionError('operator roster is read-only')
        monkeypatch.setattr(fleet, '_atomic_write', denied)
    elif failure == 'parent_is_file':
        parent = tmp_path / 'not_a_directory'
        parent.write_text('preserve this operator file')
        fleet_cfg.raw['roster_file'] = str(parent / 'roster.yaml')
    elif failure == 'missing_config':
        fleet_cfg = load_config(tmp_path / 'missing-dashboard.yaml')
    else:
        raw = yaml.safe_load(fleet_cfg.path.read_text())
        raw['roster_file'] = None
        fleet_cfg.path.write_text(yaml.safe_dump(raw))
        fleet_cfg = load_config(fleet_cfg.path)
    store = TelemetryStore(fleet_cfg)
    store.set_stack(applied_hash='previously-applied')
    manager = fleet.FleetManager(fleet_cfg, store, run_dir=tmp_path / 'isolated-generated')
    assert manager.generated is None and not manager.children
    snapshot = store.snapshot()
    assert snapshot['stack']['generation_error']
    assert snapshot['stack']['roster_hash'] is None
    assert snapshot['stack']['applied_hash'] == 'previously-applied'
    assert snapshot['run']['state'] == 'IDLE'
    assert any(row.startswith('플릿 생성:') for row in fleet_cfg.errors)
    assert any(row['level'] == 'warning' for row in snapshot['events'])
    assert not (tmp_path / 'isolated-generated/crazyflies.generated.yaml').exists()


def test_initial_roster_permission_failure_recovers_and_persists_roster(fleet_cfg, monkeypatch):
    fleet = fleet_module()
    original = fleet._atomic_write
    def denied(path, data):
        raise PermissionError('roster not writable yet')
    monkeypatch.setattr(fleet, '_atomic_write', denied)
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    error = manager._generation_error
    assert error in fleet_cfg.errors
    monkeypatch.setattr(fleet, '_atomic_write', original)
    assert manager.regenerate() is not None
    assert yaml.safe_load(manager.roster_path.read_text()) == fleet_cfg.roster
    assert store.stack['generation_error'] is None and error not in fleet_cfg.errors


def test_malformed_persisted_mock_roster_blocks_generation_until_valid_save(fleet_cfg, tmp_path):
    fleet = fleet_module()
    cfg = load_config(mock=True)
    isolated = tmp_path / 'mock'
    isolated.mkdir()
    roster_path = isolated / 'roster.yaml'
    roster_path.write_text('this is not a mapping')
    store = TelemetryStore(cfg)
    manager = fleet.FleetManager(cfg, store, run_dir=isolated)
    assert manager.generated is None and store.stack['generation_error']
    assert roster_path.read_text() == 'this is not a mapping'
    asyncio.run(manager.save_roster(dict(cfg.roster), expected_hash=None))
    assert manager.generated is not None and store.stack['generation_error'] is None


def test_roster_write_failure_keeps_real_http_and_websocket_available(tmp_path, monkeypatch):
    from aiohttp.test_utils import TestClient, TestServer
    from dashboard.server import Dashboard
    fleet = fleet_module()
    monkeypatch.setattr(fleet, 'HERE', tmp_path)
    def denied(path, data):
        raise PermissionError('test roster is read-only')
    monkeypatch.setattr(fleet, '_atomic_write', denied)
    async def exercise():
        app = Dashboard(load_config(mock=True), mock=True)
        async with TestClient(TestServer(app.app())) as client:
            assert (await client.get('/visitor.html')).status == 200
            assert (await client.get('/api/admin')).status == 200
            async with client.ws_connect('/ws') as socket:
                assert (await socket.receive_json(timeout=2))['type'] == 'hello'
                state = await socket.receive_json(timeout=2)
                assert state['type'] == 'state'
                assert 'read-only' in state['stack']['generation_error']
                assert any(row['blocking'] and not row['ok'] for row in state['checklist'])
            assert app.fleet.generated is None
    asyncio.run(exercise())


def test_bootstrap_persists_template_roles_regardless_of_enabled_flag(fleet_cfg):
    fleet = fleet_module()
    path = fleet_cfg.resolve(fleet_cfg.raw['crazyflies_template'])
    template = yaml.safe_load(path.read_text())
    template['robots'][fleet_cfg.drones[1]]['enabled'] = False
    path.write_text(yaml.safe_dump(template))
    cfg = load_config(fleet_cfg.path)
    store = TelemetryStore(cfg)
    manager = fleet.FleetManager(cfg, store)
    persisted = yaml.safe_load(cfg.resolve(cfg.raw['roster_file']).read_text())
    assert persisted == cfg.roster
    assert persisted == fleet_cfg.roster
    assert store.stack['roster_hash'] is not None
    assert store.stack['generation_error'] is None
    assert not manager.children


def test_save_roster_is_persistent_refreshes_metadata_and_invalidates_applied_hash(fleet_cfg):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    notified = []
    manager = fleet.FleetManager(fleet_cfg, store, on_reconfigure=lambda cfg: notified.append(cfg.roster.copy()))
    previous_hash = store.stack['roster_hash']
    store.set_stack(applied_hash=previous_hash)
    replacement = dict(fleet_cfg.roster)
    replacement[fleet_cfg.drones[1]] = fleet_cfg.raw['fleet']['drones'][6]['id']
    store.receive(fleet_cfg.drones[1], 'frame', b'old physical camera')
    asyncio.run(manager.save_roster(replacement, expected_hash=manager.generated.roster_hash))
    assert fleet_cfg.roster == replacement
    assert store.cfg is fleet_cfg
    assert fleet_cfg.robots[fleet_cfg.drones[1]]['fleet_id'] == replacement[fleet_cfg.drones[1]]
    assert len(fleet_cfg.robots) == 14 and len(store.data) == 14
    assert not store.latest_frames()
    assert notified == [replacement]
    assert load_config(fleet_cfg.path).roster == replacement
    assert store.stack['applied_hash'] == previous_hash
    assert store.stack['roster_hash'] != previous_hash


@pytest.mark.parametrize('state', ['CHECKING', 'READY', 'RUNNING', 'DONE', 'LANDING', 'ABORTED'])
def test_roster_and_stack_mutations_rejected_outside_idle(fleet_cfg, state):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    before = manager.roster_path.read_bytes()
    store.set_run(state=state)
    for action in (lambda: manager.save_roster(fleet_cfg.roster), manager.start_stack, manager.restart_stack, manager.stop_stack):
        with pytest.raises(ValueError, match='IDLE'):
            asyncio.run(action())
    assert manager.roster_path.read_bytes() == before
    assert not manager.children


def test_recommendation_uses_fresh_link_and_battery_without_saving(fleet_cfg):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg, clock=lambda: 10.0)
    manager = fleet.FleetManager(fleet_cfg, store)
    before = manager.roster_path.read_bytes()
    candidates = list(fleet_cfg.robots)[:10]
    for index, name in enumerate(candidates):
        store.receive(name, 'status', {'battery_v': 3.8 + index * .01, 'armed': False})
    suggestion = manager.recommend_roster()
    assert list(suggestion.values()) == [fleet_cfg.robots[name]['fleet_id'] for name in reversed(candidates[-4:])]
    assert manager.roster_path.read_bytes() == before
    assert fleet_cfg.roster != suggestion


def test_external_stack_is_recognized_and_never_restarted_or_owned(fleet_cfg):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    for key in ('server', 'aideck'):
        full = fleet_cfg.raw['nodes'][key]
        namespace, _, node_name = ('/' + full.strip('/')).rpartition('/')
        store.context['nodes'].append((node_name, namespace or '/'))
    manager.refresh()
    assert store.stack['crazyflie_server'] == store.stack['aideck'] == 'external'
    assert not manager.children
    with pytest.raises(ValueError, match='외부'):
        asyncio.run(manager.restart_stack())
    assert store.stack['applied_hash'] is None


def test_save_publishes_busy_lease_during_async_reconfiguration(fleet_cfg):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    observed = []

    async def callback(cfg):
        observed.append(store.stack.get('busy'))
        await asyncio.sleep(0)

    manager = fleet.FleetManager(fleet_cfg, store, on_reconfigure=callback)
    asyncio.run(manager.save_roster(fleet_cfg.roster, expected_hash=manager.generated.roster_hash))
    assert observed == [True]
    assert store.stack.get('busy') is False


def test_generation_failure_recovers_without_old_checklist_error(fleet_cfg):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    fleet_cfg.raw['fleet']['drones'][0]['uri'] = None
    manager = fleet.FleetManager(fleet_cfg, store)
    error = next(e for e in fleet_cfg.errors if e.startswith('플릿 생성:'))
    assert store.stack['roster_hash'] is None
    fleet_cfg.raw['fleet']['drones'][0]['uri'] = 'radio://0/80/2M/0000000001'
    assert manager.regenerate() is not None
    assert error not in fleet_cfg.errors
    assert store.stack['generation_error'] is None


def write_dummy(tmp_path):
    script = tmp_path / 'owned stack child.py'
    script.write_text('''import json, os, resource, signal, sys, time
def stop(sig, frame):
    print('SIGINT', flush=True)
    raise SystemExit(0)
signal.signal(signal.SIGINT, stop)
print(json.dumps(dict(pid=os.getpid(), pgid=os.getpgrp(), cwd=os.getcwd(),
                     marker=os.environ.get('M6_TEST_ENV'), path=bool(os.environ.get('PATH')),
                     core=list(resource.getrlimit(resource.RLIMIT_CORE)))), flush=True)
while True:
    time.sleep(.05)
''')
    return script


def test_owned_stack_start_restart_preserves_own_graph_grace_and_hash(fleet_cfg, tmp_path):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    script = write_dummy(tmp_path)
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(script))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    fleet_cfg.raw['commands']['env'] = {'M6_TEST_ENV': 'merged'}

    async def run():
        import json
        try:
            await manager.start_stack()
            original = dict(manager.children)
            assert store.stack['applied_hash'] == store.stack['roster_hash']
            for kind, child in original.items():
                assert child.pid != __import__('os').getpgrp()
                assert int(child.pid_path.read_text()) == child.pid
                deadline = time.monotonic() + 3
                while not child.log_path.read_text() and time.monotonic() < deadline:
                    await asyncio.sleep(.01)
                report = json.loads(child.log_path.read_text().splitlines()[0])
                assert report['core'] == [0, 0] and report['pgid'] == child.pid
                assert report['marker'] == 'merged' and report['path'] is True
                assert report['cwd'] == str(fleet_cfg.bt_cwd.parent)
            # ROS nodes from our old processes can remain in DDS after SIGINT.
            store.context['nodes'] = [(fleet_cfg.raw['nodes'][key], '/') for key in ('server', 'aideck')]
            await manager.restart_stack()
            assert all(manager.children[kind].pid != child.pid for kind, child in original.items())
            assert all(child.sigint_sent and child.returncode is not None for child in original.values())
            assert all(child.log_path.read_text().count('SIGINT') == 1 for child in original.values())
            assert store.stack['applied_hash'] == store.stack['roster_hash']
        finally:
            await cleanup_dummy_children(manager.children)
            await manager.close()
        assert not manager.children
        assert not list(manager.run_dir.glob('*.pid'))
    asyncio.run(run())


def test_second_spawn_failure_keeps_first_child_and_never_applies_hash(fleet_cfg, tmp_path):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    script = write_dummy(tmp_path)
    fleet_cfg.raw['commands']['crazyflie_server'] = shlex.quote(sys.executable) + ' ' + shlex.quote(str(script))
    fleet_cfg.raw['commands']['aideck'] = '/definitely_missing_m6_binary'

    async def run():
        with pytest.raises(OSError):
            await manager.start_stack()
        children = dict(manager.children)
        try:
            assert all(child.returncode is None and not child.sigint_sent for child in children.values())
            assert store.stack['applied_hash'] is None
            assert len(list(manager.run_dir.glob('*.pid'))) == 1
            await manager.close()
            assert all(child.returncode is None for child in children.values())
        finally:
            await cleanup_dummy_children(children)
    asyncio.run(run())


def test_owned_process_exit_invalidates_hash_even_with_retained_ros_node(fleet_cfg, tmp_path):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    script = write_dummy(tmp_path)
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(script))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)

    async def run():
        try:
            await manager.start_stack()
            child = manager.children['crazyflie_server']
            store.context['nodes'] = [(fleet_cfg.raw['nodes']['server'], '/')]
            child.proc.kill()
            await child.proc.wait()
            manager.refresh()
            assert store.stack['applied_hash'] is None
            assert store.stack['crazyflie_server'] == 'down'
            assert store.context['stack_processes']['crazyflie_server']['expected_nodes'] == 1
            assert any(event['level'] == 'error' and '프로세스 종료' in event['text'] for event in store.events)
        finally:
            await cleanup_dummy_children(manager.children)
            await manager.close()
    asyncio.run(run())


@pytest.mark.parametrize('template', ['sh -c true', 'python3 worker.py | cat', 'python3 $WORKER',
                                     'python3 worker.py > result', 'python3 worker.py; touch bad'])
def test_stack_commands_reject_shell_syntax_before_spawning(fleet_cfg, template):
    fleet = fleet_module()
    store = TelemetryStore(fleet_cfg)
    manager = fleet.FleetManager(fleet_cfg, store)
    fleet_cfg.raw['commands']['crazyflie_server'] = template
    with pytest.raises(ValueError):
        asyncio.run(manager.start_stack())
    assert not manager.children
    assert store.stack['applied_hash'] is None
    assert store.stack.get('busy') is False


def test_mock_roster_save_is_isolated_persistent_and_keeps_mock_bt_overrides(fleet_cfg, tmp_path):
    fleet = fleet_module()
    bt = yaml.safe_load(fleet_cfg.bt_path.read_text())
    bt['coshow']['preflight']['required'] = False
    bt['bt_runner']['bt_visualiser']['enabled'] = True
    fleet_cfg.bt_path.write_text(yaml.safe_dump(bt))
    production_roster = fleet_cfg.resolve(fleet_cfg.raw['roster_file'])
    production_roster.parent.mkdir(parents=True, exist_ok=True)
    production_roster.write_text('operator-owned: preserve\n')
    production_bytes = production_roster.read_bytes()
    cfg = load_config(fleet_cfg.path, mock=True)
    store = TelemetryStore(cfg)
    isolated = tmp_path / 'isolated-mock'
    manager = fleet.FleetManager(cfg, store, run_dir=isolated)
    assert manager.roster_path == isolated / 'roster.yaml'
    replacement = dict(cfg.roster)
    replacement[cfg.drones[1]] = cfg.raw['fleet']['drones'][6]['id']
    asyncio.run(manager.save_roster(replacement, expected_hash=manager.generated.roster_hash))
    assert cfg.mock is True
    assert cfg.bt['coshow']['preflight']['required'] is True
    assert cfg.bt['bt_runner']['bt_visualiser']['enabled'] is False
    assert cfg.roster == replacement
    assert production_roster.read_bytes() == production_bytes
    reloaded = load_config(fleet_cfg.path, mock=True)
    restarted = fleet.FleetManager(reloaded, TelemetryStore(reloaded), run_dir=isolated)
    assert reloaded.roster == replacement
    assert restarted.generated is not None
    assert production_roster.read_bytes() == production_bytes


def test_roster_override_is_in_memory_and_never_reads_or_writes_operator_roster(fleet_cfg):
    replacement = dict(fleet_cfg.roster)
    replacement[fleet_cfg.drones[1]] = fleet_cfg.raw['fleet']['drones'][6]['id']
    cfg = load_config(fleet_cfg.path, mock=True, roster_override=replacement)
    assert cfg.mock and cfg.roster == replacement
    assert not fleet_cfg.resolve(fleet_cfg.raw['roster_file']).exists()


def test_mock_stack_controls_cannot_reach_real_spawn(fleet_cfg, tmp_path, monkeypatch):
    fleet = fleet_module()
    cfg = load_config(fleet_cfg.path, mock=True)
    manager = fleet.FleetManager(cfg, TelemetryStore(cfg), run_dir=tmp_path / 'mock')
    attempts = []

    async def forbidden(*args, **kwargs):
        attempts.append(args)
        raise AssertionError('mock reached process spawn')

    monkeypatch.setattr('dashboard.runner.spawn_process', forbidden)
    for operation in (manager.start_stack, manager.restart_stack, manager.stop_stack):
        with pytest.raises(ValueError, match='MOCK'):
            asyncio.run(operation())
    assert not attempts


# M8 review regressions: configuration failures must be actionable before spawn.
def test_uri_less_spare_is_excluded_with_warning_but_remains_in_inventory(fleet_cfg):
    fleet_cfg.raw['fleet']['drones'][-1]['uri'] = None
    generated = fleet_module().generate_config(fleet_cfg)
    robots = yaml.safe_load(generated.files['crazyflies.generated.yaml'])['robots']
    assert 'spare_CF10' not in robots
    assert len(robots) == 9 and len(fleet_cfg.robots) == 14
    assert sum(generated.radio_counts.values()) == 9
    assert any('CF10' in warning and '제외' in warning for warning in generated.warnings)


def test_role_type_comes_from_template_and_spare_type_from_config(fleet_cfg):
    path = fleet_cfg.resolve(fleet_cfg.raw['crazyflies_template'])
    template = yaml.safe_load(path.read_text())
    template['robot_types']['custom_role'] = copy.deepcopy(template['robot_types']['cf21'])
    template['robot_types']['custom_spare'] = copy.deepcopy(template['robot_types']['cf21'])
    template['robots'][fleet_cfg.drones[0]]['type'] = 'custom_role'
    template['robots'][fleet_cfg.drones[1]].pop('type')
    fleet_cfg.raw['fleet']['robot_type'] = 'custom_spare'
    path.write_text(yaml.safe_dump(template))
    generated = fleet_module().generate_config(fleet_cfg)
    robots = yaml.safe_load(generated.files['crazyflies.generated.yaml'])['robots']
    assert robots[fleet_cfg.drones[0]]['type'] == 'custom_role'
    assert robots[fleet_cfg.drones[1]]['type'] == 'custom_spare'
    assert all(row['type'] == 'custom_spare' for name, row in robots.items() if name.startswith('spare_'))


def test_selected_robot_type_must_exist_in_template_types(fleet_cfg):
    fleet_cfg.raw['fleet']['robot_type'] = 'absent'
    with pytest.raises(ValueError, match='absent'):
        fleet_module().generate_config(fleet_cfg)


def test_aideck_template_is_explicit_required_configuration(fleet_cfg):
    del fleet_cfg.raw['aideck_template']
    with pytest.raises(ValueError, match='aideck_template'):
        fleet_module().generate_config(fleet_cfg)


def test_save_rejects_missing_expected_hash_before_any_write(fleet_cfg):
    manager = fleet_module().FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
    before = manager.roster_path.read_bytes()
    with pytest.raises(ValueError, match='expected_hash'):
        asyncio.run(manager.save_roster(dict(fleet_cfg.roster)))
    assert manager.roster_path.read_bytes() == before


def test_fleet_operation_runner_guard_precedes_busy_and_disk_write(fleet_cfg):
    store = TelemetryStore(fleet_cfg)
    observed = []
    def guard():
        observed.append(store.stack.get('busy'))
        raise ValueError('live preflight child')
    manager = fleet_module().FleetManager(fleet_cfg, store, operation_guard=guard)
    before = manager.roster_path.read_bytes()
    with pytest.raises(ValueError, match='live preflight'):
        asyncio.run(manager.save_roster(fleet_cfg.roster, expected_hash=manager.generated.roster_hash))
    assert len(observed) == 1 and not observed[0]
    assert manager.roster_path.read_bytes() == before


def test_bundled_aideck_fqn_detects_actual_namespace_as_external_and_healthy(fleet_cfg):
    cfg = load_config(mock=True)
    assert cfg.raw['nodes']['aideck'] == '/aideck/aideck_aruco_node'
    store = TelemetryStore(fleet_cfg)
    manager = fleet_module().FleetManager(fleet_cfg, store)
    store.nodes([('aideck_aruco_node', '/aideck')])
    manager.refresh()
    assert store.stack['aideck'] == 'external'
    row = next(row for row in evaluate(store.snapshot(), fleet_cfg, store.context, store.clock()) if row['id'] == 'global.aideck')
    assert row['ok']
    with pytest.raises(ValueError):
        asyncio.run(manager.start_stack())
    assert not manager.children


def test_reconfiguration_failure_rolls_back_roster_config_and_generated_files(fleet_cfg):
    store = TelemetryStore(fleet_cfg)
    original_roster = dict(fleet_cfg.roster)
    async def fail(cfg):
        assert cfg.roster != original_roster
        raise RuntimeError('new ROS adapter startup failed')
    manager = fleet_module().FleetManager(fleet_cfg, store, on_reconfigure=fail)
    before = {path: path.read_bytes() for path in manager.run_dir.iterdir() if path.is_file()}
    old_cfg = copy.deepcopy(fleet_cfg.__dict__)
    store.receive(fleet_cfg.drones[0], 'pose', [1, 2, 3])
    old_data = copy.deepcopy(store.data)
    original_hash = manager.generated.roster_hash
    replacement = dict(original_roster)
    replacement[fleet_cfg.drones[0]] = fleet_cfg.raw['fleet']['drones'][6]['id']
    with pytest.raises(RuntimeError, match='adapter startup'):
        asyncio.run(manager.save_roster(replacement, expected_hash=original_hash))
    assert fleet_cfg.__dict__ == old_cfg
    assert store.data == old_data
    assert manager.generated.roster_hash == original_hash
    assert all(path.read_bytes() == value for path, value in before.items())
    assert not store.stack['busy']


def test_stack_recovery_restores_owned_identity_and_applied_hash_without_signals(fleet_cfg, tmp_path):
    import json
    from dashboard.runner import stop_process
    fleet = fleet_module()
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    async def exercise():
        first = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        second = None
        try:
            await first.start_stack()
            receipt = json.loads((first.run_dir / 'stack.applied.json').read_text())
            assert receipt['applied_hash'] == first.generated.roster_hash
            second = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
            await second.recover_stacks()
            assert {key: child.pid for key, child in second.children.items()} == {
                key: child.pid for key, child in first.children.items()}
            assert second.store.stack['applied_hash'] == first.generated.roster_hash
            assert second.store.stack['aideck'] == second.store.stack['crazyflie_server'] == 'up'
            row = next(row for row in evaluate(second.store.snapshot(), fleet_cfg, second.store.context, second.store.clock()) if row['id'] == 'global.orphans')
            assert row['ok']
            with pytest.raises(ValueError):
                await second.start_stack()
            assert all(not child.sigint_sent for child in second.children.values())
            # Missing hash receipt must not prevent safe ownership restoration.
            (first.run_dir / 'stack.applied.json').unlink()
            third = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
            await third.recover_stacks()
            assert len(third.children) == 2
            assert third.store.stack['applied_hash'] is None
            await third.detach_monitors()
        finally:
            if second:
                await second.detach_monitors()
            for child in first.children.values():
                await stop_process(child)
    asyncio.run(exercise())


@pytest.mark.parametrize('receipt_problem', ['missing', 'identity', 'argv'])
def test_live_stack_pid_without_matching_identity_blocks_spawn_even_without_graph(fleet_cfg, tmp_path, receipt_problem):
    import json
    from dashboard.runner import stop_process
    fleet = fleet_module()
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    async def exercise():
        first = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        recovered = None
        try:
            await first.start_stack()
            receipt_path = first.run_dir / 'aideck.process.json'
            if receipt_problem == 'missing':
                receipt_path.unlink()
            else:
                value = json.loads(receipt_path.read_text())
                value[receipt_problem] = 'other-start' if receipt_problem == 'identity' else ['other-program']
                receipt_path.write_text(json.dumps(value))
            recovered = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
            await recovered.recover_stacks()
            assert recovered.store.stack['aideck'] == 'external'
            assert 'aideck' not in recovered.children
            assert recovered.store.stack['applied_hash'] is None
            row = next(row for row in evaluate(recovered.store.snapshot(), fleet_cfg, recovered.store.context, recovered.store.clock()) if row['id'] == 'global.orphans')
            assert not row['ok'] and row['blocking']
            before = (first.run_dir / 'aideck.pid').read_bytes()
            with pytest.raises(ValueError):
                await recovered.start_stack()
            assert (first.run_dir / 'aideck.pid').read_bytes() == before
            assert first.children['aideck'].returncode is None
        finally:
            if recovered:
                await recovered.detach_monitors()
            for child in first.children.values():
                await stop_process(child)
    asyncio.run(exercise())


def test_dead_stack_pidfile_is_removed_during_startup(fleet_cfg):
    manager = fleet_module().FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
    path = manager.run_dir / 'aideck.pid'
    path.write_text('2147483647\n')
    asyncio.run(manager.recover_stacks())
    assert not path.exists()
    assert manager.store.stack['aideck'] == 'down'


def test_bundled_template_bootstraps_all_four_roles_and_excludes_six_null_spares(tmp_path):
    original = load_config(roster_override={})
    raw = copy.deepcopy(original.raw)
    raw['roster_file'] = str(tmp_path / 'fresh-roster.yaml')
    path = tmp_path / 'dashboard.yaml'
    path.write_text(yaml.safe_dump(raw))
    cfg = load_config(path, original.field_path)
    assert cfg.roster == {'cf230': 'CF01', 'cf231': 'CF02', 'cf232': 'CF03', 'cf233': 'CF04'}
    generated = fleet_module().generate_config(cfg)
    robots = yaml.safe_load(generated.files['crazyflies.generated.yaml'])['robots']
    assert set(robots) == set(cfg.drones)
    assert len(generated.warnings) == 6
    assert all('제외' in row for row in generated.warnings)


def test_stack_recovery_handles_ros2_style_shebang_executables(fleet_cfg, tmp_path):
    from dashboard.runner import stop_process
    fleet = fleet_module()
    script = write_dummy(tmp_path)
    executable = tmp_path / 'console-entrypoint'
    executable.write_text('#!' + sys.executable + '\n' + script.read_text())
    executable.chmod(0o755)
    fleet_cfg.raw['commands'].update(crazyflie_server=shlex.quote(str(executable)),
                                    aideck=shlex.quote(str(executable)))
    async def exercise():
        first = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        recovered = None
        try:
            await first.start_stack()
            recovered = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
            await recovered.recover_stacks()
            assert len(recovered.children) == 2
            assert recovered.store.stack['applied_hash'] == first.generated.roster_hash
        finally:
            if recovered:
                await recovered.detach_monitors()
            for child in first.children.values():
                await stop_process(child)
    asyncio.run(exercise())


@pytest.mark.parametrize('empty', ['', '   '])
def test_empty_spare_uri_is_a_warning_and_excluded(fleet_cfg, empty):
    fleet_cfg.raw['fleet']['drones'][-1]['uri'] = empty
    generated = fleet_module().generate_config(fleet_cfg)
    assert 'spare_CF10' not in yaml.safe_load(generated.files['crazyflies.generated.yaml'])['robots']
    assert any('CF10' in warning and '제외' in warning for warning in generated.warnings)


def test_missing_fallback_type_has_no_code_literal_default(fleet_cfg):
    fleet_cfg.raw['fleet'].pop('robot_type')
    fleet_cfg.raw.pop('robot_type', None)
    with pytest.raises(ValueError, match='robot_type'):
        fleet_module().generate_config(fleet_cfg)


def test_v26_root_robot_type_alias_is_supported(fleet_cfg):
    fleet_cfg.raw['fleet'].pop('robot_type')
    fleet_cfg.raw['robot_type'] = 'cf21'
    assert fleet_module().generate_config(fleet_cfg).files


async def cleanup_dummy_children(children):
    """Harness-owned children only: explicit cleanup independent of backend.close."""
    from dashboard.runner import stop_process
    await asyncio.gather(*(stop_process(child) for child in children.values()))


def test_backend_close_detaches_stacks_without_signals_and_preserves_receipts(fleet_cfg, tmp_path):
    import json
    fleet = fleet_module()
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    async def exercise():
        manager = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        children = {}
        recovered = None
        try:
            await manager.start_stack()
            children = dict(manager.children)
            files = {path: path.read_bytes() for path in manager.run_dir.iterdir() if path.suffix in ('.pid', '.json')}
            receipt = json.loads((manager.run_dir / 'stack.applied.json').read_text())
            await manager.close()
            assert not manager.children
            assert all(child.returncode is None and not child.sigint_sent for child in children.values())
            assert all(path.read_bytes() == content for path, content in files.items())
            assert all(receipt['processes'][kind]['argv'] == child.argv for kind, child in children.items())
            assert any('스택은 계속 실행 중' in row['text'] for row in manager.store.events)
            recovered = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
            await recovered.recover_stacks()
            assert len(recovered.children) == 2
            assert recovered.store.stack['applied_hash'] == manager.generated.roster_hash
        finally:
            if recovered:
                await recovered.detach_monitors()
            await cleanup_dummy_children(children)
    asyncio.run(exercise())


def test_partial_spawn_failure_preserves_first_stack_and_unknown_applied_hash(fleet_cfg, tmp_path):
    fleet = fleet_module()
    fleet_cfg.raw['commands']['crazyflie_server'] = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands']['aideck'] = '/definitely-missing-aideck-executable'
    async def exercise():
        manager = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        children = {}
        try:
            with pytest.raises(OSError):
                await manager.start_stack()
            children = dict(manager.children)
            assert set(children) == {'crazyflie_server'}
            child = children['crazyflie_server']
            assert child.returncode is None and not child.sigint_sent
            assert manager.store.stack['crazyflie_server'] == 'up'
            assert manager.store.stack['aideck'] == 'down'
            assert manager.store.stack['applied_hash'] is None
            assert child.pid_path.exists()
            assert any('스택 기동 실패' in row['text'] and 'missing-aideck' in row['text'] for row in manager.store.events)
            with pytest.raises(ValueError):
                await manager.start_stack()
        finally:
            await cleanup_dummy_children(children)
            await manager.close()
    asyncio.run(exercise())


def test_cancelled_start_preserves_spawned_stack_without_signals(fleet_cfg, tmp_path, monkeypatch):
    from dashboard import runner
    fleet = fleet_module()
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    original = runner.spawn_process
    async def exercise():
        reached_second = asyncio.Event()
        async def spawn(*args, **kwargs):
            if str(args[4]).endswith('aideck.pid'):
                reached_second.set()
                raise asyncio.CancelledError()
            return await original(*args, **kwargs)
        monkeypatch.setattr(runner, 'spawn_process', spawn)
        manager = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        children = {}
        try:
            with pytest.raises(asyncio.CancelledError):
                await manager.start_stack()
            assert reached_second.is_set()
            children = dict(manager.children)
            assert len(children) == 1
            assert all(child.returncode is None and not child.sigint_sent for child in children.values())
            assert manager.store.stack['applied_hash'] is None
        finally:
            await cleanup_dummy_children(children)
            await manager.close()
    asyncio.run(exercise())


def test_explicit_stop_stack_uses_parallel_owned_stop_and_is_idle_only(fleet_cfg, tmp_path, monkeypatch):
    from dashboard import runner
    fleet = fleet_module()
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    original = runner.stop_process
    async def exercise():
        entered = []
        both = asyncio.Event()
        async def stop(child, *args, **kwargs):
            entered.append(child.pid)
            if len(entered) == 2:
                both.set()
            await asyncio.wait_for(both.wait(), 1)
            return await original(child, *args, **kwargs)
        monkeypatch.setattr(runner, 'stop_process', stop)
        manager = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        children = {}
        try:
            await manager.start_stack()
            children = dict(manager.children)
            manager.store.set_run(state='RUNNING')
            with pytest.raises(ValueError, match='IDLE'):
                await manager.stop_stack()
            assert not entered
            manager.store.set_run(state='IDLE')
            await manager.stop_stack()
            assert len(entered) == 2
            assert all(child.sigint_sent and child.returncode is not None for child in children.values())
            assert not manager.children and manager.store.stack['applied_hash'] is None
            assert not list(manager.run_dir.glob('*.pid'))
        finally:
            monkeypatch.setattr(runner, 'stop_process', original)
            await cleanup_dummy_children(children)
            await manager.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('cancellations', [1, 2])
def test_caller_cancellation_waits_for_spawn_identity_recording(fleet_cfg, tmp_path, monkeypatch, cancellations):
    from dashboard import runner
    fleet = fleet_module()
    command = shlex.quote(sys.executable) + ' ' + shlex.quote(str(write_dummy(tmp_path)))
    fleet_cfg.raw['commands'].update(crazyflie_server=command, aideck=command)
    original = runner.spawn_process
    async def exercise():
        spawned = asyncio.Event()
        release = asyncio.Event()
        harness_children = {}
        async def slow_spawn(*args, **kwargs):
            child = await original(*args, **kwargs)
            harness_children[child.pid] = child
            spawned.set()
            await release.wait()
            return child
        monkeypatch.setattr(runner, 'spawn_process', slow_spawn)
        manager = fleet.FleetManager(fleet_cfg, TelemetryStore(fleet_cfg))
        children = {}
        task = asyncio.create_task(manager.start_stack())
        try:
            await asyncio.wait_for(spawned.wait(), 2)
            for _ in range(cancellations):
                task.cancel()
                await asyncio.sleep(.02)
            assert not task.done(), 'cancel must finish recording ownership of the already spawned process'
            release.set()
            with pytest.raises(asyncio.CancelledError):
                await task
            children = dict(manager.children)
            assert set(children) == {'crazyflie_server'}
            child = children['crazyflie_server']
            assert child.returncode is None and not child.sigint_sent
            assert child.pid_path.exists()
            assert (manager.run_dir / 'crazyflie_server.process.json').exists()
            assert manager.store.stack['applied_hash'] is None
        finally:
            release.set()
            await asyncio.gather(task, return_exceptions=True)
            await cleanup_dummy_children(harness_children)
            await manager.close()
    asyncio.run(exercise())
