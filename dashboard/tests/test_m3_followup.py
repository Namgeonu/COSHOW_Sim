"""M2 review regressions, including real ROS and a running HTTP application."""
import asyncio
import copy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from dashboard.config import load_config
from dashboard.checklist import evaluate, external_observation
from dashboard.mock import MockWorld, scenario
from dashboard.ros_io import interface_specs, check_config
from dashboard.server import Dashboard, SocketSlots
from dashboard.state import TelemetryStore


def write_config(tmp_path, edit):
    source = load_config(mock=True)
    raw = copy.deepcopy(source.raw)
    raw['commands']['bt_cwd'] = str(tmp_path)
    raw['commands']['bt'] = 'python3 main.py --config bt.yaml'
    raw['roster_file'] = str(tmp_path / 'roster.yaml')
    (tmp_path / 'roster.yaml').write_text(yaml.safe_dump(source.roster))
    (tmp_path / 'bt.yaml').write_text(yaml.safe_dump(source.bt))
    (tmp_path / 'field.yaml').write_text(yaml.safe_dump(source.field))
    edit(raw)
    path = tmp_path / 'dashboard.yaml'
    path.write_text(yaml.safe_dump(raw))
    return load_config(path)


@pytest.mark.parametrize('kind,key,value', [
    ('topics', 'limo_status_template', None),
    ('topics', 'new_unsupported_topic', '/test'),
    ('topics', 'pose_template', '/{drone}/pose'),
    ('topics', 'pose_template', '/{cf.x}/pose'),
])
def test_bad_interface_is_reported_without_losing_valid_specs(kind, key, value):
    cfg = load_config(mock=True)
    cfg.raw[kind][key] = value
    specs, errors = interface_specs(cfg)
    assert any(e['kind'] == kind and e['key'] == key and e['reason'] for e in errors)
    assert not any(s['kind'] == kind and s['key'] == key for s in specs)
    assert any(s['channel'] == 'mission' for s in specs)


@pytest.mark.parametrize('kind', ['topics', 'services', 'actions'])
def test_type_key_typo_is_an_interface_configuration_error(kind):
    cfg = load_config(mock=True)
    cfg.raw['types'][kind]['misspelled_key'] = 'std_msgs/msg/String'
    specs, errors = interface_specs(cfg)
    assert specs
    assert any(e['kind'] == 'types.' + kind and e['key'] == 'misspelled_key' for e in errors)


def test_null_types_are_skipped_even_without_ros_graph():
    cfg = load_config(mock=True)
    rows = check_config(cfg, timeout=0)
    skipped = [r for r in rows if r.get('status') == 'skip']
    assert len(skipped) == 4
    assert all(r['expected_type'] is None and r['ok'] is None for r in skipped)
    assert all(r['key'] == 'limo_status_template' for r in skipped)


def test_inventory_warnings_are_separate_from_fatal_role_errors(tmp_path):
    def edit(raw):
        raw['fleet']['drones'][-1].update(uri=None, aideck_ip=None)
        raw['fleet']['limos'][-1]['ip'] = None
        raw['fleet']['drones'][0].update(uri=None, aideck_ip=None)
        raw['network']['aideck_ips'] = {}
        raw['network']['limo_ips'] = {}
    cfg = write_config(tmp_path, edit)
    role_id = cfg.raw['fleet']['drones'][0]['id']
    spare_id = cfg.raw['fleet']['drones'][-1]['id']
    assert any(role_id in e and 'URI' in e for e in cfg.errors)
    assert any(role_id in e and 'IP' in e for e in cfg.errors)
    assert any(spare_id in e and 'URI' in e for e in cfg.warnings)
    assert not any(spare_id in e for e in cfg.errors)
    state = TelemetryStore(cfg)
    rows = evaluate(state.snapshot(), cfg, state.checklist_context(), state.clock())
    assert any(role_id in r['detail'] and 'URI' in r['detail'] and r['blocking'] for r in rows)
    assert any(spare_id in r['detail'] and 'URI' in r['detail'] and not r['blocking'] and r['status'] == 'warning' for r in rows)
    assert not any(r['id'] == 'global.bt_config' for r in rows)


def test_missing_spare_prefix_is_fatal_configuration(tmp_path):
    cfg = write_config(tmp_path, lambda raw: raw.pop('spare_prefix'))
    assert any('spare_prefix' in e for e in cfg.errors)


def test_visualiser_is_advisory_but_display_gate_stays_blocking():
    cfg = load_config(mock=True)
    cfg.bt['bt_runner']['bt_visualiser']['enabled'] = True
    store = TelemetryStore(cfg)
    context = store.checklist_context()
    context['env'] = {}
    rows = {r['id']: r for r in evaluate(store.snapshot(), cfg, context, store.clock())}
    visual = rows['global.bt_visualiser']
    assert not visual['ok'] and not visual['blocking'] and visual['status'] == 'warning'
    assert rows['global.bt_environment']['blocking'] and not rows['global.bt_environment']['ok']


def test_mock_receives_only_configured_ros_channels():
    cfg = load_config(mock=True)
    cfg.raw['topics'].pop('camera_fps_template')
    store = TelemetryStore(cfg)
    world = MockWorld(cfg, store)
    world.apply(scenario(32, cfg), store.clock())
    for name, meta in cfg.robots.items():
        if meta['kind'] == 'drone' and not meta['role']:
            assert 'pose' not in store.data[name]
        if meta['kind'] == 'limo' and not meta['role']:
            assert 'nav_ready' not in store.data[name]
        assert 'camera_fps' not in store.data[name]
    assert 'pose' in store.data[cfg.drones[0]]
    assert 'frame' in store.data[cfg.drones[0]]


def test_elapsed_and_ready_only_use_monotonic_receipt_clock():
    clock = [100.0]
    store = TelemetryStore(load_config(mock=True), clock=lambda: clock[0])
    store.ready(True)
    clock[0] = 104.5
    snap = store.snapshot()
    assert snap['run']['elapsed_s'] == 4.5
    assert snap['preflight'] == {'ready': True, 'age': 4.5}
    assert snap['t'] > 1000000
    assert 'elapsed_s' not in store.run
    store.set_run(state='LANDING')
    store.set_stack(aideck='up')
    clock[0] = 107.0
    assert store.snapshot()['run']['elapsed_s'] == 2.5
    assert store.snapshot()['stack']['aideck'] == 'up'


def test_ready_stream_is_not_masked_by_one_retained_status():
    cfg = load_config(mock=True)
    context = {'receipts': {'preflight_status': [99], 'preflight_ready': [98.5, 99.5]}}
    observed = external_observation('preflight', cfg, context, 100)
    assert observed['active'] and observed['external']
    context['receipts']['preflight_ready'] = [99.5]
    assert not external_observation('preflight', cfg, context, 100)['active']


def test_unexpected_socket_exception_warns_closes_and_drops_sender():
    async def exercise():
        class Socket:
            closed = False
            async def send_str(self, value):
                raise ValueError('unexpected encoder failure')
            async def close(self):
                self.closed = True
        socket, warnings = Socket(), []
        slots = SocketSlots(socket, 1, on_error=warnings.append)
        slots.offer('{}', {})
        await asyncio.wait_for(slots.run(), 1)
        assert socket.closed
        assert len(warnings) == 1 and 'unexpected encoder failure' in warnings[0]
    asyncio.run(exercise())


def test_aggregate_recovers_next_tick_and_warns_only_once():
    async def exercise():
        app = Dashboard(load_config(mock=True), mock=True)
        calls, values = [], []
        def tick():
            calls.append(1)
            if len(calls) <= 2:
                raise ValueError('temporary sample failure')
        app.world.tick = tick
        class Slots:
            def offer(self, state, frames):
                values.append(json.loads(state))
        app.sockets.add(Slots())
        task = asyncio.create_task(app.aggregate())
        try:
            await asyncio.sleep(.36)
            assert not task.done() and len(values) >= 1
            assert len([e for e in app.store.events if 'temporary sample failure' in e['text']]) == 1
        finally:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
    asyncio.run(exercise())


@pytest.mark.parametrize('fail_start', [False, True])
def test_partial_bad_interface_and_ros_failure_keep_http_alive(monkeypatch, fail_start):
    from aiohttp.test_utils import TestClient, TestServer
    import dashboard.ros_io as ros_io
    async def exercise():
        cfg = load_config(mock=True)
        cfg.raw['topics']['limo_status_template'] = None
        cfg.raw['types']['topics']['typo_type_key'] = 'std_msgs/msg/String'
        if fail_start:
            def fail(prefix):
                raise ValueError('simulated ROS initialization failure')
            monkeypatch.setattr(ros_io, '_new_node', fail)
        app = Dashboard(cfg)
        async with TestClient(TestServer(app.app())) as client:
            response = await client.get('/')
            assert response.status == 200
            state = app.state()
            assert any(r['label'] == '인터페이스 설정 오류' and r['blocking'] and not r['ok'] for r in state['checklist'])
            assert any('limo_status_template' in e['text'] for e in state['events'])
            if fail_start:
                assert any('simulated ROS initialization failure' in e['text'] for e in state['events'])
            else:
                assert app.io.thread.is_alive() and app.io.subscriptions
    asyncio.run(exercise())


def test_literal_audit_catches_each_reviewed_external_name():
    import ast
    from dashboard.tests.verify_m2_constraints import assert_external_names_absent
    literals = ['cf230', '/preflight/ready', '/coshow/mission_state', '/aideck/data',
                '/renamed/land', '/renamed/arm', '/renamed/navigate_to_pose',
                'marker_detections', 'limo_new', 'spare_', 'spare_CF07']
    for literal in literals:
        with pytest.raises(AssertionError):
            assert_external_names_absent(ast.parse('value = ' + repr(literal)), 'fixture.py')
    assert_external_names_absent(ast.parse("value = 'spare_prefix'"), 'fixture.py')


def test_sender_failure_is_removed_from_dashboard_socket_set():
    from aiohttp.test_utils import TestClient, TestServer
    async def exercise():
        app = Dashboard(load_config(mock=True), mock=True)
        async with TestClient(TestServer(app.app())) as client:
            async with client.ws_connect('/ws') as socket:
                assert (await socket.receive_json())['type'] == 'hello'
                for _ in range(100):
                    if app.sockets:
                        break
                    await asyncio.sleep(.005)
                assert app.sockets, 'sender was not registered after hello'
                slots = next(iter(app.sockets))
                async def fail(value):
                    raise ValueError('live socket writer failure')
                slots.ws.send_str = fail
                await asyncio.sleep(.25)
                assert slots not in app.sockets
                assert any('live socket writer failure' in event['text'] for event in app.store.events)
    asyncio.run(exercise())


def test_explicitly_malformed_spare_uri_is_fatal_not_unfilled_inventory(tmp_path):
    def edit(raw):
        raw['fleet']['drones'][-1]['uri'] = 'not-a-radio-uri'
    cfg = write_config(tmp_path, edit)
    physical = cfg.raw['fleet']['drones'][-1]['id']
    assert any(physical in error and 'URI' in error for error in cfg.errors)
    assert not any(physical in warning and 'URI' in warning for warning in cfg.warnings)


@pytest.mark.parametrize('kind,key', [('topics', 'pose_template'),
                                     ('services', 'land_template'),
                                     ('actions', 'nav_template')])
def test_missing_type_entry_is_blocking_configuration_not_explicit_null_skip(kind, key):
    cfg = load_config(mock=True)
    cfg.raw['types'][kind].pop(key)
    specs, errors = interface_specs(cfg)
    assert any(e['kind'] == 'types.' + kind and e['key'] == key for e in errors)
    assert not any(s['kind'] == kind and s['key'] == key for s in specs)
    store = TelemetryStore(cfg)
    MockWorld(cfg, store)
    rows = evaluate(store.snapshot(), cfg, store.checklist_context(), store.clock())
    assert any(r['label'] == '인터페이스 설정 오류' and r['blocking'] and key in r['detail'] for r in rows)
    assert any(key in e['text'] for e in store.events)
