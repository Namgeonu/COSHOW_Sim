"""Backend contracts independent of ROS (also run on the Python 3.10 harness)."""
import asyncio
import copy
import json
from pathlib import Path

import yaml
import pytest

from dashboard.config import load_config
from dashboard.state import TelemetryStore
from dashboard.mock import scenario
from dashboard.server import SocketSlots


def test_mock_inventory_and_static_contract():
    cfg = load_config(mock=True)
    hello = cfg.hello(True)
    assert len(cfg.robots) == 14
    assert sum(r['kind'] == 'drone' and r['role'] is None for r in cfg.robots.values()) == 6
    assert all(r['fleet_id'] for r in cfg.robots.values())
    assert hello['type'] == 'hello' and hello['mock'] is True
    assert len(hello['drones']) == 4 and len(hello['limos']) == 2
    assert hello['field'] == cfg.field
    assert hello['bases'] == {n: v['base'] for k in ('drones', 'limos')
                              for n, v in cfg.bt['coshow'][k].items()}


def test_receipt_age_hold_and_detached_snapshot():
    cfg = load_config(mock=True)
    clock = [100.0]
    store = TelemetryStore(cfg, clock=lambda: clock[0])
    name = cfg.drones[0]
    store.receive(name, 'pose', dict(x=1, y=2, z=3, yaw=0))
    store.receive(name, 'detections', [3, 1])
    store.mission({'t': -99999, 'phase': 'observe', 'led': {name: 'blue'}, 'cmd': {}})
    clock[0] += .3
    store.receive(name, 'detections', [])
    state = store.snapshot()
    assert abs(state['robots'][name]['pose_age'] - .3) < 1e-6
    assert state['mission']['age'] < .31
    assert state['robots'][name]['detections'] == [1, 3]
    state['robots'][name]['pose']['x'] = 999
    assert store.snapshot()['robots'][name]['pose']['x'] == 1
    clock[0] += .21
    assert store.snapshot()['robots'][name]['detections'] == []
    store.reset_cached()
    reset = store.snapshot()
    assert reset['mission'] is None and reset['robots'][name]['led'] == 'off'
    assert reset['robots'][name]['pose']['x'] == 1


def test_mock_phase_led_and_ground_order():
    cfg = load_config(mock=True)
    phases = ['observe', 'handover', 'search', 'capture', 'rescue_dispatch', 'rescue', 'return', 'done']
    samples = [scenario(t, cfg) for t in (0, 22, 32, 47, 52, 60, 75, 91)]
    assert [s['mission']['phase'] for s in samples] == phases
    searchers = cfg.bt['coshow']['searchers']
    assert all(samples[1]['robots'][n]['pose']['z'] == 0 for n in searchers)
    finder = samples[3]['mission']['finder']
    assert samples[3]['mission']['led'][finder] == 'blue'
    assert samples[4]['mission']['target_confirmed'] is True
    assert samples[4]['mission']['led'][finder] == 'red'
    assert samples[5]['mission']['led'][finder] == 'green'
    assert samples[6]['mission']['led'][finder] == 'off'
    assert scenario(1000, cfg)['mission']['phase'] == 'done'


def test_mock_uses_actual_m1_json_shapes():
    cfg = load_config(mock=True)
    row = scenario(47, cfg)
    assert isinstance(row['preflight']['stages'], list)
    assert row['preflight']['stages'][0]['name'] == 'server_ready'
    assert set(row['mission']['P_N']) == {'x', 'y', 'z'}
    assert row['mission']['rescue_done_t'] == 0.0
    assert row['preflight']['t'] == 47
    for report in row['preflight']['drones'].values():
        assert len(report['pose_err']) == 3 and isinstance(report['pose_err'], list)
    for command in row['mission']['cmd'].values():
        assert 't' in command
        assert len(command['goal']) == (3 if command['kind'] == 'go_to' else 0)


def test_socket_slots_are_bounded_and_independent():
    async def exercise():
        class Socket:
            def __init__(self, blocked=False):
                self.values = []
                self.closed = False
                self.gate = asyncio.Event()
                if not blocked:
                    self.gate.set()
            async def send_str(self, value):
                await self.gate.wait()
                self.values.append(json.loads(value))
            async def send_bytes(self, value):
                await self.gate.wait()
                self.values.append(value)
            async def close(self):
                self.closed = True
        slow, fast = Socket(True), Socket()
        a, b = SocketSlots(slow, 4, timeout=.05), SocketSlots(fast, 4, timeout=.05)
        tasks = [asyncio.create_task(s.run()) for s in (a, b)]
        for i in range(100):
            for s in (a, b):
                s.offer(json.dumps({'sequence': i}), {0: bytes([0, i])})
        assert len(a.frames) <= 4
        await asyncio.sleep(.09)
        assert slow.closed
        assert {'sequence': 99} in fast.values and bytes([0, 99]) in fast.values
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
    asyncio.run(exercise())


def test_partial_config_missing_field_and_bt_are_reported(tmp_path):
    path = tmp_path / 'dashboard.yaml'
    path.write_text('robots: {drones: [], limos: []}\ncommands: {bt: python3}\n')
    cfg = load_config(path)
    assert cfg.dashboard_ok and not cfg.field_ok
    assert any('--config' in error for error in cfg.errors)
    assert cfg.hello()['bases'] == {} and cfg.hello()['observe_point'] is None


def test_config_renaming_and_count_need_no_code_changes(tmp_path):
    source = load_config(mock=True)
    original = source.drones[1]
    replacement = 'new_search_role'
    raw, bt = copy.deepcopy(source.raw), copy.deepcopy(source.bt)
    raw['robots']['drones'][1] = replacement
    raw['display']['names'][replacement] = raw['display']['names'].pop(original)
    bt['coshow']['drones'][replacement] = bt['coshow']['drones'].pop(original)
    bt['coshow']['search']['zones'][replacement] = bt['coshow']['search']['zones'].pop(original)
    bt['coshow']['searchers'][0] = replacement
    raw['robots']['drones'].append('fifth_role')
    bt['coshow']['drones']['fifth_role'] = {'base': [0, 1]}
    (tmp_path / 'bt.yaml').write_text(yaml.safe_dump(bt))
    raw['commands']['bt_cwd'] = str(tmp_path)
    raw['commands']['bt'] = 'python3 main.py --config bt.yaml'
    path = tmp_path / 'dashboard.yaml'
    path.write_text(yaml.safe_dump(raw))
    (tmp_path / 'field.yaml').write_text(yaml.safe_dump(source.field))
    cfg = load_config(path, mock=True)
    hello = cfg.hello(True)
    assert original not in cfg.robots and replacement in cfg.robots
    assert len(cfg.drones) == 5 and len(cfg.robots) == 14
    assert len(hello['lanes'][replacement]) == 2
    assert sum(m['kind'] == 'drone' and m['role'] is None for m in cfg.robots.values()) == 5
    assert len(scenario(32, cfg)['robots']) == 14


def test_admin_authorization_and_mock_commands():
    from dashboard.server import Dashboard
    from aiohttp.test_utils import make_mocked_request
    from aiohttp import web
    async def exercise():
        cfg = load_config(mock=True)
        app = Dashboard(cfg, True)
        await app.command({'cmd': 'preflight'}, False)
        assert app.store.run['state'] == 'IDLE'
        assert 'rejected' in app.store.events[-1]['text']
        await app.command({'cmd': 'preflight'}, True)
        assert app.store.run['state'] == 'CHECKING'
        app.world.checking -= 7
        app.world.tick()
        assert app.store.run['state'] == 'READY'
        await app.command({'cmd': 'start'}, True)
        assert app.store.run['state'] == 'RUNNING'
        await app.command({'cmd': 'start'}, True)
        assert 'rejected' in app.store.events[-1]['text']
        app.store.run['state'] = 'LANDING'
        await app.command({'cmd': 'reset'}, True)
        assert app.store.run['state'] == 'LANDING'
        # Rejection is before WebSocket prepare, so remote peers never become admin.
        class Transport:
            def get_extra_info(self, key, default=None):
                return ('192.0.2.99', 1234) if key == 'peername' else default
        request = make_mocked_request('GET', '/ws?role=admin', transport=Transport())
        try:
            await app.websocket(request)
        except web.HTTPForbidden:
            pass
        else:
            raise AssertionError('remote admin was accepted')
    asyncio.run(exercise())


def test_events_are_snapshots_with_bounded_history():
    store = TelemetryStore(load_config(mock=True))
    for i in range(300):
        store.event('info', str(i))
    assert len(store.events) == 200
    assert [e['text'] for e in store.snapshot()['events']] == [str(i) for i in range(270, 300)]


def test_mock_accepts_absolute_namespaced_nodes():
    from dashboard.server import Dashboard
    cfg = load_config(mock=True)
    cfg.raw['nodes'] = {key: '/renamed/' + key for key in cfg.raw['nodes']}
    app = Dashboard(cfg, True)
    app.world.command('preflight')
    app.world.checking -= 7
    app.world.tick()
    assert not [r for r in app.state()['checklist'] if r['blocking'] and not r['ok']]


@pytest.mark.parametrize('alive,expected,observed,status', [
    (True, 1, 0, 'up'), (True, 1, 1, 'up'), (True, 1, 2, 'external'),
    (False, 1, 1, 'down'), (False, 1, 2, 'external'),
    (False, 0, 0, 'down'), (False, 0, 1, 'external'),
    (True, 2, 2, 'up'), (True, 2, 3, 'external')])
def test_graph_respects_owned_stack_and_counts_duplicate_external_nodes(alive, expected, observed, status):
    cfg = load_config(mock=True)
    cfg.raw['nodes']['server'] = '/isolated/server'
    cfg.raw['nodes']['aideck'] = '/isolated/camera'
    store = TelemetryStore(cfg)
    store.context['stack_processes'] = {kind: dict(alive=alive, expected_nodes=expected, pid=123 if alive else None)
                                        for kind in ('crazyflie_server', 'aideck')}
    store.nodes([('server', '/isolated'), ('camera', '/isolated')] * observed)
    assert store.stack['crazyflie_server'] == store.stack['aideck'] == status


def test_unknown_fleet_radio_cannot_silently_pass_static_config(tmp_path):
    source = load_config(mock=True)
    raw = copy.deepcopy(source.raw)
    unconfigured = raw['fleet']['drones'][-1]
    unconfigured['uri'] = None
    raw['commands']['bt_cwd'] = str(source.bt_cwd)
    raw['roster_file'] = str(tmp_path / 'roster.yaml')
    (tmp_path / 'roster.yaml').write_text(yaml.safe_dump(source.roster))
    (tmp_path / 'field.yaml').write_text(yaml.safe_dump(source.field))
    path = tmp_path / 'dashboard.yaml'
    path.write_text(yaml.safe_dump(raw))
    cfg = load_config(path)
    assert any(unconfigured['id'] in error and 'URI' in error for error in cfg.warnings)


def test_mock_failure_keeps_start_blocked():
    from dashboard.server import Dashboard
    async def exercise():
        cfg = load_config(mock=True)
        app = Dashboard(cfg, True, True)
        await app.command({'cmd': 'preflight'}, True)
        app.world.checking -= 7
        app.world.tick()
        state = app.state()
        assert state['run']['state'] == 'CHECKING'
        assert state['preflight']['stages'][2]['result'] == 'fail'
        assert state['preflight']['drones'][cfg.drones[2]]['pose_err'][1] == .31
        assert state['robots'][cfg.drones[-1]]['ping']['ok'] is False
        await app.command({'cmd': 'start'}, True)
        assert app.store.run['state'] == 'CHECKING'
        assert 'rejected' in app.store.events[-1]['text']
    asyncio.run(exercise())
