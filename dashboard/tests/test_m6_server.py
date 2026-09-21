"""Fleet HTTP integration, receipt reset and control interlocks; no robot I/O."""
import asyncio
import json
from types import SimpleNamespace
import pytest
from aiohttp import web
from dashboard.config import load_config
from dashboard.fleet import FleetManager
from dashboard.server import Dashboard


def request(action, payload=None, remote='127.0.0.1'):
    async def body(): return payload
    return SimpleNamespace(remote=remote, headers={}, scheme='http', host='127.0.0.1',
                           match_info={'action': action}, json=body)


def make_dashboard(tmp_path):
    app = Dashboard(load_config(mock=True), mock=True)
    app.fleet = FleetManager(app.cfg, app.store, app.reconfigure, run_dir=tmp_path)
    app.mock_applied_hash = app.fleet.generated.roster_hash
    app.world.tick()
    return app


def test_roster_http_reconnects_and_replacement_cannot_inherit_telemetry(tmp_path):
    async def run():
        app = make_dashboard(tmp_path)
        old_hash = app.mock_applied_hash
        role = app.cfg.drones[0]
        old_physical = app.cfg.roster[role]
        replacement = dict(app.cfg.roster)
        replacement[role] = next(item['id'] for item in app.cfg.raw['fleet']['drones']
                                 if item['id'] not in replacement.values())
        app.store.receive(role, 'frame', b'old physical frame')
        closed = []
        class Socket:
            async def close(self): closed.append(True)
        app.sockets.add(SimpleNamespaceHashable(ws=Socket()))
        before = app.fleet.roster_path.read_bytes()
        response = await app.fleet_action(request('roster', {'roster': replacement}))
        assert response.status == 409 and 'expected_hash' in json.loads(response.body)['error']
        assert app.fleet.roster_path.read_bytes() == before and closed == []
        response = await app.fleet_action(request('roster', {'roster': replacement, 'expected_hash': old_hash}))
        assert response.status == 200
        assert closed == [True]
        assert not app.store.latest_frames()
        assert all(not value for value in app.store.data.values())
        assert app.cfg.robots[role]['fleet_id'] == replacement[role] != old_physical
        assert app.fleet.generated.roster_hash != old_hash == app.mock_applied_hash
        assert len(app.cfg.hello()) == 16
        assert len(app.cfg.hello()['drones']) == 4 and len(app.cfg.robots) == 14
        assert any(row['blocking'] and not row['ok'] and row['id'] == 'global.roster'
                   for row in app.state()['checklist'])
        app.sockets.clear()
        response = await app.fleet_action(request('restart'))
        assert response.status == 200
        assert app.mock_applied_hash == app.fleet.generated.roster_hash
        assert not app.fleet.children
        await app.cleanup(None)
    asyncio.run(run())


class SimpleNamespaceHashable(SimpleNamespace):
    __hash__ = object.__hash__


def test_running_and_nonlocal_fleet_writes_are_rejected_and_stack_blocks_start(tmp_path):
    async def run():
        app = make_dashboard(tmp_path)
        before = app.fleet.roster_path.read_bytes()
        for action in ('roster', 'start', 'restart', 'stop'):
            with pytest.raises(web.HTTPForbidden):
                await app.fleet_action(request(action, {'roster': app.cfg.roster}, remote='10.0.0.2'))
        app.store.set_run(state='RUNNING')
        for action in ('roster', 'start', 'restart', 'stop'):
            response = await app.fleet_action(request(action, {'roster': app.cfg.roster,
                'expected_hash': app.fleet.generated.roster_hash}))
            assert response.status == 409
            assert 'IDLE' in json.loads(response.body)['error']
        assert app.fleet.roster_path.read_bytes() == before
        app.store.set_run(state='IDLE')
        app.store.set_stack(busy=True)
        for command in ('preflight', 'start'):
            await app.command({'cmd': command}, True)
            assert app.store.run['state'] == 'IDLE'
            assert app.store.events[-1]['text'].startswith('cmd:' + command + ' rejected')
        await app.cleanup(None)
    asyncio.run(run())


def test_mock_stop_persists_through_aggregation_and_can_start_again(tmp_path):
    async def run():
        app = make_dashboard(tmp_path)
        response = await app.fleet_action(request('stop'))
        assert response.status == 200
        aggregate = asyncio.create_task(app.aggregate())
        try:
            await asyncio.sleep(.12)
            stack = app.store.stack
            assert stack['crazyflie_server'] == stack['aideck'] == 'down'
            assert stack['applied_hash'] is None
            assert (await app.fleet_action(request('start'))).status == 200
            await asyncio.sleep(.12)
            assert stack['crazyflie_server'] == stack['aideck'] == 'up'
            assert stack['applied_hash'] == app.fleet.generated.roster_hash
        finally:
            aggregate.cancel()
            await asyncio.gather(aggregate, return_exceptions=True)
            await app.cleanup(None)
    asyncio.run(run())
