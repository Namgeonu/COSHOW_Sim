#!/usr/bin/env python3
"""A1: refused roster save must not close the adapter used by the next estop.

Only local DDS services and disposable preflight children are used. The first
child ignores SIGINT, reproducing reset -> IDLE with a still-live preflight.
After the rejected save it exits naturally; a second preflight and estop use
the production lifecycle and the same ROS adapter. No radio or deck is opened.
"""
import asyncio
import json
import os
from pathlib import Path
import shlex
import signal
import sys
import tempfile
import time
from types import SimpleNamespace
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dashboard import runner
from dashboard.server import Dashboard
from dashboard.tests.verify_m5_server_shutdown import LocalROS, fixture_config, until


def child(root):
    first = not (root / 'first.started').exists()
    (root / 'first.started').touch()
    def stop(number, frame):
        if not first:
            raise SystemExit(0)
    signal.signal(signal.SIGINT, stop)
    (root / ('first.ready' if first else 'second.ready')).touch()
    while True:
        if first and (root / 'first.exit').exists():
            os._exit(0)
        time.sleep(.02)


async def probe():
    assert os.environ.get('ROS_DOMAIN_ID') == '91'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    with tempfile.TemporaryDirectory(prefix='m8-adapter-') as directory:
        root = Path(directory)
        prefix = '/adapter_probe_' + uuid.uuid4().hex[:10]
        cfg = fixture_config(root, prefix, 'stubborn')
        cfg.raw['commands']['preflight'] = shlex.join([
            sys.executable, str(Path(__file__).resolve()), '--child', str(root)])
        runner.HERE = root / 'dashboard'
        ros = LocalROS(cfg, prefix)
        app = Dashboard(cfg)
        try:
            await app.startup(None)
            app.runner.state_callback = app.store.snapshot
            await until(lambda: all(c.service_is_ready() for c in app.io.service_clients.values()))
            assert (await app.runner.command('preflight'))[0]
            await until(lambda: app.store.run['state'] == 'READY' and (root / 'first.ready').exists())
            first_pid = app.store.run['preflight_pid']
            assert (await app.runner.command('reset'))[0]
            await until(lambda: app.store.run['state'] == 'IDLE')
            assert app.runner._alive('preflight') and app.store.run['preflight_pid'] == first_pid
            old_io = app.io
            old_roster = app.fleet.roster_path.read_bytes()
            async def body():
                return dict(roster=cfg.roster, expected_hash=app.fleet.generated.roster_hash)
            request = SimpleNamespace(remote='127.0.0.1', headers={}, scheme='http',
                host='127.0.0.1', match_info={'action': 'roster'}, json=body)
            response = await app.fleet_action(request)
            assert response.status == 409, response.text
            assert 'IDLE' in json.loads(response.body)['error']
            assert app.runner.io is app.io is old_io and old_io.node is not None
            assert app.fleet.roster_path.read_bytes() == old_roster
            assert not app.store.stack.get('busy')
            print('ROSTER_REFUSED', response.text, 'adapter_alive=True', flush=True)
            (root / 'first.exit').touch()
            await until(lambda: not app.runner._alive('preflight'))
            with ros.lock:
                ros.requests.clear()
                ros.landed_at.clear()
            assert (await app.runner.command('preflight'))[0]
            await until(lambda: app.store.run['state'] == 'READY' and (root / 'second.ready').exists())
            second_pid = app.store.run['preflight_pid']
            assert second_pid != first_pid
            assert (await app.runner.command('estop'))[0]
            await until(lambda: app.store.run['state'] == 'ABORTED')
            with ros.lock:
                requests = list(ros.requests)
            lands = [r for r in requests if r['channel'] == 'land']
            cancels = [r for r in requests if r['channel'] == 'cancel']
            assert sorted(r['robot'] for r in lands) == sorted(cfg.drones)
            assert sorted(r['robot'] for r in cancels) == sorted(cfg.limos)
            assert app.runner.io is old_io and old_io.node is not None
            print('PASS A1 DDS', json.dumps(dict(first_preflight=first_pid,
                next_preflight=second_pid, land_count=len(lands), cancel_count=len(cancels),
                same_live_adapter=True, requests=requests), ensure_ascii=False), flush=True)
        finally:
            (root / 'first.exit').touch()
            if app.runner:
                await until(lambda: not app.runner._alive('preflight') or (root / 'second.ready').exists())
            await app.shutdown(None)
            await app.cleanup(None)
            await asyncio.to_thread(ros.close)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--child':
        child(Path(sys.argv[2]))
    else:
        asyncio.run(probe())
