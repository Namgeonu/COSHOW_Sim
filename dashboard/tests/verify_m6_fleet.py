#!/usr/bin/env python3
"""M6 Docker evidence: generated files, real owned processes and ROS node graph.

All devices are dummy ROS nodes. No radio, camera socket, Webots or robot service
is used. ROS_DOMAIN_ID=92 and ROS_LOCALHOST_ONLY=1 isolate the graph.
"""
import argparse
import asyncio
import copy
import difflib
import json
import os
from pathlib import Path
import resource
import shlex
import signal
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import yaml

from dashboard.config import load_config
from dashboard.checklist import evaluate
from dashboard.fleet import FleetManager
from dashboard.state import TelemetryStore
from dashboard.tests.verify_m2_ros import prepare_fixture


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'dashboard/REPORTS/evidence'
SCRIPT = Path(__file__).resolve()


def dummy_node(name, ignore_int):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    namespace, _, basename = ('/' + name.strip('/')).rpartition('/')
    node = Node(basename, namespace=namespace or '/')
    stopped = [False]

    def sigint(signum, frame):
        print('SIGNAL SIGINT', flush=True)
        if not ignore_int:
            stopped[0] = True

    signal.signal(signal.SIGINT, sigint)
    print(json.dumps(dict(event='READY', pid=os.getpid(), pgid=os.getpgrp(),
                          core=resource.getrlimit(resource.RLIMIT_CORE),
                          marker=os.environ.get('M6_TEST_ENV'), path=bool(os.environ.get('PATH')),
                          cwd=os.getcwd(), node=name)), flush=True)
    try:
        while not stopped[0]:
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command(node, ignore=False):
    argv = [sys.executable, str(SCRIPT), '--child', node]
    if ignore:
        argv.append('--ignore-int')
    return shlex.join(argv)


async def verify():
    import rclpy
    from rclpy.node import Node
    assert os.environ.get('ROS_DOMAIN_ID') in ('88', '92')
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    directory = ROOT / 'dashboard/run/m6_fleet_fixture'
    cfg = prepare_fixture(directory, port=8096)
    original_cf = ROOT / 'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml'
    original_camera = ROOT / 'ros2_ws/src/aideck_aruco_ros/config/drones.yaml'
    original_bt = ROOT / 'bt/scenarios/coshow/configs/coshow_rehearsal.yaml'
    protected = {path: path.read_bytes() for path in (original_cf, original_camera, original_bt)}
    raw = copy.deepcopy(cfg.raw)
    raw['crazyflies_template'] = str(original_cf)
    raw['aideck_template'] = str(original_camera)
    for index, physical in enumerate(raw['fleet']['drones']):
        physical['aideck_ip'] = '127.0.0.{}'.format(index + 11)
    raw['commands']['crazyflie_server'] = command(raw['nodes']['server'])
    raw['commands']['aideck'] = command(raw['nodes']['aideck'], True)
    raw['commands']['env']['M6_TEST_ENV'] = 'merged'
    cfg.path.write_text(yaml.safe_dump(raw, sort_keys=False))
    cfg = load_config(cfg.path)
    assert not cfg.errors, cfg.errors
    store = TelemetryStore(cfg)
    manager = FleetManager(cfg, store)
    manager.log_dir = directory / 'logs'
    manager.log_dir.mkdir(parents=True, exist_ok=True)
    for name in ('crazyflie_server', 'aideck'):
        (manager.log_dir / (name + '.log')).write_text('')
    old_hash = store.stack['roster_hash']
    old_files = dict(manager.generated.files)
    rclpy.init(args=[])
    observer = Node('m6_fleet_observer')

    async def graph_until(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0)
            store.nodes(observer.get_node_names_and_namespaces())
            manager.refresh()
            if predicate():
                return
            await asyncio.sleep(.05)
        raise AssertionError('ROS graph condition timed out: ' + str(store.context['nodes']))

    async def ready(child):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            lines = child.log_path.read_text().splitlines()
            for line in reversed(lines):
                if line.startswith('{'):
                    report = json.loads(line)
                    if report.get('pid') == child.pid:
                        assert report['core'] == [0, 0] and report['pgid'] == child.pid
                        assert report['marker'] == 'merged' and report['path'] is True
                        return report
            await asyncio.sleep(.05)
        raise AssertionError('dummy stack did not become ready')

    external = None
    harness_children = {}
    try:
        await manager.start_stack()
        old_children = dict(manager.children)
        harness_children.update({child.pid: child for child in old_children.values()})
        reports = [await ready(child) for child in old_children.values()]
        await graph_until(lambda: all(store.stack[kind] == 'up' for kind in old_children))
        assert store.stack['applied_hash'] == old_hash
        print('PASS owned startup: 2 new sessions, merged ROS/PATH environment, RLIMIT_CORE=(0,0), logs/pids', flush=True)
        # New manager instance reads durable pid+argv+identity receipts just as a
        # restarted backend does; dummy ROS processes continue without signals.
        recovered = FleetManager(cfg, store)
        recovered.log_dir = manager.log_dir
        await recovered.recover_stacks()
        assert {kind: child.pid for kind, child in recovered.children.items()} == {
            kind: child.pid for kind, child in old_children.items()}
        assert store.stack['applied_hash'] == old_hash
        assert all(not child.sigint_sent for child in recovered.children.values())
        await manager.detach_monitors()
        manager = recovered
        recovered_children = dict(manager.children)
        print('PASS restart ownership recovery: two verified identities, applied hash restored, zero startup signals', flush=True)
        replacement = dict(cfg.roster)
        replacement[cfg.drones[1]] = raw['fleet']['drones'][6]['id']
        await manager.save_roster(replacement, expected_hash=old_hash)
        new_hash = store.stack['roster_hash']
        assert new_hash and new_hash != old_hash
        assert store.stack['applied_hash'] == old_hash
        assert len(cfg.robots) == len(store.data) == 14
        assert load_config(cfg.path).roster == replacement
        with (EVIDENCE / 'M8_fleet_generated_diff.log').open('w') as log:
            for name in sorted(old_files):
                before, after = old_files[name].decode(), manager.generated.files[name].decode()
                log.write('FILE ' + name + '\n')
                delta = ''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                                    fromfile='before/' + name, tofile='after/' + name))
                log.write(delta or 'UNCHANGED: role names/base coordinates retained\n')
        print('PASS role swap: persisted, 14 metadata rows refreshed, file hash mismatch blocks old stack', flush=True)
        # Old camera process ignores SIGINT; replacement cooperates for final cleanup.
        cfg.raw['commands']['aideck'] = command(cfg.raw['nodes']['aideck'])
        restarted_at = time.monotonic()
        await manager.restart_stack()
        restart_duration = time.monotonic() - restarted_at
        assert restart_duration >= 10.0
        assert old_children['aideck'].returncode == -signal.SIGKILL
        assert all(child.sigint_sent for child in recovered_children.values())
        for child in old_children.values():
            assert child.log_path.read_text().count('SIGNAL SIGINT') == 1
        assert all(child.pid != old_children[kind].pid for kind, child in manager.children.items())
        assert store.stack['applied_hash'] == new_hash
        for child in manager.children.values():
            harness_children[child.pid] = child
            await ready(child)
        print('PASS restart: one SIGINT each, ignoring child SIGKILL after {:.3f}s, new PIDs and new applied hash'.format(
            restart_duration), flush=True)
        detached = dict(manager.children)
        receipt_bytes = {path: path.read_bytes() for path in manager.run_dir.iterdir()
                         if path.suffix in ('.pid', '.json')}
        await manager.close()
        assert not manager.children
        assert all(child.returncode is None and not child.sigint_sent for child in detached.values())
        assert all(path.read_bytes() == data for path, data in receipt_bytes.items())
        print('PASS backend close: two stack children alive, zero signals, PID/identity/applied receipts unchanged', flush=True)
        store = TelemetryStore(cfg)
        manager = FleetManager(cfg, store)
        manager.log_dir = directory / 'logs'
        await manager.recover_stacks()
        assert {kind: child.pid for kind, child in manager.children.items()} == {
            kind: child.pid for kind, child in detached.items()}
        assert store.stack['applied_hash'] == new_hash
        await manager.stop_stack()
        assert not manager.children and store.stack['applied_hash'] is None
        assert not list(manager.run_dir.glob('*.pid'))
        assert all(child.returncode is not None for child in detached.values())
        print('PASS explicit stop after re-adoption: owned children stopped, PID files removed, applied hash null', flush=True)

        # A failed camera spawn must preserve the already connected radio server.
        camera_command = cfg.raw['commands']['aideck']
        cfg.raw['commands']['aideck'] = '/definitely-missing-m8-aideck'
        try:
            await manager.start_stack()
        except OSError:
            pass
        else:
            raise AssertionError('missing camera executable unexpectedly spawned')
        partial = manager.children['crazyflie_server']
        harness_children[partial.pid] = partial
        await ready(partial)
        assert partial.returncode is None and not partial.sigint_sent
        assert store.stack['crazyflie_server'] == 'up' and store.stack['aideck'] == 'down'
        assert store.stack['applied_hash'] is None
        assert partial.pid_path.exists()
        print('PASS partial camera spawn failure: owned server remains alive/up, applied hash null, no automatic signal', flush=True)
        await manager.stop_stack()
        cfg.raw['commands']['aideck'] = camera_command

        # Independent harness-owned child must never become FleetManager-owned.
        external = await asyncio.create_subprocess_exec(
            sys.executable, str(SCRIPT), '--child', cfg.raw['nodes']['aideck'],
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
            start_new_session=True)
        external_store = TelemetryStore(cfg)
        external_manager = FleetManager(cfg, external_store)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0)
            external_store.nodes(observer.get_node_names_and_namespaces())
            external_manager.refresh()
            if external_store.stack['aideck'] == 'external':
                break
            await asyncio.sleep(.05)
        assert external_store.stack['aideck'] == 'external'
        row = next(row for row in evaluate(external_store.snapshot(), cfg, external_store.context,
                                          external_store.clock()) if row['id'] == 'global.aideck')
        assert row['ok'], row
        try:
            await external_manager.restart_stack()
        except ValueError as exc:
            assert '외부' in str(exc)
        else:
            raise AssertionError('external stack restart was not rejected')
        await external_manager.close()
        assert external.returncode is None
        assert all(path.read_bytes() == content for path, content in protected.items())
        print('PASS /aideck/aideck_aruco_node external detection + global.aideck PASS, restart rejected, process remains alive', flush=True)
        print(json.dumps(dict(old_hash=old_hash, new_hash=new_hash, restart_s=round(restart_duration, 3),
                              old_pids={key: child.pid for key, child in old_children.items()},
                              processes=reports, inventory_rows=len(cfg.robots),
                              generated_files=sorted(old_files), protected_templates_unchanged=True), indent=2), flush=True)
        print('PASS M6 dummy Docker verification; no hardware or camera/radio connection', flush=True)
    finally:
        from dashboard.runner import stop_process
        harness_children.update({child.pid: child for child in manager.children.values()})
        await asyncio.gather(*(stop_process(child) for child in harness_children.values()), return_exceptions=True)
        await manager.close()
        await manager.detach_monitors()
        if external is not None and external.returncode is None:
            external.send_signal(signal.SIGINT)
            try:
                await asyncio.wait_for(external.wait(), 5)
            except asyncio.TimeoutError:
                external.kill()
                await external.wait()
        observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--child')
    parser.add_argument('--ignore-int', action='store_true')
    arguments = parser.parse_args()
    if arguments.child:
        dummy_node(arguments.child, arguments.ignore_int)
    else:
        asyncio.run(verify())
