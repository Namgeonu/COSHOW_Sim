#!/usr/bin/env python3
"""Run M2 graph/CLI and two-role WebSocket evidence against the shared fixture.

All generated inventory, roster, and BT flag overrides live in ignored run/.
--prepare-only works without ROS and can prepare the same local --mock run.
"""
import argparse
import asyncio
import copy
import json
import os
from pathlib import Path
import resource
import shlex
import signal
import subprocess
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import aiohttp
import yaml
from dashboard.config import load_config
from dashboard.ros_io import interface_specs
from dashboard.tests.probe_m2 import probe


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'dashboard/REPORTS/evidence'


def prepare_fixture(directory=None, port=8092):
    """Fill an explicitly fake local inventory without touching operator files."""
    directory = Path(directory or ROOT / 'dashboard/run/m2_fixture').resolve()
    directory.mkdir(parents=True, exist_ok=True)
    source = load_config(mock=True)
    raw, bt = copy.deepcopy(source.raw), copy.deepcopy(source.bt)
    bt['coshow']['preflight']['required'] = True
    bt['bt_runner']['bt_visualiser']['enabled'] = False
    raw['http'] = {'host': '127.0.0.1', 'port': port}
    raw['roster_file'] = str(directory / 'roster.yaml')
    raw['crazyflies_template'] = ''
    raw['commands']['bt_cwd'] = str(ROOT / 'bt')
    raw['commands']['bt'] = 'python3 main.py --config ' + shlex.quote(str(directory / 'bt.yaml'))
    raw['commands']['env'] = {'SDL_VIDEODRIVER': 'dummy', 'SDL_AUDIODRIVER': 'dummy'}
    raw['types']['topics']['limo_status_template'] = raw['types']['topics']['status_template']
    for i, drone in enumerate(raw['fleet']['drones']):
        drone['uri'] = 'radio://{}/80/2M/{:010X}'.format(i // 4, 0xE7E7E7E700 + i)
        drone['aideck_ip'] = '127.0.0.1'
    for limo in raw['fleet']['limos']:
        limo['ip'] = '127.0.0.1'
    raw['network']['aideck_ips'] = {name: '127.0.0.1' for name in raw['robots']['drones']}
    raw['network']['limo_ips'] = {name: '127.0.0.1' for name in raw['robots']['limos']}
    raw['network']['ping_period_s'] = 0.5
    roster = {role: physical['id'] for role, physical in zip(raw['robots']['drones'], raw['fleet']['drones'])}
    for filename, data in (('dashboard.yaml', raw), ('field.yaml', source.field),
                           ('bt.yaml', bt), ('roster.yaml', roster)):
        (directory / filename).write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
    cfg = load_config(directory / 'dashboard.yaml')
    assert not cfg.errors, cfg.errors
    assert len(cfg.robots) == 14
    assert sorted(cfg.radio_counts.values()) == [2, 4, 4], cfg.radio_counts
    print('FIXTURE CONFIG: {} (14 robots, 3 radios, role roster, loopback IPs)'.format(cfg.path), flush=True)
    return cfg


def run_cli(command, destination, expected):
    print('$ ' + ' '.join(command), flush=True)
    result = subprocess.run(command, cwd=str(ROOT), capture_output=True, text=True, timeout=25)
    destination.write_text('$ ' + ' '.join(command) + '\n' + result.stdout + result.stderr
                           + '\nEXIT CODE: {}\n'.format(result.returncode), encoding='utf-8')
    assert result.returncode == expected, destination.read_text()
    print('{}: rc={}, PASS rows={}, FAIL rows={}'.format(
        destination.name, result.returncode, result.stdout.count('PASS\t'), result.stdout.count('FAIL\t')), flush=True)
    return result


def inspect_ros(cfg):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from rosidl_runtime_py.utilities import get_message

    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('dashboard_m2_qos_probe', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        specs, errors = interface_specs(cfg)
        checked = []
        for spec in specs:
            if spec['kind'] != 'topics' or spec['channel'] not in ('frame', 'detections', 'mission', 'preflight', 'ready'):
                continue
            deadline = time.monotonic() + 4
            info = []
            while not info and time.monotonic() < deadline:
                info = node.get_publishers_info_by_topic(spec['name'])
                time.sleep(.02)
            assert len(info) == 1, (spec, info)
            qos = info[0].qos_profile
            if spec['channel'] == 'frame':
                assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
            else:
                assert qos.reliability == ReliabilityPolicy.RELIABLE
                expected = DurabilityPolicy.VOLATILE if spec['channel'] == 'detections' else DurabilityPolicy.TRANSIENT_LOCAL
                assert qos.durability == expected
            checked.append(spec['name'])
        print('QOS PASS: {} configured JPEG/detection/JSON endpoints'.format(len(checked)), flush=True)
        samples = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for spec in specs:
            if spec['kind'] == 'topics' and spec['channel'] in ('mission', 'preflight', 'ready'):
                node.create_subscription(get_message(spec['type']), spec['name'],
                                         lambda msg, key=spec['channel']: samples.setdefault(key, msg), qos)
        deadline = time.monotonic() + 3
        while len(samples) < 3 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert set(samples) == {'mission', 'preflight', 'ready'}, samples
        assert samples['ready'].data is True
        print('LATE SUBSCRIBER PASS: mission, preflight and ready received', flush=True)
        services = {name for name, _ in node.get_service_names_and_types()}
        for name, robot in cfg.robots.items():
            if robot['kind'] == 'drone' and robot['role'] is None:
                for template in cfg.raw['services'].values():
                    assert template.format(cf=name, limo=name, name=name) not in services
        print('SPARE CONTROL PASS: no spare Land/Arm service servers', flush=True)
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


async def wait_http(url, child, timeout=8):
    async with aiohttp.ClientSession() as session:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert child.poll() is None, 'Dashboard exited before HTTP was ready'
            try:
                async with session.get(url) as response:
                    if response.status == 200:
                        return
            except aiohttp.ClientError:
                pass
            await asyncio.sleep(.1)
        raise AssertionError('HTTP server did not become ready')


def stop(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=5)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=3)
    return child.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--directory')
    parser.add_argument('--port', type=int, default=8092)
    parser.add_argument('--seconds', type=float, default=5.0)
    args = parser.parse_args()
    cfg = prepare_fixture(args.directory, args.port)
    if args.prepare_only:
        return
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use an isolated local Docker ROS domain'
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    fixture_log_path = EVIDENCE / 'M2_ros_fixture.log'
    server_log_path = EVIDENCE / 'M2_ros_server.log'
    children = []
    with fixture_log_path.open('w') as fixture_log, server_log_path.open('w') as server_log:
        try:
            fixture = subprocess.Popen(
                [sys.executable, 'dashboard/tests/fake_ros_world.py', '--config', str(cfg.path)],
                cwd=str(ROOT), stdout=fixture_log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(fixture)
            deadline = time.monotonic() + 8
            while 'FIXTURE READY:' not in fixture_log_path.read_text() and time.monotonic() < deadline:
                assert fixture.poll() is None, fixture_log_path.read_text()
                time.sleep(.05)
            assert 'FIXTURE READY:' in fixture_log_path.read_text(), fixture_log_path.read_text()
            command = [sys.executable, 'dashboard/server.py', '--config', str(cfg.path),
                       '--check-config', '--check-timeout', '4']
            present = run_cli(command, EVIDENCE / 'M2_check_config_present.log', 0)
            assert 'FAIL\t' not in present.stdout
            wrong = copy.deepcopy(cfg.raw)
            wrong['topics']['mission_state'] += '_intentional_typo'
            typo_path = cfg.path.with_name('dashboard_typo.yaml')
            typo_path.write_text(yaml.safe_dump(wrong, allow_unicode=True, sort_keys=False))
            bad_command = list(command)
            bad_command[bad_command.index('--config') + 1] = str(typo_path)
            absent = run_cli(bad_command, EVIDENCE / 'M2_check_config_missing.log', 1)
            failures = [line for line in absent.stdout.splitlines() if line.startswith('FAIL\t')]
            assert len(failures) == 1 and wrong['topics']['mission_state'] in failures[0], failures
            inspect_ros(cfg)
            server = subprocess.Popen(
                [sys.executable, 'dashboard/server.py', '--config', str(cfg.path)], cwd=str(ROOT),
                stdout=server_log, stderr=subprocess.STDOUT, start_new_session=True)
            children.append(server)
            url = 'http://127.0.0.1:' + str(args.port)

            async def exercise():
                await wait_http(url, server)
                results = await probe(url, args.seconds, expect_external=True)
                (EVIDENCE / 'M2_ros_websocket.log').write_text(
                    '\n'.join(json.dumps(row, ensure_ascii=False) for row in results) + '\nPASS\n', encoding='utf-8')

            asyncio.run(exercise())
        finally:
            for child in reversed(children):
                print('CHILD STOP: pid={} rc={}'.format(child.pid, stop(child)), flush=True)
    contents = fixture_log_path.read_text()
    assert 'FIXTURE CLOSED: command_requests=0' in contents, contents
    print('PASS: M2 ROS graph, check-config good/typo, two sockets, fixture command_requests=0', flush=True)


if __name__ == '__main__':
    main()
