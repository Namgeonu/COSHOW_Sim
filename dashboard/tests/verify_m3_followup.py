#!/usr/bin/env python3
"""M3 first-commit evidence: real CLI policy and degraded HTTP/WS startup."""
import asyncio
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import yaml
from dashboard.tests.verify_m2_ros import prepare_fixture, run_cli, wait_http, stop
from dashboard.tests.probe_m2 import probe

ROOT = Path(__file__).resolve().parents[2]
EVIDENCE = ROOT / 'dashboard/REPORTS/evidence'


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') == '83'
    cfg = prepare_fixture(ROOT / 'dashboard/run/m3_followup', port=8093)
    command = [sys.executable, 'dashboard/server.py', '--check-config', '--check-timeout', '3']
    fixture_path = EVIDENCE / 'M3_followup_fixture.log'
    server_path = EVIDENCE / 'M3_followup_server.log'
    children = []
    with fixture_path.open('w') as fixture_log, server_path.open('w') as server_log:
        try:
            fixture_command = [sys.executable, 'dashboard/tests/fake_ros_world.py', '--config', str(cfg.path)]
            fixture_log.write('$ ' + ' '.join(fixture_command) + '\n'); fixture_log.flush()
            fixture = subprocess.Popen(fixture_command, stdout=fixture_log, stderr=subprocess.STDOUT)
            children.append(fixture)
            deadline = time.monotonic() + 8
            while 'FIXTURE READY:' not in fixture_path.read_text() and time.monotonic() < deadline:
                assert fixture.poll() is None, fixture_path.read_text()
                time.sleep(.05)
            assert 'FIXTURE READY:' in fixture_path.read_text()

            def check(raw, name, expected):
                path = cfg.path.with_name(name + '.yaml')
                path.write_text(yaml.safe_dump(raw, allow_unicode=True))
                result = run_cli(command + ['--config', str(path)],
                                 EVIDENCE / ('M3_followup_' + name + '_green.log'), expected)
                return path, result

            raw = copy.deepcopy(cfg.raw)
            raw['types']['topics']['limo_status_template'] = None
            assigned = set(cfg.roster.values())
            for item in raw['fleet']['drones']:
                if item['id'] not in assigned:
                    item.update(uri=None, aideck_ip=None)
            for item in raw['fleet']['limos']:
                if item['namespace'] not in cfg.limos:
                    item['ip'] = None
            path, result = check(raw, 'skip_warn', 0)
            assert result.stdout.count('SKIP\t') == 4, result.stdout
            assert result.stdout.count('WARN\t') == 14, result.stdout
            assert result.stdout.count('PASS\t') == 51 and 'FAIL\t' not in result.stdout
            wrong = copy.deepcopy(raw)
            wrong['topics']['mission_state'] += '_intentional_typo'
            _, result = check(wrong, 'known_missing', 1)
            assert result.stdout.count('FAIL\t') == 1 and result.stdout.count('SKIP\t') == 4
            missing_type = copy.deepcopy(raw)
            missing_type['types']['topics'].pop('pose_template')
            _, result = check(missing_type, 'missing_type', 1)
            assert result.stdout.count('FAIL\t') == 1 and result.stdout.count('SKIP\t') == 4
            assert 'types.topics' in result.stdout and 'pose_template' in result.stdout
            role_error = copy.deepcopy(raw)
            role_error['fleet']['drones'][0].update(uri=None, aideck_ip=None)
            role_error['network']['aideck_ips'] = {}
            _, result = check(role_error, 'role_required', 1)
            assert result.stdout.count('FAIL\tconfiguration') == 2
            bad = copy.deepcopy(raw)
            bad['topics']['limo_status_template'] = None
            bad['types']['topics']['pose_templat'] = 'geometry_msgs/msg/PoseStamped'
            bad_path, result = check(bad, 'partial_interface', 1)
            assert result.stdout.count('FAIL\t') == 2
            real = run_cli(command, EVIDENCE / 'M3_followup_actual_config_green.log', 1)
            assert real.stdout.count('SKIP\t') == 4
            assert 'WARN\tconfiguration' in real.stdout and 'FAIL\tconfiguration' in real.stdout
            server_command = [sys.executable, 'dashboard/server.py', '--config', str(bad_path)]
            server_log.write('$ ' + ' '.join(server_command) + '\n'); server_log.flush()
            server = subprocess.Popen(server_command, stdout=server_log, stderr=subprocess.STDOUT)
            children.append(server)

            async def exercise():
                url = 'http://127.0.0.1:8093'
                await wait_http(url, server)
                results = await probe(url, 3, expect_external=True)
                output = EVIDENCE / 'M3_followup_degraded_websocket_green.log'
                output.write_text('$ ' + ' '.join([sys.executable] + sys.argv) + ' # embedded probe: ' + url + ', seconds=3\n'
                                  + '\n'.join(json.dumps(r, ensure_ascii=False) for r in results) + '\nPASS\n')
            asyncio.run(exercise())
        finally:
            for child in reversed(children):
                print('CHILD STOP: pid={} rc={}'.format(child.pid, stop(child)), flush=True)
    assert 'FIXTURE CLOSED: command_requests=0' in fixture_path.read_text()
    print('PASS: null SKIP4 + spare WARN14 rc0; typed missing rc1; role URI/IP rc1;', flush=True)
    print('PASS: actual config retains SKIP4 while fatal role/graph errors rc1;', flush=True)
    print('PASS: partial template/type-key errors leave HTTP and two live ROS WebSockets available; commands=0', flush=True)


if __name__ == '__main__':
    main()
