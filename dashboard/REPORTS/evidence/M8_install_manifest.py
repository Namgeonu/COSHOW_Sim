"""Verify the installer/test-kit follow-up evidence and record exact provenance."""
import gzip
import hashlib
import json
from pathlib import Path

dashboard = Path(__file__).resolve().parents[2]
evidence = dashboard / 'REPORTS/evidence'
commands = {
    'M8_install_prepare.log': 'docker exec coshow-m8-install-isolated bash /source/dashboard/REPORTS/evidence/M8_install_prepare.sh',
    'M8_install_clean.log': 'docker exec -u coshow coshow-m8-install-isolated env -i HOME=/home/coshow USER=coshow PATH=/usr/local/bin:/usr/bin:/bin DISPLAY=:99 XDG_SESSION_TYPE=x11 ROS_DOMAIN_ID=92 ROS_LOCALHOST_ONLY=1 bash --noprofile --norc /source/dashboard/REPORTS/evidence/M8_install_validate.sh clean',
    'M8_install_merged.log': 'docker exec -u coshow coshow-m8-install-merged env -i HOME=/home/coshow USER=coshow PATH=/usr/local/bin:/usr/bin:/bin DISPLAY=:99 XDG_SESSION_TYPE=x11 ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 bash --noprofile --norc /source/dashboard/REPORTS/evidence/M8_install_validate.sh merged',
    'M8_install_kiosk_no_x.log': 'docker exec -u coshow coshow-m8-install-isolated env -i HOME=/home/coshow USER=coshow PATH=/usr/local/bin:/usr/bin:/bin XDG_SESSION_TYPE=wayland DISPLAY= bash /source/dashboard/kiosk.sh --dry-run',
    'M8_install_wrong_clone.log': 'docker exec -u coshow coshow-m8-install-isolated env -i HOME=/home/coshow USER=coshow PATH=/usr/local/bin:/usr/bin:/bin bash /source/dashboard/install.sh',
    'M8_install_kit_green.log': 'docker run --rm --name coshow-m8-kit-tests --entrypoint /bin/bash -v <repository>:/ws -w /ws coshow-humble-dev -lc "source /opt/ros/humble/setup.bash && python3 -m pytest dashboard/tests/test_stage_tools.py -q"',
    'M8_install_archive.log': 'python3 dashboard/REPORTS/evidence/M8_install_archive.py',
}
print('BASE_IMAGE ros:humble sha256:1813d3c85d7f96ff7d3012d865204583255740182db5d0065f8f8cd029a83138')
print('PREPARED_BEFORE_ANY_ROS_BUILD coshow-m8-install-pristine:20260913 sha256:ab07a31b650dc6cdc54bc553c194fb2c0d271caf2b0fb4570580eb315b960bbc')
for name, command in commands.items():
    data = (evidence / name).read_bytes()
    print('COMMAND', name, command)
    print('LOG_BYTES', len(data), 'SHA256', hashlib.sha256(data).hexdigest())
installer_hash = hashlib.sha256((dashboard / 'install.sh').read_bytes()).hexdigest()
for name in ('M8_install_clean.log', 'M8_install_merged.log'):
    output = (evidence / name).read_text()
    assert installer_hash in output
    assert 'PASS initial interfaces and dashboard/OpenCV dependencies absent' in output
    assert 'OpenCV 4.5.4 NumPy 1.21.5 ArUco API: detectMarkers synthetic marker=1; JPEG encode/decode PASS' in output
    assert 'PASS latest installer, exact source YAML/setup_env preservation, no foreign ROS overlay' in output
assert 'Building missing interfaces: crazyflie_interfaces coshow_interfaces' in (evidence / 'M8_install_clean.log').read_text()
assert 'FINAL_ENV AMENT_PREFIX_PATH /home/coshow/COSHOW/ros2_ws/install:/opt/ros/humble' in (evidence / 'M8_install_merged.log').read_text()
assert 'FAIL clone path' in (evidence / 'M8_install_wrong_clone.log').read_text()
assert '--window-position=1920' in (evidence / 'M8_install_kiosk_no_x.log').read_text()
assert '20 passed' in (evidence / 'M8_install_kit_green.log').read_text()
assert '4 failed' in (evidence / 'M8_install_kit_red.log').read_text()
print('PASS latest installer SHA256', installer_hash)
print('PASS D1/D4 RED4 -> complete kit GREEN20; wrong-clone real invocation expected exit2; no-X dry-run exit0')
data = gzip.decompress((evidence / 'M7_stage_full.jsonl.gz').read_bytes())
assert hashlib.sha256(data).hexdigest() == 'f35d640d0e845f9a8da2dc1523c3f7e99161293f86f50ed3fd0e2dc2218d1091'
rows = [json.loads(line) for line in data.splitlines()]
states = [row for row in rows if row['message']['type'] == 'state']
phases = []
for row in states:
    phase = (row['message'].get('mission') or {}).get('phase')
    if phase and (not phases or phase != phases[-1]):
        phases.append(phase)
assert len(rows) == 1060 and len(states) == 1059
assert phases == ['observe', 'handover', 'search', 'capture', 'rescue_dispatch', 'rescue', 'return', 'done']
print('PASS tracked full gzip: 1060 messages,1059 states,8 phases; original recording hash unchanged')
poses = gzip.decompress((evidence / 'M7_poses_full.csv.gz').read_bytes())
assert hashlib.sha256(poses).hexdigest() == '5c1a6428571ebf828538e63f8518351dfa7d77ba7e9c9255e167951117c4b364'
assert len(poses.splitlines()) == 14827  # Header + 14,826 pose rows.
stdout = (evidence / 'M7_stage_probe.log').read_bytes().split(b'\n', 1)[1]
assert hashlib.sha256(stdout).hexdigest() == 'e30cc0f3bc46ee28e9f8ff1b531d5b6c2a37abf4a83c82c3e69197e8c5d45539'
print('PASS tracked CSV gzip:14826 pose rows; M7_stage_probe.log minus command header preserves original stdout SHA256')
print('LIMITS: disposable Chromium140 + Xvfb3840x2160; no physical monitor/GPU byte measurement, Webots, radio, AI Deck socket or robot flight')
