"""Final checks in the disposable non-root install container, after recording."""
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import time

project = Path('/home/coshow/COSHOW')
assert os.getuid() != 0
for relative in ('bt/scenarios/coshow/configs/coshow_rehearsal.yaml',
                 'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml',
                 'ros2_ws/src/aideck_aruco_ros/config/drones.yaml'):
    original, copied = Path('/source') / relative, project / relative
    assert original.read_bytes() == copied.read_bytes(), relative
    print('UNCHANGED', relative, hashlib.sha256(copied.read_bytes()).hexdigest())
for name in ('build', 'install'):
    tree = project / 'ros2_ws' / name
    assert all(path.lstat().st_uid == os.getuid() for path in tree.rglob('*')), 'root-owned build/install file'
print('PASS build/install ownership belongs to non-root installer user')
assert not Path('/home/coshow/.cache/coshow-kiosk').exists(), 'dry-run created browser profiles'
print('PASS kiosk dry-run created no browser profiles')
rows = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True).splitlines()
matches = []
for row in rows:
    pid, command = row.strip().split(None, 1)
    if command == 'python3 -u server.py --config config/dashboard.yaml --mock':
        matches.append(int(pid))
assert len(matches) == 1, matches
pid = matches[0]
print('RUN_SH_PID', pid, 'is the Python server directly (exec preserved)')
first = (project / 'dashboard/logs/dashboard.log').read_text().splitlines()[0]
assert 'ROS_DOMAIN_ID=89' in first and 'bind=127.0.0.1:8080' in first and 'mock=True' in first
print('FIRST_LOG_LINE', first)
os.kill(pid, signal.SIGINT)
deadline = time.monotonic() + 5
while Path('/proc', str(pid)).exists() and time.monotonic() < deadline:
    time.sleep(.05)
assert not Path('/proc', str(pid)).exists(), 'run.sh server did not stop after SIGINT'
print('PASS run.sh direct Python PID handled SIGINT and exited')
