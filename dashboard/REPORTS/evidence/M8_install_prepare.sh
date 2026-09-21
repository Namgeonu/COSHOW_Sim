#!/usr/bin/env bash
# Development fixture only: fresh ros:humble, /source read-only, no ROS overlay.
set -eo pipefail
apt-get update
apt-get install -y --no-install-recommends sudo xvfb x11-xserver-utils python3-pip
python3 -m pip install playwright==1.55.0
PLAYWRIGHT_BROWSERS_PATH=/opt/playwright python3 -m playwright install --with-deps chromium
useradd -m -s /bin/bash coshow
printf 'coshow ALL=(root) NOPASSWD: /usr/bin/apt-get\n' > /etc/sudoers.d/coshow-m8
chmod 440 /etc/sudoers.d/coshow-m8
python3 - <<'PY'
from pathlib import Path
import shutil
source, project = Path('/source'), Path('/home/coshow/COSHOW')
for name in ('dashboard', 'bt', 'ros2_ws/src/coshow_interfaces',
             'ros2_ws/src/aideck_aruco_ros', 'ros2_ws/src/crazyswarm2/crazyflie_interfaces',
             'ros2_ws/src/crazyswarm2/crazyflie'):
    shutil.copytree(source / name, project / name,
                   ignore=shutil.ignore_patterns('__pycache__', 'logs', 'run', 'REPORTS', '.venv'))
shutil.copy2(source / 'setup_env.sh', project / 'setup_env.sh')
assert not (project / 'ros2_ws/install').exists()
assert not (project / 'ros2_ws/build').exists()
print('CLEAN SOURCE: no workspace build/install; setup_env.sh copied unchanged')
PY
chown -R coshow:coshow /home/coshow/COSHOW
ln -s /opt/playwright/chromium-1187/chrome-linux/chrome /usr/local/bin/chromium
python3 - <<'PY'
import importlib.util
for name in ('cv2', 'aiohttp', 'crazyflie_interfaces', 'coshow_interfaces'):
    print('BEFORE INSTALL', name, importlib.util.find_spec(name))
    assert importlib.util.find_spec(name) is None, name
PY
printf 'Prepared non-root source fixture and genuine Chromium; no dashboard/AI Deck dependencies or ROS overlays installed.\n'
