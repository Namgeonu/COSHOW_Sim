#!/usr/bin/env bash
# Disposable test-container provisioning only; not the production installer.
# Start ros:humble with this repository mounted read-only at /source. Before
# this script, the actual preparation command was:
# apt-get update && apt-get install -y --no-install-recommends sudo xvfb x11-xserver-utils python3-pip && python3 -m pip install playwright==1.55.0 && PLAYWRIGHT_BROWSERS_PATH=/opt/playwright python3 -m playwright install --with-deps chromium
set -e
useradd -m -s /bin/bash coshow
printf 'coshow ALL=(root) NOPASSWD: /usr/bin/apt-get\n' > /etc/sudoers.d/coshow-m7
chmod 440 /etc/sudoers.d/coshow-m7
python3 - <<'PY'
from pathlib import Path
import shutil
source = Path('/source')
target = Path('/home/coshow/COSHOW')
for name in ('dashboard', 'bt', 'ros2_ws/src/coshow_interfaces',
             'ros2_ws/src/aideck_aruco_ros', 'ros2_ws/src/crazyswarm2/crazyflie_interfaces',
             'ros2_ws/src/crazyswarm2/crazyflie'):
    shutil.copytree(source / name, target / name,
                    ignore=shutil.ignore_patterns('__pycache__', 'logs', 'run', 'REPORTS'))
shutil.copy2(source / 'setup_env.sh', target / 'setup_env.sh')
PY
chown -R coshow:coshow /home/coshow/COSHOW
ln -s /opt/playwright/chromium-1187/chrome-linux/chrome /usr/local/bin/chromium
Xvfb :99 -screen 0 3840x2160x24 -ac >/tmp/m7-xvfb.log 2>&1 &
printf 'Prepared genuine ROS source packages, non-root account, Chromium binary and Xvfb.\n'
