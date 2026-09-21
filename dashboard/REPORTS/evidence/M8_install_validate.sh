#!/usr/bin/env bash
# Run under env -i as coshow in the dedicated fresh installer fixture.
set -eo pipefail
mode="${1:?clean or merged}"
cd "$HOME/COSHOW"
cp /source/dashboard/install.sh dashboard/install.sh
printf 'MODE=%s UID=%s CLONE=%s\n' "$mode" "$(id -u)" "$PWD"
python3 --version
test ! -e ros2_ws/build
test ! -e ros2_ws/install
test -z "${AMENT_PREFIX_PATH:-}"
test -z "${COLCON_PREFIX_PATH:-}"
printf 'PASS initial process has no ROS overlay and no existing workspace build/install\n'
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import importlib.util
import os
print('UNDERLAY_ONLY AMENT_PREFIX_PATH=' + os.environ.get('AMENT_PREFIX_PATH', ''))
assert os.environ.get('AMENT_PREFIX_PATH') == '/opt/ros/humble'
for name in ('crazyflie_interfaces', 'coshow_interfaces', 'cv2', 'aiohttp'):
    assert importlib.util.find_spec(name) is None, name
print('PASS initial interfaces and dashboard/OpenCV dependencies absent')
PY
if [[ "$mode" == merged ]]; then
  (cd ros2_ws && colcon build --merge-install --packages-select crazyflie_interfaces coshow_interfaces --event-handlers console_direct+)
  printf 'PASS merged interfaces were built from this clone using only Humble underlay\n'
elif [[ "$mode" != clean ]]; then
  exit 2
fi
bash dashboard/install.sh
source setup_env.sh
python3 - <<'PY'
import hashlib
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
project = Path.home() / 'COSHOW'
for key in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'PYTHONPATH'):
    value = os.environ.get(key, '')
    print('FINAL_ENV', key, value)
    for prefix in value.split(':'):
        if prefix:
            assert prefix.startswith((str(project), '/opt/ros/humble')), (key, prefix)
for name in ('aideck_aruco_ros', 'crazyflie_interfaces', 'coshow_interfaces'):
    value = get_package_prefix(name)
    assert value.startswith(str(project / 'ros2_ws/install')), (name, value)
    print('SELECTED_PREFIX', name, value)
for relative in ('setup_env.sh', 'bt/scenarios/coshow/configs/coshow_rehearsal.yaml',
                 'ros2_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml',
                 'ros2_ws/src/aideck_aruco_ros/config/drones.yaml', 'dashboard/install.sh'):
    expected, actual = Path('/source') / relative, project / relative
    assert expected.read_bytes() == actual.read_bytes(), relative
    print('SOURCE_BYTES_IDENTICAL', relative, hashlib.sha256(actual.read_bytes()).hexdigest())
for name in ('build', 'install'):
    root = project / 'ros2_ws' / name
    assert root.stat().st_uid == os.getuid()
    assert all(path.lstat().st_uid == os.getuid() for path in root.rglob('*')), name
print('PASS all build/install files owned by the non-root installer user')
print('PASS latest installer, exact source YAML/setup_env preservation, no foreign ROS overlay')
PY
