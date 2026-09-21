#!/usr/bin/env bash
# Run as the desktop user; only apt-get receives elevated privileges.
set -eo pipefail
DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$DASHBOARD_DIR/.." && pwd)"
WORKSPACE_DIR="$PROJECT_DIR/ros2_ws"
if [[ $EUID -eq 0 ]]; then
  printf 'FAIL user: run bash dashboard/install.sh as the desktop user, without sudo.\n' >&2
  exit 2
fi
EXPECTED_PROJECT_DIR="$(realpath -m "$HOME/COSHOW")"
if [[ "$(realpath "$PROJECT_DIR")" != "$EXPECTED_PROJECT_DIR" ]]; then
  printf 'FAIL clone path: setup_env.sh requires ~/COSHOW (%s); current clone is %s. Use the existing clone at ~/COSHOW.\n' "$EXPECTED_PROJECT_DIR" "$PROJECT_DIR" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash || ! -f "$PROJECT_DIR/setup_env.sh" ]]; then
  printf 'FAIL environment: existing ROS Humble and project setup_env.sh are required.\n' >&2
  exit 2
fi
missing_apt=()
# NumPy/OpenCV are already aideck_aruco_ros execution dependencies. Colcon's
# Python build does not install package.xml exec_depend packages itself.
for package in python3-aiohttp python3-yaml iputils-ping python3-numpy python3-opencv; do
  if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then
    missing_apt+=("$package")
  fi
done
if ((${#missing_apt[@]})); then
  sudo apt-get update
  sudo apt-get install -y "${missing_apt[@]}"
fi
source /opt/ros/humble/setup.bash
# A clean source checkout needs its first interface build before the existing
# setup_env.sh can source the workspace. Existing installations are sourced first.
if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
  source "$PROJECT_DIR/setup_env.sh"
  source "$WORKSPACE_DIR/install/setup.bash"
fi
layout=()
if [[ -f "$WORKSPACE_DIR/install/.colcon_install_layout" ]]; then
  case "$(cat "$WORKSPACE_DIR/install/.colcon_install_layout")" in
    merged) layout=(--merge-install) ;;
    isolated) ;;
    *) printf 'FAIL colcon: unknown existing install layout.\n' >&2; exit 2 ;;
  esac
fi
missing_interfaces=()
if ! python3 -c 'from crazyflie_interfaces.srv import Land; Land.Request()' >/dev/null 2>&1; then
  missing_interfaces+=(crazyflie_interfaces)
fi
if ! python3 -c 'from coshow_interfaces.msg import MarkerDetections; MarkerDetections()' >/dev/null 2>&1; then
  missing_interfaces+=(coshow_interfaces)
fi
cd "$WORKSPACE_DIR"
if ((${#missing_interfaces[@]})); then
  printf 'Building missing interfaces: %s\n' "${missing_interfaces[*]}"
  colcon build "${layout[@]}" --packages-select "${missing_interfaces[@]}" --event-handlers console_direct+
  source "$WORKSPACE_DIR/install/setup.bash"
else
  printf 'PASS interfaces already importable; interface build skipped.\n'
fi
# Always refresh ament_python's installed .py, launch, and config copies.
colcon build "${layout[@]}" --packages-select aideck_aruco_ros --event-handlers console_direct+
source "$PROJECT_DIR/setup_env.sh"
source "$WORKSPACE_DIR/install/setup.bash"
mkdir -p "$DASHBOARD_DIR/logs" "$DASHBOARD_DIR/run"
failures=0
check() {
  local label="$1"
  shift
  if "$@"; then printf 'PASS %s\n' "$label"; else printf 'FAIL %s\n' "$label"; failures=$((failures + 1)); fi
}
check 'Python aiohttp/PyYAML/rclpy imports' python3 -c 'import aiohttp, yaml, rclpy'
check 'crazyflie Land interface' ros2 interface show crazyflie_interfaces/srv/Land
check 'coshow MarkerDetections interface' ros2 interface show coshow_interfaces/msg/MarkerDetections
check 'workspace package prefix, node import and OpenCV runtime' python3 - "$WORKSPACE_DIR" <<'PY'
import hashlib
import importlib
from pathlib import Path
import sys
from ament_index_python.packages import get_package_prefix

workspace = Path(sys.argv[1]).resolve()
install = workspace / 'install'
layout = (install / '.colcon_install_layout').read_text().strip()
expected = install if layout == 'merged' else install / 'aideck_aruco_ros'
actual = Path(get_package_prefix('aideck_aruco_ros')).resolve()
assert actual == expected.resolve(), 'Stale package prefix: ' + str(actual)
source = workspace / 'src/aideck_aruco_ros/aideck_aruco_ros/aideck_aruco_node.py'
# Execute the installed module imports. Finding a module alone does not check
# cv2/numpy or the ROS node's existing execution dependencies. Importing this
# module does not construct a Node or open the AI Deck UDP streams.
module = importlib.import_module('aideck_aruco_ros.aideck_aruco_node')
selected = Path(module.__file__)
assert hashlib.sha256(selected.read_bytes()).digest() == hashlib.sha256(source.read_bytes()).digest(), 'Selected installed node differs from current source'
print(actual)
print('Selected Python module:', selected)

import cv2
import numpy as np
from aideck_aruco_ros.udp_stream import create_aruco_detector, detect_markers

# Use the existing node's compatibility helper: Ubuntu's OpenCV uses the old
# detectMarkers API; newer installed OpenCV builds use ArucoDetector.
detector, dictionary, parameters = create_aruco_detector('DICT_4X4_50')
draw = getattr(cv2.aruco, 'generateImageMarker', None) or cv2.aruco.drawMarker
marker = draw(dictionary, 1, 64)
canvas = np.full((96, 96), 255, dtype=np.uint8)
canvas[16:80, 16:80] = marker
_, ids, _ = detect_markers(detector, dictionary, parameters, canvas)
assert ids is not None and ids.flatten().tolist() == [1], 'ArUco detection capability failed'
ok, jpeg = cv2.imencode('.jpg', canvas)
assert ok and jpeg.size > 0, 'OpenCV JPEG encoder unavailable'
assert cv2.imdecode(jpeg, cv2.IMREAD_GRAYSCALE).shape == canvas.shape, 'OpenCV JPEG decoder unavailable'
print('OpenCV', cv2.__version__, 'NumPy', np.__version__,
      'ArUco API:', 'ArucoDetector' if detector is not None else 'detectMarkers',
      'synthetic marker=1; JPEG encode/decode PASS')
PY
check 'ping executable' command -v ping
browser=''
for candidate in google-chrome chromium chromium-browser; do
  if command -v "$candidate" >/dev/null 2>&1; then browser="$(command -v "$candidate")"; break; fi
done
if [[ -n "$browser" ]]; then
  check 'browser binary' "$browser" --version
else
  printf 'FAIL browser: install Google Chrome or Chromium.\n'; failures=$((failures + 1))
fi
check 'X11 monitors' bash -c '[[ ${XDG_SESSION_TYPE:-} == x11 && -n ${DISPLAY:-} ]] && xrandr --listmonitors'
if ((failures)); then
  printf 'FAIL installation self-check: %d failed item(s); resolve them before starting.\n' "$failures" >&2
  exit 1
fi
printf 'PASS installation self-check; collaborator YAML files were not modified.\n'
