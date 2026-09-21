#!/bin/bash
# 컨테이너 진입: ROS 환경 + (필요 시) 인터페이스 2개 빌드 + install 소스
set -e
source /opt/ros/humble/setup.bash
if [ ! -f /ws_build/install/setup.bash ] || [ "${REBUILD:-0}" = "1" ]; then
  echo "[entry] building crazyflie_interfaces + coshow_interfaces -> /ws_build"
  colcon build --base-paths /ws/ros2_ws/src \
    --packages-select crazyflie_interfaces coshow_interfaces \
    --build-base /ws_build/build --install-base /ws_build/install \
    --event-handlers console_direct+ >/ws_build/colcon.log 2>&1 || { tail -30 /ws_build/colcon.log; exit 1; }
fi
source /ws_build/install/setup.bash
exec "$@"
