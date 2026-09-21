#!/usr/bin/env bash
# 관람자 대시보드 실행 스크립트.
#
# ROS 환경을 스스로 준비하므로, 시뮬 PC 든 실기체 담당 노트북이든 같은 명령으로 뜬다.
#   시뮬 PC        : ~/COSHOW 레이아웃(setup_env.sh + ros2_ws)을 자동 감지해 소스한다.
#   실기 노트북     : 아래 우선순위로 ROS 를 찾는다. 자기 워크스페이스(crazyswarm2 +
#                     coshow_interfaces + nav2_msgs 가 있는 overlay)를 미리 source 해두거나,
#                     COSHOW_ROS_SETUP 에 그 setup.bash 경로를 지정하면 된다.
#
# ROS 소스 우선순위:
#   1) COSHOW_ROS_SETUP 이 가리키는 setup.bash          (실기 노트북에서 명시 지정)
#   2) ~/COSHOW 레이아웃 (setup_env.sh + ros2_ws)        (시뮬 PC 자동 감지)
#   3) 이미 source 된 환경(ROS_DISTRO 존재)이면 그대로 사용
#   4) /opt/ros/humble 만
#
# 도메인은 이 스크립트가 건드리지 않는다. BT/시뮬/리모와 같은 값으로 맞춰서 실행할 것:
#   export ROS_DOMAIN_ID=<모두 동일>   ./run.sh
set -eo pipefail
DASHBOARD_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$DASHBOARD_DIR/.." && pwd)"

if [[ -n "$COSHOW_ROS_SETUP" && -f "$COSHOW_ROS_SETUP" ]]; then
  echo "[run] ROS 소스: COSHOW_ROS_SETUP=$COSHOW_ROS_SETUP"
  # shellcheck disable=SC1090
  source "$COSHOW_ROS_SETUP"
elif [[ -f "$PROJECT_DIR/setup_env.sh" && -f "$PROJECT_DIR/ros2_ws/install/setup.bash" ]]; then
  echo "[run] ROS 소스: 프로젝트 레이아웃 ($PROJECT_DIR)"
  # shellcheck disable=SC1091
  source "$PROJECT_DIR/setup_env.sh"
elif [[ -n "$ROS_DISTRO" ]]; then
  echo "[run] ROS 소스: 이미 source 된 환경 사용 (ROS_DISTRO=$ROS_DISTRO)"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  echo "[run] ROS 소스: /opt/ros/humble (메시지 패키지 overlay 는 별도 확인 필요)"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[run] 경고: ROS 환경을 찾지 못했습니다. crazyswarm2/coshow_interfaces overlay 를 먼저 source 하거나 COSHOW_ROS_SETUP 을 지정하세요." >&2
fi

echo "[run] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(기본 0)}"
mkdir -p "$DASHBOARD_DIR/logs" "$DASHBOARD_DIR/run"
cd "$DASHBOARD_DIR"
# Process substitution preserves Python as this script's PID and signal target.
# A pipeline with Python on its left would make shutdown ownership ambiguous.
exec python3 -u server.py --config config/dashboard.yaml "$@" \
  > >(tee -a "$DASHBOARD_DIR/logs/dashboard.log") 2>&1
