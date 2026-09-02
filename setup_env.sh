#!/bin/bash
# COSHOW 프로젝트 공용 환경 설정
# 사용: source ~/COSHOW/setup_env.sh
source /opt/ros/humble/setup.bash
source ~/COSHOW/ros2_ws/install/setup.bash
export PYTHONPATH=~/COSHOW/controllers:~/COSHOW/crazyflie-firmware/build:$(ls -d ~/COSHOW/ros2_ws/install/crazyflie_sim/lib/python3.*/site-packages | head -1):$PYTHONPATH
