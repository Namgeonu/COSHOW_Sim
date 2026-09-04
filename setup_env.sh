#!/bin/bash
# COSHOW 프로젝트 공용 환경 설정
# 사용: source ~/COSHOW/setup_env.sh
source /opt/ros/humble/setup.bash
source ~/COSHOW/ros2_ws/install/setup.bash
# crazyflie_sim 은 위 install/setup.bash 가 이미 PYTHONPATH 에 넣어준다.
# (실제 경로가 .../crazyflie_sim/local/lib/python3.10/dist-packages 라
#  예전의 .../lib/python3.*/site-packages glob 은 매번 ls 오류만 냈다)
export PYTHONPATH=~/COSHOW/controllers:~/COSHOW/crazyflie-firmware/build:$PYTHONPATH
