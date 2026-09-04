"""COSHOW 실기체 리허설용 드라이버 launch (Webots 예행연습).

drivers.launch.py 와의 차이:
  - 리모 드라이버를 띄우지 않는다. 리허설 월드(coshow_rehearsal.wbt)에 리모가 없다.
    리모 인터페이스는 tools/fake_limo.py 가 대신 제공한다.
  - Ros2Supervisor 는 그대로 포함한다. 이게 빠지면 /clock 이 나오지 않아
    시뮬레이션이 진행되지 않고, 드론 pose 도 카메라 프레임도 흐르지 않는다.
    (crazyflie_driver.launch.py 만 단독으로 띄우면 이 문제가 생긴다)

실행:
  webots ~/COSHOW/worlds/coshow_rehearsal.wbt      # 먼저 월드를 띄우고
  ros2 launch ~/COSHOW/rehearsal_drivers.launch.py
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher

HOME = os.path.expanduser('~')


def generate_launch_description():
    ros2_supervisor = Ros2SupervisorLauncher(port='1234')
    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HOME, 'COSHOW', 'crazyflie_driver.launch.py')))
    return LaunchDescription([ros2_supervisor, drone_launch])
