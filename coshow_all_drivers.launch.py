"""COSHOW 통합 드라이버 launch: 드론 4대 + LIMO 드라이버를 한 번에 실행.

사용:
  source /home/namgeonwoo/COSHOW/setup_env.sh
  ros2 launch /home/namgeonwoo/COSHOW/coshow_all_drivers.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from webots_ros2_driver.webots_launcher import Ros2SupervisorLauncher

HOME = os.path.expanduser('~')

def generate_launch_description():
    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HOME, 'COSHOW', 'coshow_stage2.launch.py')))
    limo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(HOME, 'COSHOW', 'limo_stack.launch.py')),
        launch_arguments={'robots': 'limo_a,limo_b'}.items())
    ros2_supervisor = Ros2SupervisorLauncher(port='1234')
    return LaunchDescription([ros2_supervisor, drone_launch, limo_launch])
