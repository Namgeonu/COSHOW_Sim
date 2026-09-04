"""Launch the AI-Deck ArUco UDP node.

Examples::

    ros2 launch aideck_aruco_ros aideck_aruco.launch.py
    ros2 launch aideck_aruco_ros aideck_aruco.launch.py display:=true
    ros2 launch aideck_aruco_ros aideck_aruco.launch.py \
        params_file:=/abs/path/to/my_drones.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('aideck_aruco_ros'), 'config', 'drones.yaml'
    )

    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    display = LaunchConfiguration('display')
    log_detections = LaunchConfiguration('log_detections')
    publish_empty_text = LaunchConfiguration('publish_empty_text')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('namespace', default_value='aideck'),
        DeclareLaunchArgument('display', default_value='false'),
        DeclareLaunchArgument('log_detections', default_value='false'),
        DeclareLaunchArgument('publish_empty_text', default_value='false'),

        Node(
            package='aideck_aruco_ros',
            executable='aideck_aruco_node',
            name='aideck_aruco_node',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'display': display,
                    'log_detections': log_detections,
                    'publish_empty_text': publish_empty_text,
                },
            ],
        ),
    ])
