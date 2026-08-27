"""COSHOW 1단계 launch: LIMO 1대 단독 cmd_vel 테스트 (v2 수정판).

수정 내역 (v1 -> v2):
  - robot_state_publisher 노드 추가: /robot_description 토픽 발행
    (controller_manager 가 이 토픽으로 URDF 를 받아 초기화됨)
  - WebotsController 에 set_robot_state_publisher: True 추가
  -> v1 에서 "Waiting for data on 'robot_description' topic" 무한 대기로
     컨트롤러가 안 뜨던 문제 해결
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

COSHOW = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    urdf_path = os.path.join(COSHOW, 'controllers', 'limo_a.urdf')
    ros2_control_params = os.path.join(COSHOW, 'config', 'limo_ros2control.yaml')

    mappings = [
        ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
        ('/diffdrive_controller/odom', '/odom'),
        ('/limo_a/laser', '/scan'),
    ]

    # 관문 1: 진짜 URDF 를 읽어 base_link->laser_link TF 발행
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc
        }],
    )

    limo_driver = WebotsController(
        robot_name='limo_a',
        parameters=[
            {'robot_description': urdf_path,
             'set_robot_state_publisher': True},
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50']
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    ros_control_spawners = [joint_state_broadcaster_spawner, diffdrive_controller_spawner]

    waiting = WaitForControllerConnection(
        target_driver=limo_driver,
        nodes_to_start=ros_control_spawners,
    )

    return LaunchDescription([
        robot_state_publisher,
        limo_driver,
        waiting,
    ])
