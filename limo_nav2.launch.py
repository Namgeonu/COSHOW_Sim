"""COSHOW 2단계 launch: LIMO 단독 + Nav2 (좌표 명령 NavigateToPose 검증).

구성:
  [1단계 그대로]
    - WebotsController(limo_a) 드라이버 (MCU 대체)
    - robot_state_publisher (base_link->laser_link TF, 관문1 완료분)
    - joint_state_broadcaster + diffdrive_controller

  [2단계 신규]
    - static_transform_publisher: map -> odom 고정 (방법 A, GT 대용)
    - Nav2: controller/planner/bt_navigator/behavior + costmap (빈 지도)
    - lifecycle_manager: Nav2 노드 순차 기동

⚠️ 버전 이식 (Ubuntu 22.04/Humble): config/limo_nav2_params.yaml 헤더 주석 참조.
   cmd_vel 타입(enable_stamped_cmd_vel)이 버전별로 다름. 방법2(파라미터) 사용 중.
   Humble에서 안 맞으면 twist_stamper(방법1)로 폴백.

실행:
  터미널1: webots ~/COSHOW/worlds/limo_test.wbt
  터미널2: ros2 launch ~/COSHOW/limo_nav2.launch.py
  터미널3: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
             "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}}}}"
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

COSHOW = os.path.dirname(os.path.realpath(__file__))


def generate_launch_description():
    urdf_path = os.path.join(COSHOW, 'controllers', 'limo_a.urdf')
    ros2_control_params = os.path.join(COSHOW, 'config', 'limo_ros2control.yaml')
    nav2_params = os.path.join(COSHOW, 'config', 'limo_nav2_params.yaml')

    # ===== 1단계 그대로: 드라이버 + 컨트롤러 + TF =====
    mappings = [
        ('/diffdrive_controller/cmd_vel', '/cmd_vel'),
        # DiffDriveController 는 cmd_vel->바퀴 전용. 엔코더 odom 은 안 씀(GT가 /odom 담당).
        # 엔코더 odom 을 안 쓰는 이름으로 보내 /odom 충돌 방지.
        ('/diffdrive_controller/odom', '/_unused_odom_encoder'),
        ('/limo_a/laser', '/scan'),
    ]

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}],
    )

    limo_driver = WebotsController(
        robot_name='limo_a',
        parameters=[
            {'robot_description': urdf_path, 'set_robot_state_publisher': True},
            ros2_control_params,
        ],
        remappings=mappings,
        respawn=True,
    )

    cm_timeout = ['--controller-manager-timeout', '50']
    jsb_spawner = Node(package='controller_manager', executable='spawner',
                       output='screen', arguments=['joint_state_broadcaster'] + cm_timeout)
    ddc_spawner = Node(package='controller_manager', executable='spawner',
                       output='screen', arguments=['diffdrive_controller'] + cm_timeout)
    waiting = WaitForControllerConnection(
        target_driver=limo_driver,
        nodes_to_start=[jsb_spawner, ddc_spawner],
    )

    # ===== 2단계 신규: map->odom 정적 TF (방법 A) =====
    # map 과 odom 을 같은 위치로 고정. 로봇은 원점에서 출발, odom 으로 추적.
    static_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ===== 2단계 신규: Nav2 노드들 =====
    use_sim_time = {'use_sim_time': True}
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_planner', executable='planner_server',
             output='screen', parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_behaviors', executable='behavior_server',
             output='screen', parameters=[nav2_params, use_sim_time]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             output='screen', parameters=[nav2_params, use_sim_time]),
    ]

    # lifecycle_manager: 위 Nav2 노드들을 순차적으로 configure/activate
    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['controller_server', 'planner_server',
                           'behavior_server', 'bt_navigator'],
        }],
    )

    # Nav2 는 드라이버/컨트롤러가 뜬 뒤 시작되도록 약간 지연
    nav2_delayed = TimerAction(period=12.0, actions=nav2_nodes + [lifecycle_mgr])

    return LaunchDescription([
        robot_state_publisher,
        limo_driver,
        waiting,
        static_map_odom,
        nav2_delayed,
    ])
