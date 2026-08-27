"""LIMO 스택 launch (다중 LIMO 대응).

원본 limo_nav2.launch.py 구조 유지 + /{ns} 네임스페이스 적용.
로봇 이름은 launch 인자에서만 옴. urdf/nav2 params는 템플릿에서 로봇별로 /tmp에 생성.

사용:
  source /home/namgeonwoo/COSHOW/setup_env.sh
  ros2 launch /home/namgeonwoo/COSHOW/limo_stack.launch.py                 # limo_a
  ros2 launch /home/namgeonwoo/COSHOW/limo_stack.launch.py robots:=limo_a,limo_b
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

HOME = os.path.expanduser('~')
COSHOW = os.path.join(HOME, 'COSHOW')
URDF_TPL = os.path.join(COSHOW, 'controllers', 'limo.urdf')        # __NS__ 자리표시자
ROS2CONTROL = os.path.join(COSHOW, 'config', 'limo_ros2control.yaml')
NAV2_SRC = os.path.join(COSHOW, 'config', 'limo_nav2_params.yaml')


def make_urdf(ns):
    """템플릿의 __NS__를 로봇 이름으로 치환해 /tmp/{ns}.urdf 생성."""
    with open(URDF_TPL) as f:
        s = f.read()
    assert '__NS__' in s, 'limo.urdf 템플릿에 __NS__ 자리표시자가 없음'
    s = s.replace('__NS__', ns)
    out = f'/tmp/{ns}.urdf'
    with open(out, 'w') as f:
        f.write(s)
    return out


def make_nav2_params(ns):
    """Nav2 params(기본 이름 템플릿)의 프레임/토픽을 {ns} 버전으로 치환."""
    import re as _re
    with open(NAV2_SRC) as f:
        s = f.read()
    # 최상위 노드 키(들여쓰기 0)에 /{ns} 접두 → 네임스페이스된 노드가 파라미터를 인식.
    _top_keys = ['bt_navigator', 'controller_server', 'global_costmap',
                 'local_costmap', 'planner_server', 'behavior_server']
    for _k in _top_keys:
        s = _re.sub(rf'(?m)^{_k}:', f'/{ns}/{_k}:', s)
    s = s.replace('robot_base_frame: base_link', f'robot_base_frame: {ns}/base_link')
    s = s.replace('global_frame: odom', f'global_frame: {ns}/odom')
    s = s.replace('local_frame: odom', f'local_frame: {ns}/odom')
    s = s.replace('odom_topic: /odom', f'odom_topic: /{ns}/odom')
    s = s.replace('topic: /scan', f'topic: /{ns}/scan')
    out = f'/tmp/{ns}_nav2_params.yaml'
    with open(out, 'w') as f:
        f.write(s)
    return out


def make_limo_stack(ns):
    urdf = make_urdf(ns)
    nav2_params = make_nav2_params(ns)
    with open(urdf) as f:
        robot_desc = f.read()

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=ns, output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': f'{ns}/',
            'use_sim_time': True,
        }])

    driver = WebotsController(
        robot_name=ns,
        namespace=ns,
        parameters=[
            {'robot_description': urdf, 'set_robot_state_publisher': False, 'use_sim_time': True},
            ROS2CONTROL,
        ],
        remappings=[
            (f'/{ns}/diffdrive_controller/cmd_vel', f'/{ns}/cmd_vel'),
            (f'/{ns}/diffdrive_controller/odom', f'/{ns}/_unused_odom_encoder'),
            (f'/{ns}/laser', f'/{ns}/scan'),
        ],
        respawn=True)

    cm_timeout = ['--controller-manager-timeout', '50']
    jsb = Node(package='controller_manager', executable='spawner',
               namespace=ns, output='screen',
               arguments=['joint_state_broadcaster',
                          '-c', f'/{ns}/controller_manager'] + cm_timeout)
    ddc = Node(package='controller_manager', executable='spawner',
               namespace=ns, output='screen',
               arguments=['diffdrive_controller',
                          '-c', f'/{ns}/controller_manager'] + cm_timeout)
    waiting = WaitForControllerConnection(
        target_driver=driver, nodes_to_start=[jsb, ddc])

    static_map_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        namespace=ns, output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['0', '0', '0', '0', '0', '0', 'map', f'{ns}/odom'])

    use_sim = {'use_sim_time': True}
    nav2_nodes = [
        Node(package='nav2_controller', executable='controller_server',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
        Node(package='nav2_planner', executable='planner_server',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
        Node(package='nav2_behaviors', executable='behavior_server',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             namespace=ns, output='screen', parameters=[nav2_params, use_sim]),
    ]
    lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', namespace=ns, output='screen',
        parameters=[{
            'use_sim_time': True, 'autostart': True,
            'node_names': ['controller_server', 'planner_server',
                           'behavior_server', 'bt_navigator'],
        }])
    nav2_delayed = TimerAction(period=12.0, actions=nav2_nodes + [lifecycle])

    return [rsp, driver, waiting, static_map_odom, nav2_delayed]


def launch_setup(context):
    robots = [r.strip() for r in
              LaunchConfiguration('robots').perform(context).split(',') if r.strip()]
    actions = []
    for ns in robots:
        actions += make_limo_stack(ns)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='limo_a',
                              description='쉼표 구분 LIMO 이름 목록 (월드의 로봇 name과 일치)'),
        OpaqueFunction(function=launch_setup),
    ])
