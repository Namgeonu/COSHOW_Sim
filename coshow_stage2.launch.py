"""COSHOW 방향2 2단계 launch: cf_a~d 4대 (WebotsController, Jazzy 정석).

각 드론마다 WebotsController 하나 (robot_name으로 로봇 지정).
각 driver = 별도 프로세스 → cf_node.CfNode → 자기 cffirmware(static 독립).

전제:
  - 월드(coshow_stage2.wbt): cf_a~d 모두 controller "<extern>"
  - cf_X.urdf가 controllers/에 있음 (cf_node.CfNode 플러그인 지정)

실행:
  터미널1: webots ~/COSHOW/worlds/coshow_stage2.wbt
  터미널2: ros2 launch ~/COSHOW/coshow_stage2.launch.py
"""
import os

from launch import LaunchDescription
from webots_ros2_driver.webots_controller import WebotsController


def get_cf_controller(name, coshow, pypath=None):
    """드론 하나에 WebotsController를 붙인다 (robot_name으로 매칭)."""
    urdf_path = os.path.join(coshow, 'controllers', f'{name}.urdf')
    return WebotsController(
        robot_name=name,
        namespace=name,
        parameters=[
            {'robot_description': urdf_path},
        ],
        respawn=True,
    )


def generate_launch_description():
    coshow = os.path.expanduser('~/COSHOW')
    controllers_dir = os.path.join(coshow, 'controllers')
    fw = os.path.join(coshow, 'crazyflie-firmware', 'build')
    # python 버전 자동 감지 (Jazzy=3.12, Humble=3.10 등 무관)
    import glob
    sim_candidates = glob.glob(os.path.join(
        coshow, 'ros2_ws', 'install', 'crazyflie_sim',
        'lib', 'python3.*', 'site-packages'))
    sim = sim_candidates[0] if sim_candidates else os.path.join(
        coshow, 'ros2_ws', 'install', 'crazyflie_sim', 'lib',
        f'python{__import__("sys").version_info.major}.{__import__("sys").version_info.minor}',
        'site-packages')
    pypath = os.pathsep.join([controllers_dir, fw, sim,
                              os.environ.get('PYTHONPATH', '')])

    names = ['cf_a', 'cf_b', 'cf_c', 'cf_d']
    return LaunchDescription([
        get_cf_controller(name, coshow, pypath) for name in names
    ])
