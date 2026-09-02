"""COSHOW A-1 서버: 실물 crazyswarm2 서버 흉내 (radio -> 내부 서비스 전달).

실물 crazyswarm2 서버와 동일한 바깥 인터페이스를 노출:
  - /cf_a/takeoff, /cf_a/go_to, /cf_a/land  (개별)
  - /all/takeoff, /all/go_to, /all/land     (브로드캐스트)
실물과 유일한 차이: radio(CRTP) 대신 내부 서비스(/cf_a/_impl/...)로 각 드라이버에 전달.

BT는 이 서버에만 명령을 보내면 된다 (실물이든 시뮬이든 동일).
  BT --/cf_a/go_to--> [이 서버] --/cf_a/_impl/go_to--> [cf_node 드라이버] --> Webots

로봇 목록은 파라미터(robots)로 받는다. 예:
  ros2 run <pkg> coshow_server --ros-args -p robots:="['cf_a','cf_b','cf_c','cf_d']"
또는 launch에서 parameters=[{'robots': [...]}]

pose는 각 드라이버가 /cf_a/pose로 직접 발행
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from functools import partial

from crazyflie_interfaces.srv import Takeoff, Land, GoTo


class CoshowServer(Node):
    def __init__(self):
        super().__init__('coshow_server',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # 로봇 목록 (파라미터)
        self.declare_parameter('robots', ['cf_a', 'cf_b', 'cf_c', 'cf_d'])
        self.robots = self.get_parameter('robots').value
        self.get_logger().info(f'[coshow_server] robots={list(self.robots)}')

        # 각 드라이버의 내부 서비스로 전달할 클라이언트
        self._impl_clients = {}   # name -> {'takeoff':client, 'go_to':client, 'land':client}

        for name in self.robots:
            self._impl_clients[name] = {
                'takeoff': self.create_client(Takeoff, f'/{name}/_impl/takeoff'),
                'go_to':   self.create_client(GoTo,   f'/{name}/_impl/go_to'),
                'land':    self.create_client(Land,   f'/{name}/_impl/land'),
            }
            # 바깥 인터페이스 (실물 crazyswarm2와 동일한 이름)
            self.create_service(
                Takeoff, f'/{name}/takeoff',
                partial(self._takeoff_cb, name=name))
            self.create_service(
                GoTo, f'/{name}/go_to',
                partial(self._goto_cb, name=name))
            self.create_service(
                Land, f'/{name}/land',
                partial(self._land_cb, name=name))

        # 브로드캐스트 (all/)
        self.create_service(Takeoff, '/all/takeoff',
                            partial(self._takeoff_cb, name='all'))
        self.create_service(GoTo, '/all/go_to',
                            partial(self._goto_cb, name='all'))
        self.create_service(Land, '/all/land',
                            partial(self._land_cb, name='all'))

        self.get_logger().info(
            '[coshow_server] 인터페이스 준비 완료 '
            '(/{name}/takeoff|go_to|land + /all/...). '
            'radio 대신 /{name}/_impl/... 로 각 드라이버에 전달.')

    def _targets(self, name):
        return list(self.robots) if name == 'all' else [name]

    def _takeoff_cb(self, request, response, name='all'):
        for n in self._targets(name):
            cli = self._impl_clients[n]['takeoff']
            if cli.service_is_ready():
                cli.call_async(request)   # 비동기 전달 (드라이버가 실행)
            else:
                self.get_logger().warning(f'[{n}] _impl/takeoff 아직 준비 안 됨')
        self.get_logger().info(f'[{name}] takeoff -> 전달 (h={request.height})')
        return response

    def _goto_cb(self, request, response, name='all'):
        for n in self._targets(name):
            cli = self._impl_clients[n]['go_to']
            if cli.service_is_ready():
                cli.call_async(request)
            else:
                self.get_logger().warning(f'[{n}] _impl/go_to 아직 준비 안 됨')
        self.get_logger().info(
            f'[{name}] go_to -> 전달 '
            f'({request.goal.x},{request.goal.y},{request.goal.z})')
        return response

    def _land_cb(self, request, response, name='all'):
        for n in self._targets(name):
            cli = self._impl_clients[n]['land']
            if cli.service_is_ready():
                cli.call_async(request)
        self.get_logger().info(f'[{name}] land -> 전달')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoshowServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
