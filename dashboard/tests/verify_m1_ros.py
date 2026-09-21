#!/usr/bin/env python3
"""M1 evidence using the actual BT/preflight executables in isolated Docker ROS.

No drone/action services are provided. Pose fixtures come from the existing BT
configuration; the only temporary config override disables the pygame window.
Run inside dashboard/docker/run.sh with ROS_LOCALHOST_ONLY=1.
"""
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from crazyflie_interfaces.msg import Status
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import yaml


ROOT = Path(__file__).resolve().parents[2]
QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wait_for(predicate, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for ROS evidence")


class EvidenceNode(Node):
    def __init__(self, config):
        super().__init__('dashboard_m1_evidence')
        self.samples = {'mission': [], 'preflight': []}
        self.publish_poses = False
        self.pose_publishers = []
        self.publish_status = False
        self.status_publishers = []
        self.status_ticks = 0
        for key, topic in (('mission', '/coshow/mission_state'),
                           ('preflight', '/preflight/status')):
            self.create_subscription(
                String, topic,
                lambda msg, name=key: self.samples[name].append(
                    (time.monotonic(), json.loads(msg.data))), QOS)
        for robot, data in config['coshow']['drones'].items():
            msg = PoseStamped()
            msg.pose.position.x, msg.pose.position.y = map(float, data['base'])
            msg.pose.orientation.w = 1.0
            self.pose_publishers.append(
                (self.create_publisher(PoseStamped, '/{}/pose'.format(robot), 10), msg))
            self.status_publishers.append(self.create_publisher(Status, '/{}/status'.format(robot), 10))
        for robot, data in config['coshow']['limos'].items():
            msg = Odometry()
            msg.pose.pose.position.x, msg.pose.pose.position.y = map(float, data['base'])
            msg.pose.pose.orientation.w = 1.0
            self.pose_publishers.append(
                (self.create_publisher(Odometry, data['pose_topic'], 10), msg))
        self.create_timer(0.1, self.publish)

    def publish(self):
        if self.publish_poses:
            for publisher, message in self.pose_publishers:
                publisher.publish(message)
        if self.publish_status:
            self.status_ticks += 1
            for index, publisher in enumerate(self.status_publishers):
                message = Status()
                # Change only continuous telemetry; safety flags stay fixed.
                message.battery_voltage = 3.9 + index * 0.01 + (self.status_ticks % 100) * 0.0001
                message.supervisor_info = Status.SUPERVISOR_INFO_CAN_BE_ARMED
                publisher.publish(message)


def echo_once(topic):
    command = ['ros2', 'topic', 'echo', topic, 'std_msgs/msg/String',
               '--qos-durability', 'transient_local', '--once']
    print('$ ' + ' '.join(command), flush=True)
    result = subprocess.run(command, capture_output=True, text=True, timeout=12)
    print(result.stdout.strip(), flush=True)
    if result.returncode:
        raise AssertionError(result.stderr)


def check_qos(node, topic):
    info = node.get_publishers_info_by_topic(topic)
    assert len(info) == 1, (topic, info)
    qos = info[0].qos_profile
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    print('{}: RELIABLE / TRANSIENT_LOCAL (depth verified in unit test)'.format(topic), flush=True)


def check_heartbeat(node, key, condition='unchanged payload'):
    start = time.monotonic()
    time.sleep(6.4)
    received = [t for t, _ in list(node.samples[key]) if t >= start]
    gaps = [round(b - a, 3) for a, b in zip(received, received[1:])]
    print('{} ({}): {} frames / 6.4 s; gaps={}'.format(
        key, condition, len(received), gaps), flush=True)
    assert 5 <= len(received) <= 7, (key, 'frame count', len(received))
    assert all(0.85 <= gap <= 1.25 for gap in gaps), gaps


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Use isolated local Docker ROS'
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    config = yaml.safe_load((ROOT / 'bt/scenarios/coshow/configs/coshow_rehearsal.yaml').read_text())
    config['bt_runner']['bt_visualiser']['enabled'] = False
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy',
               PYGAME_HIDE_SUPPORT_PROMPT='1', PYTHONUNBUFFERED='1')
    rclpy.init()
    node = EvidenceNode(config)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    children = []
    try:
        with tempfile.TemporaryDirectory(prefix='dashboard-m1-') as temporary:
            temp = Path(temporary)
            override = temp / 'bt.yaml'
            override.write_text(yaml.safe_dump(config, allow_unicode=True))
            with (temp / 'bt.log').open('w+') as bt_log, (temp / 'preflight.log').open('w+') as preflight_log:
                try:
                    bt = subprocess.Popen([sys.executable, 'main.py', '--config', str(override)],
                                          cwd=str(ROOT / 'bt'), env=env, stdout=bt_log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
                    children.append(bt)
                    wait_for(lambda: any(s['phase'] == 'waiting_poses' for _, s in node.samples['mission']))
                    print('BEFORE POSES:', json.dumps(node.samples['mission'][-1][1]), flush=True)
                    echo_once('/coshow/mission_state')
                    check_qos(node, '/coshow/mission_state')
                    check_heartbeat(node, 'mission')
                    node.publish_poses = True
                    wait_for(lambda: any(s['phase'] == 'observe' and s['missing_pose'] == []
                                         for _, s in node.samples['mission']))
                    print('AFTER POSES:', json.dumps(node.samples['mission'][-1][1]), flush=True)
                    echo_once('/coshow/mission_state')
                    check_heartbeat(node, 'mission')

                    preflight = subprocess.Popen(
                        [sys.executable, 'tools/preflight_node.py', '--ros-args',
                         '-p', 'server_ready_timeout:=2.0'], cwd=str(ROOT), env=env,
                        stdout=preflight_log, stderr=subprocess.STDOUT, start_new_session=True)
                    children.append(preflight)
                    wait_for(lambda: any(s['stages'][0]['result'] == 'fail'
                                         for _, s in node.samples['preflight']))
                    assert preflight.poll() is None, 'Preflight must stay alive after failure'
                    print('PREFLIGHT FAILURE:', json.dumps(node.samples['preflight'][-1][1]), flush=True)
                    echo_once('/preflight/status')
                    check_qos(node, '/preflight/status')
                    check_heartbeat(node, 'preflight')
                    node.publish_status = True
                    wait_for(lambda: any(s['drones'][next(iter(config['coshow']['drones']))]['battery_v']
                                         is not None for _, s in node.samples['preflight']))
                    ticks_before = node.status_ticks
                    check_heartbeat(node, 'preflight', 'changing /cfX/status battery at 10 Hz per drone')
                    print('STATUS FIXTURE: {} messages across {} drones during measurement'.format(
                        (node.status_ticks - ticks_before) * len(node.status_publishers),
                        len(node.status_publishers)), flush=True)
                    assert node.status_ticks - ticks_before >= 55, 'status traffic did not run'
                    print('PASS: actual BT waiting_poses -> observe, both transient publishers and 1 Hz heartbeat', flush=True)
                finally:
                    for child in children:
                        if child.poll() is None:
                            child.send_signal(signal.SIGINT)
                            try:
                                child.wait(timeout=12)
                            except subprocess.TimeoutExpired:
                                child.kill()
                                child.wait()
                        print('CHILD EXIT: pid={} rc={}'.format(child.pid, child.returncode), flush=True)
                    for name, log in (('BT', bt_log), ('PREFLIGHT', preflight_log)):
                        log.seek(0)
                        print(name + ' LOG:\n' + log.read(), flush=True)
    finally:
        executor.shutdown()
        thread.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
