#!/usr/bin/env python3
"""Publish the shared mock timeline as isolated ROS fixtures, never hardware I/O.

Use only in a dedicated ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1. Configured
service/action servers are inert observation fixtures and count any requests.
The last spare intentionally has a Status publisher but receives no samples,
matching mock.scenario's link_ok=False and keeping graph checks separate from
message freshness. Unknown LIMO battery measurements remain unknown.
"""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from dashboard.config import load_config
from dashboard.mock import scenario
from dashboard.ros_io import interface_specs


def message_for(spec, message_type, fragment, jpeg):
    """Inverse fixture mapping for ROSIO.decode, using configured ROS types."""
    from rosidl_runtime_py.set_message import set_message_fields

    message = message_type()
    channel, name = spec['channel'], spec['robot']
    row = fragment['robots'].get(name, {})
    if hasattr(message, 'header'):
        # Deliberately stale, fixed header: downstream freshness must use receipt.
        message.header.stamp.sec = 1
        message.header.frame_id = 'mock_world'
    if channel == 'pose':
        pose = message.pose.pose if hasattr(message.pose, 'pose') else message.pose
        for axis in ('x', 'y', 'z'):
            setattr(pose.position, axis, float(row['pose'].get(axis, 0.0)))
        yaw = row['pose'].get('yaw', 0.0)
        pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    elif channel == 'status':
        if not row.get('link_ok', True):
            return None
        message.battery_voltage = float(row['battery_v'])
        message.rssi = int(row['rssi'])
        bits = 0
        for field, constant in (('armed', 'SUPERVISOR_INFO_IS_ARMED'),
                                ('can_fly', 'SUPERVISOR_INFO_CAN_FLY'),
                                ('tumbled', 'SUPERVISOR_INFO_IS_TUMBLED')):
            if row[field]:
                bits |= getattr(message, constant)
        message.supervisor_info = bits
        if row['low_power']:
            message.pm_state = message.PM_STATE_LOW_POWER
    elif channel == 'limo_status':
        if row['battery_v'] is None:
            return None
        message.battery_voltage = float(row['battery_v'])
    elif channel == 'frame':
        message.format, message.data = 'jpeg', jpeg
    elif channel == 'camera_fps':
        message.data = float(row['camera']['fps'])
    elif channel == 'camera_ok':
        message.data = bool(row['camera']['stream_ok'])
    elif channel == 'detections':
        set_message_fields(message, {
            'drone': name,
            'markers': [{'id': int(marker)} for marker in row['detections']],
        })
    elif channel == 'ready':
        if fragment['preflight'] is None:
            return None
        message.data = bool(fragment['preflight']['ready'])
    elif channel in ('mission', 'preflight'):
        if fragment[channel] is None:
            return None
        message.data = json.dumps(fragment[channel], ensure_ascii=False, allow_nan=False)
    else:
        raise ValueError('Unsupported fixture channel: ' + channel)
    return message


def qos_for(channel):
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                          qos_profile_sensor_data)

    if channel in ('mission', 'preflight', 'ready'):
        return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if channel == 'frame':
        return qos_profile_sensor_data
    return QoSProfile(depth=10)


class FakeROSWorld:
    def __init__(self, cfg, offset=0.0):
        import rclpy
        from rclpy.action import ActionServer
        from rclpy.executors import SingleThreadedExecutor
        from rosidl_runtime_py.utilities import get_action, get_message, get_service

        self.cfg, self.offset = cfg, offset
        self.started = time.monotonic()
        self.last_json, self.signature = -math.inf, None
        self.command_requests = 0
        self.publishers, self.services, self.actions, self.nodes = [], [], [], {}
        self.executor = SingleThreadedExecutor()
        self.jpeg = (Path(__file__).resolve().parents[1] / 'static/mock/camera.jpg').read_bytes()
        for key, configured_name in cfg.raw['nodes'].items():
            parts = configured_name.strip('/').split('/')
            namespace = '/' + '/'.join(parts[:-1])
            node = rclpy.create_node(parts[-1], namespace=namespace)
            self.nodes[key] = node
            self.executor.add_node(node)
        specs, errors = interface_specs(cfg)
        assert not errors, errors
        for spec in specs:
            if spec['type'] is None:
                continue
            if spec['kind'] == 'topics':
                owner = ('bt' if spec['channel'] == 'mission' else 'preflight'
                         if spec['channel'] in ('preflight', 'ready') else 'aideck'
                         if spec['channel'] in ('frame', 'camera_fps', 'camera_ok', 'detections')
                         else 'server')
                message_type = get_message(spec['type'])
                publisher = self.nodes[owner].create_publisher(
                    message_type, spec['name'], qos_for(spec['channel']))
                self.publishers.append((spec, publisher, message_type))
            elif spec['kind'] == 'services':
                assert cfg.robots[spec['robot']]['role'] is not None
                self.services.append(self.nodes['server'].create_service(
                    get_service(spec['type']), spec['name'], self.service_request))
            else:
                assert cfg.robots[spec['robot']]['role'] is not None
                action_type = get_action(spec['type'])
                self.actions.append(ActionServer(
                    self.nodes['server'], action_type, spec['name'],
                    execute_callback=lambda handle, kind=action_type: self.action_request(handle, kind)))
        self.nodes['server'].create_timer(0.1, self.publish)
        self.publish()

    def service_request(self, request, response):
        self.command_requests += 1
        print('UNEXPECTED FIXTURE SERVICE REQUEST: ' + type(request).__name__, flush=True)
        return response

    def action_request(self, handle, action_type):
        self.command_requests += 1
        print('UNEXPECTED FIXTURE ACTION REQUEST', flush=True)
        handle.succeed()
        return action_type.Result()

    def publish(self):
        now = time.monotonic()
        fragment = scenario(now - self.started + self.offset, self.cfg)
        mission, preflight = fragment['mission'] or {}, fragment['preflight'] or {}
        # Same transition/heartbeat schedule as MockWorld.apply.
        signature = (mission.get('phase'), preflight.get('stage'), preflight.get('ready'))
        send_json = signature != self.signature or now - self.last_json >= 1.0
        for spec, publisher, message_type in self.publishers:
            if spec['channel'] in ('mission', 'preflight', 'ready') and not send_json:
                continue
            message = message_for(spec, message_type, fragment, self.jpeg)
            if message is not None:
                publisher.publish(message)
        if send_json:
            self.signature, self.last_json = signature, now

    def close(self):
        self.executor.shutdown()
        for action in self.actions:
            action.destroy()
        for node in self.nodes.values():
            node.destroy_node()
        print('FIXTURE CLOSED: command_requests={}'.format(self.command_requests), flush=True)


def main():
    import rclpy

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--field')
    parser.add_argument('--offset', type=float, default=30.0)
    args = parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Fixture requires local-only isolated ROS'
    cfg = load_config(args.config, args.field)
    assert not cfg.errors, cfg.errors
    rclpy.init(args=[])
    world = None
    try:
        world = FakeROSWorld(cfg, args.offset)
        print('FIXTURE READY: robots={} nodes={} topics={} services={} actions={}'.format(
            len(cfg.robots), len(world.nodes), len(world.publishers), len(world.services), len(world.actions)), flush=True)
        world.executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if world:
            world.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
