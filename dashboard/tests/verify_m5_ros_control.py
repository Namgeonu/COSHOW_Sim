#!/usr/bin/env python3
"""Real local DDS service probe; requires ROS_DOMAIN_ID=87, no hardware endpoints.

Run with: python3 dashboard/tests/verify_m5_ros_control.py
Uses ROS + the dashboard's existing runtime dependencies, not pytest.
"""
import asyncio
import json
import os
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from dashboard.config import load_config
from dashboard.ros_io import ROSIO, interface_specs
from dashboard.state import TelemetryStore


async def probe():
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rosidl_runtime_py.utilities import get_service

    assert os.environ.get('ROS_DOMAIN_ID') == '87', 'Run this local probe in ROS_DOMAIN_ID=87'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'Run with ROS_LOCALHOST_ONLY=1'
    cfg = load_config(mock=True)
    prefix = '/dashboard_control_probe_' + uuid.uuid4().hex[:10]
    cfg.raw['topics'] = {}
    cfg.raw['types']['topics'] = {}
    cfg.raw['services'] = {'land_template': prefix + '/{cf}/descend',
                           'arm_template': prefix + '/{cf}/disable'}
    cfg.raw['actions'] = {'nav_template': prefix + '/{limo}/travel'}
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node('dashboard_control_probe_services', context=context)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    group = ReentrantCallbackGroup()
    services, requests, behavior = {}, [], {}

    def callback(robot, channel):
        def receive(request, response):
            if channel == 'land':
                data = dict(height=request.height, group_mask=request.group_mask,
                            sec=request.duration.sec, nanosec=request.duration.nanosec)
            elif channel == 'arm':
                data = dict(arm=request.arm)
            else:
                data = dict(uuid=[int(value) for value in request.goal_info.goal_id.uuid],
                            sec=request.goal_info.stamp.sec, nanosec=request.goal_info.stamp.nanosec)
            requests.append(dict(robot=robot, channel=channel, request=data))
            delay, reject = behavior.get((robot, channel), (0, False))
            if delay:
                time.sleep(delay)
            if channel == 'cancel':
                response.return_code = response.ERROR_REJECTED if reject else response.ERROR_NONE
            return response
        return receive

    for spec in interface_specs(cfg)[0]:
        if spec['kind'] == 'services':
            channel, path, typename = spec['channel'], spec['name'], spec['type']
        elif spec['kind'] == 'actions':
            channel, path, typename = 'cancel', spec['name'] + '/_action/cancel_goal', 'action_msgs/srv/CancelGoal'
        else:
            continue
        assert path.startswith(prefix + '/')
        services[(spec['robot'], channel)] = node.create_service(
            get_service(typename), path, callback(spec['robot'], channel), callback_group=group)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=.02)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    store = TelemetryStore(cfg)
    io = ROSIO(cfg, store)
    try:
        await io.start()
        deadline = time.monotonic() + 3
        while not all(client.service_is_ready() for client in io.service_clients.values()):
            assert time.monotonic() < deadline, 'Dummy service discovery timed out'
            await asyncio.sleep(.02)
        sent = await asyncio.gather(*(io.land(name, .04, 2.125) for name in cfg.drones))
        assert sent == [True] * len(cfg.drones)
        assert await io.arm(cfg.drones[0], False) is True
        assert await asyncio.gather(*(io.cancel(name) for name in cfg.limos)) == [True] * len(cfg.limos)
        for entry in requests:
            data = entry['request']
            if entry['channel'] == 'land':
                assert abs(data['height'] - .04) < .00001
                assert (data['group_mask'], data['sec'], data['nanosec']) == (0, 2, 125000000)
            elif entry['channel'] == 'arm':
                assert data == {'arm': False}
            else:
                assert data == {'uuid': [0] * 16, 'sec': 0, 'nanosec': 0}
        print('WIRE', json.dumps(requests, ensure_ascii=False))
        before = len(requests)
        spare = next(name for name, meta in cfg.robots.items() if meta['kind'] == 'drone' and meta['role'] is None)
        assert await io.land(spare, .04, 2) is False
        assert await io.arm(spare, False) is False
        assert await io.arm(cfg.drones[0], True) is False
        assert len(requests) == before
        target = cfg.limos[0]
        behavior[(target, 'cancel')] = (0, True)
        assert await io.cancel(target) is False
        behavior[(target, 'cancel')] = (1.3, False)
        ticks = []

        async def heartbeat():
            for _ in range(24):
                ticks.append(time.monotonic())
                await asyncio.sleep(.025)

        started = time.monotonic()
        result, _ = await asyncio.gather(io.cancel(target), heartbeat())
        elapsed = time.monotonic() - started
        assert result is False and .9 <= elapsed <= 1.15
        maximum_gap = max(b - a for a, b in zip(ticks, ticks[1:]))
        assert maximum_gap < .12
        await asyncio.sleep(.4)  # Late DDS response must not fail the asyncio loop.
        missing = io.service_clients.pop((cfg.drones[0], 'arm'))
        assert await io.arm(cfg.drones[0], False) is None
        io.service_clients[(cfg.drones[0], 'arm')] = missing
        missing_role = cfg.drones[-1]
        node.destroy_service(services.pop((missing_role, 'land')))
        deadline = time.monotonic() + 3
        while io.service_clients[(missing_role, 'land')].service_is_ready():
            assert time.monotonic() < deadline, 'Removed dummy service remained ready'
            await asyncio.sleep(.02)
        assert await io.land(missing_role, .04, 2) is None
        await asyncio.gather(io.close(), io.close())
        assert await io.cancel(target) is None
        print('BOUNDS', json.dumps(dict(timeout_seconds=round(elapsed, 3), asyncio_ticks=len(ticks),
                                        maximum_tick_gap=round(maximum_gap, 3))))
        print('WARNINGS', json.dumps([event['text'] for event in store.events if event['level'] == 'warning'],
                                      ensure_ascii=False))
        print('PASS: configured role land/arm(false), zero UUID/stamp cancel, rejection, '
              '1s timeout, responsive asyncio, missing client/service, spare protection, idempotent close')
    finally:
        await io.close()
        stop.set()
        thread.join(2)
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        context.shutdown()


if __name__ == '__main__':
    asyncio.run(probe())
