#!/usr/bin/env python3
"""Measure both M2 WebSocket roles using only aiohttp and the standard library.

Examples: python3 dashboard/tests/probe_m2.py --seconds 5 --expect-external
          python3 dashboard/tests/probe_m2.py --url http://127.0.0.1:8080 --start-mock
"""
import argparse
import asyncio
from collections import Counter
import contextlib
import json
import time

import aiohttp


async def wait_for(predicate, description, seconds=12):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.03)
    raise AssertionError('Timed out: ' + description)


class Measurements:
    def __init__(self, role, hello):
        assert hello.get('type') == 'hello', hello
        for field in ('drones', 'limos', 'field', 'bases', 'lanes', 'display_names', 'idle_lines'):
            assert field in hello, 'hello missing ' + field
        self.role, self.hello, self.hello_count = role, hello, 1
        self.state, self.states, self.frames, self.seen_frames = None, [], Counter(), set()
        self.started = None
        self.phases = set()

    async def receive(self, ws):
        async for message in ws:
            if message.type == aiohttp.WSMsgType.TEXT:
                payload = json.loads(message.data)
                if payload.get('type') == 'hello':
                    self.hello_count += 1
                    raise AssertionError('hello repeated on ' + self.role)
                assert payload.get('type') == 'state', payload
                self.state = payload
                if self.started is not None:
                    self.states.append(time.monotonic())
                    self.phases.add((payload.get('mission') or {}).get('phase'))
            elif message.type == aiohttp.WSMsgType.BINARY:
                payload = message.data
                assert len(payload) > 4 and payload[1:3] == b'\xff\xd8' and payload[-2:] == b'\xff\xd9'
                index = payload[0]
                assert 0 <= index < len(self.hello['drones']), index
                self.seen_frames.add(index)
                if self.started is not None:
                    self.frames[index] += 1
            elif message.type == aiohttp.WSMsgType.ERROR:
                raise AssertionError(str(ws.exception()))

    def validate(self, seconds, expect_external=False):
        expected = 10 * seconds
        assert abs(len(self.states) - expected) <= max(5, expected * .15), (self.role, len(self.states), expected)
        assert self.hello_count == 1
        roles = self.hello['drones']
        assert set(self.frames) == set(range(len(roles))), self.frames
        assert sum(self.frames.values()) >= len(roles) * seconds * 5, self.frames
        assert all(count <= seconds * 15 + 3 for count in self.frames.values()), self.frames
        state = self.state
        robots = state['robots']
        assert len(robots) == 14, 'Expected 10 drones + 4 LIMOs: ' + str(len(robots))
        assert all(row.get('fleet_id') and 'role' in row for row in robots.values())
        drone_names = [name for name, row in robots.items() if row['kind'] == 'drone']
        spare_names = [name for name in drone_names if robots[name]['role'] is None]
        limo_names = [name for name, row in robots.items() if row['kind'] == 'limo']
        assert len(drone_names) == 10 and len(spare_names) == 6 and len(limo_names) == 4
        assert {name for name in drone_names if robots[name]['role']} == set(roles)
        freshness = self.hello['freshness_s']
        for name in roles:
            row = robots[name]
            assert 0 <= row['pose_age'] < freshness['pose'], (name, row)
            assert 0 <= row['status_age'] < freshness['status'], (name, row)
            assert row['camera']['stream_ok'] is True
            assert row['camera']['fps'] >= self.hello['camera_min_fps']
            assert 0 <= row['camera']['frame_age'] < freshness['camera']
        for name in limo_names:
            assert 0 <= robots[name]['pose_age'] < freshness['odom'], (name, robots[name])
        disconnected = [name for name in spare_names if robots[name]['status_age'] is None
                        or robots[name]['status_age'] >= freshness['status']]
        assert len(disconnected) == 1, disconnected
        advisory = next(row for row in state['checklist'] if row['id'] == disconnected[0] + '.radio')
        assert advisory['blocking'] is False and advisory['ok'] is False
        assert state['mission'] is not None and 0 <= state['mission']['age'] < 2.0
        assert state['preflight'] is not None and 0 <= state['preflight']['age'] < 2.0
        if expect_external:
            assert state['run']['state'] == 'IDLE'
            assert state['run']['external_bt'] is True
            assert state['run']['bt_pid'] is None and state['run']['preflight_pid'] is None
            foreign = next(row for row in state['checklist'] if row['id'] == 'global.external_bt')
            assert foreign['blocking'] is True and foreign['ok'] is False
            assert state['stack']['crazyflie_server'] == 'external'
            assert state['stack']['aideck'] == 'external'
        gaps = [b - a for a, b in zip(self.states, self.states[1:])]
        return dict(role=self.role, hello=self.hello_count, seconds=seconds, states=len(self.states),
                    state_hz=round(len(self.states) / seconds, 2),
                    maximum_state_gap_s=round(max(gaps), 3) if gaps else None,
                    binary_frames=sum(self.frames.values()), per_camera=dict(self.frames),
                    robot_count=len(robots), role_drones=roles, spare_drones=spare_names, limos=limo_names,
                    disconnected_spare=disconnected[0], spare_link_blocking=advisory['blocking'],
                    mission_phases=sorted(p for p in self.phases if p),
                    mission_age=round(state['mission']['age'], 3),
                    external_bt=state['run']['external_bt'], run_state=state['run']['state'])


async def probe(url, seconds=5.0, start_mock=False, expect_external=False):
    async with aiohttp.ClientSession() as session:
        sockets, readers, measurements = {}, [], {}
        try:
            for role in ('visitor', 'admin'):
                ws = await session.ws_connect(url.rstrip('/') + '/ws?role=' + role)
                sockets[role] = ws
                hello = await ws.receive_json(timeout=5)
                item = Measurements(role, hello)
                measurements[role] = item
                readers.append(asyncio.create_task(item.receive(ws)))
            await wait_for(lambda: all(item.state and len(item.seen_frames) == len(item.hello['drones'])
                                       for item in measurements.values()), 'first state and camera frames')
            admin = measurements['admin']
            if start_mock:
                assert admin.hello['mock'] is True
                await sockets['admin'].send_json({'cmd': 'preflight'})
                await wait_for(lambda: admin.state['run']['state'] == 'READY', 'mock READY')
                blocking = [row for row in admin.state['checklist'] if row['blocking'] and not row['ok']]
                assert not blocking, blocking
                await sockets['admin'].send_json({'cmd': 'start'})
                await wait_for(lambda: admin.state['run']['state'] == 'RUNNING', 'mock RUNNING')
            elif expect_external:
                await wait_for(lambda: all(item.state['run']['external_bt'] for item in measurements.values()),
                               'two fresh external BT mission receipts')
            for item in measurements.values():
                item.started = time.monotonic()
            await asyncio.sleep(seconds)
            for task in readers:
                if task.done():
                    task.result()
            results = [item.validate(seconds, expect_external) for item in measurements.values()]
            for result in results:
                print(json.dumps(result, ensure_ascii=False), flush=True)
            print('PASS: hello once, state 10 Hz, indexed JPEG, 14-robot fleet, advisory spare link', flush=True)
            return results
        finally:
            for task in readers:
                task.cancel()
            await asyncio.gather(*readers, return_exceptions=True)
            for ws in sockets.values():
                with contextlib.suppress(Exception):
                    await ws.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8080')
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--start-mock', action='store_true')
    parser.add_argument('--expect-external', action='store_true')
    args = parser.parse_args()
    assert args.seconds > 0
    asyncio.run(probe(args.url, args.seconds, args.start_mock, args.expect_external))


if __name__ == '__main__':
    main()
