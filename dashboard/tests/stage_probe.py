#!/usr/bin/env python3
"""Read-only admin WebSocket recorder for human-operated Stages A–D.

This tool never sends commands. --record saves every hello/state as JSONL;
--dump-poses writes receipt-time pose CSV. Tables use server freshness limits.
"""
import argparse
import asyncio
from collections import Counter, deque
from contextlib import contextmanager
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import aiohttp


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds')


def number(value, digits=2):
    return ('{:.%df}' % digits).format(value) if type(value) in (int, float) and math.isfinite(value) else '—'


class ProbeRecorder:
    def __init__(self, record=None, poses=None, fleet=False):
        self.record, self.poses, self.fleet = record, poses, fleet
        self.hello, self.state = {}, None
        self.phase, self.run_state = None, None
        self.event_keys, self.event_order = set(), deque()
        self.frames, self.window_started = Counter(), None
        self.pose_writer = None
        if poses is not None:
            fields = ['received_at', 'monotonic_s', 'robot', 'role', 'fleet_id', 'kind',
                      'x', 'y', 'z', 'yaw', 'pose_age', 'pose_fresh']
            self.pose_writer = csv.DictWriter(poses, fieldnames=fields)
            self.pose_writer.writeheader()

    def fresh_pose(self, robot):
        age = robot.get('pose_age')
        key = 'pose' if robot.get('kind') == 'drone' else 'odom'
        limit = self.hello.get('freshness_s', {}).get(key)
        return (isinstance(robot.get('pose'), dict) and type(age) in (int, float) and
                type(limit) in (int, float) and math.isfinite(age) and 0 <= age <= limit)

    def ingest(self, message, received_at=None, monotonic_s=None):
        if not isinstance(message, dict) or message.get('type') not in ('hello', 'state'):
            raise ValueError('Expected a hello or state object')
        stamp = received_at or utc_now()
        now = time.monotonic() if monotonic_s is None else monotonic_s
        if self.record is not None:
            self.record.write(json.dumps(dict(received_at=stamp, monotonic_s=now, message=message),
                                         ensure_ascii=False, allow_nan=False) + '\n')
            self.record.flush()
        if self.window_started is None:
            self.window_started = now
        if message['type'] == 'hello':
            self.hello = message
            return ['{} HELLO roles={}+{} mock={}'.format(stamp, len(message.get('drones', [])),
                                                        len(message.get('limos', [])), message.get('mock'))]
        self.state = message
        lines = []
        phase = (message.get('mission') or {}).get('phase')
        run = (message.get('run') or {}).get('state')
        if phase != self.phase:
            self.phase = phase
            if phase:
                lines.append('{} PHASE {}'.format(stamp, phase))
        if run != self.run_state:
            self.run_state = run
            lines.append('{} RUN {} external_bt={}'.format(stamp, run, message.get('run', {}).get('external_bt', False)))
        for event in message.get('events', []):
            key = (event.get('t'), event.get('level'), event.get('text'))
            if key in self.event_keys:
                continue
            self.event_keys.add(key)
            self.event_order.append(key)
            if len(self.event_order) > 2048:
                self.event_keys.discard(self.event_order.popleft())
            lines.append('{} EVENT event_t={} {} {}'.format(stamp, *key))
        if self.pose_writer is not None:
            for name, robot in message.get('robots', {}).items():
                pose = robot.get('pose') or {}
                row = dict(received_at=stamp, monotonic_s=now, robot=name, role=robot.get('role'),
                           fleet_id=robot.get('fleet_id'), kind=robot.get('kind'),
                           pose_age=robot.get('pose_age'), pose_fresh='yes' if self.fresh_pose(robot) else 'no')
                row.update({key: pose.get(key) for key in ('x', 'y', 'z', 'yaw')})
                self.pose_writer.writerow(row)
            self.poses.flush()
        return lines

    def frame(self, payload):
        drones = self.hello.get('drones', [])
        if payload and payload[0] < len(drones):
            self.frames[drones[payload[0]]] += 1

    def table(self, monotonic_s=None):
        now = time.monotonic() if monotonic_s is None else monotonic_s
        seconds = max(.001, now - self.window_started) if self.window_started is not None else 1.
        robots = (self.state or {}).get('robots', {})
        names = list(robots) if self.fleet else self.hello.get('drones', []) + self.hello.get('limos', [])
        lines = ['TABLE {} window_s={} run={} phase={}'.format(utc_now(), number(seconds), self.run_state, self.phase),
                 'robot\trole\tkind\tstatus_age\tbattery_v\trssi\tarmed\tping\tcamera_fps\tcamera_rx_fps\tstream_ok\tframe_age\tpose_age\tpose']
        for name in names:
            robot = robots.get(name, {})
            camera = robot.get('camera') or {}
            lines.append('\t'.join(str(value) for value in (
                name, robot.get('role') or 'spare', robot.get('kind', '—'), number(robot.get('status_age')),
                number(robot.get('battery_v')), number(robot.get('rssi'), 0), robot.get('armed'),
                (robot.get('ping') or {}).get('ok'), number(camera.get('fps')),
                number(self.frames[name] / seconds), camera.get('stream_ok'), number(camera.get('frame_age')),
                number(robot.get('pose_age')), 'fresh' if self.fresh_pose(robot) else 'STALE')))
        self.frames.clear()
        self.window_started = now
        return '\n'.join(lines)


async def collect(url, recorder, seconds=0, interval=5, emit=print):
    """Receive only; reconnecting never repeats a user command."""
    end = time.monotonic() + seconds if seconds > 0 else float('inf')
    next_table = time.monotonic() + interval
    socket_url = url.rstrip('/') + '/ws?role=admin'
    timeout = aiohttp.ClientTimeout(total=None, sock_connect=5)
    async with aiohttp.ClientSession(timeout=timeout) as session:
        while time.monotonic() < end:
            try:
                async with session.ws_connect(socket_url) as socket:
                    emit('{} CONNECTED read-only admin {}'.format(utc_now(), socket_url))
                    while time.monotonic() < end:
                        now = time.monotonic()
                        if now >= next_table:
                            emit(recorder.table())
                            next_table = now + interval
                        try:
                            message = await socket.receive(timeout=max(.001, min(end, next_table) - now))
                        except asyncio.TimeoutError:
                            continue
                        if message.type == aiohttp.WSMsgType.TEXT:
                            for line in recorder.ingest(json.loads(message.data)):
                                emit(line)
                        elif message.type == aiohttp.WSMsgType.BINARY:
                            recorder.frame(message.data)
                        elif message.type in (aiohttp.WSMsgType.CLOSE, aiohttp.WSMsgType.CLOSED, aiohttp.WSMsgType.ERROR):
                            break
            except (aiohttp.ClientError, OSError, ValueError) as exc:
                emit('{} DISCONNECTED {}'.format(utc_now(), exc))
            if time.monotonic() < end:
                await asyncio.sleep(min(1., end - time.monotonic()))
        if recorder.state is not None:
            emit(recorder.table())


@contextmanager
def open_recordings(record=None, poses=None, emit=print):
    """Reserve both outputs before announcing them; preserve previous evidence."""
    tag = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    logs = Path(__file__).resolve().parents[1] / 'logs'
    streams, created = [], []
    committed = False
    try:
        for value, suffix in ((record, '.jsonl'), (poses, '_poses.csv')):
            if value is None:
                streams.append(None)
                continue
            requested = Path(value) if value else logs / ('stage_' + tag + suffix)
            requested.parent.mkdir(parents=True, exist_ok=True)
            candidate, counter = requested, 0
            while True:
                try:
                    stream = candidate.open('x', encoding='utf-8', newline='')
                    break
                except FileExistsError:
                    counter += 1
                    candidate = requested.with_name('{}{}_{}{}'.format(
                        requested.stem, '_' + tag, counter, requested.suffix))
            streams.append(stream)
            created.append(candidate)
        committed = True
        for path in created:
            emit('OUTPUT ' + str(path.resolve()))
        yield tuple(streams)
    finally:
        for stream in streams:
            if stream is not None:
                stream.close()
        if not committed:
            for path in created:
                path.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8080')
    parser.add_argument('--fleet', action='store_true', help='show all inventory rows, including spares')
    parser.add_argument('--record', nargs='?', const='', metavar='JSONL', help='record every hello/state')
    parser.add_argument('--dump-poses', nargs='?', const='', metavar='CSV', help='record all pose rows')
    parser.add_argument('--seconds', type=float, default=0, help='0 means until Ctrl+C')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be finite and nonnegative')
    try:
        with open_recordings(args.record, args.dump_poses,
                             emit=lambda value: print(value, flush=True)) as (record, poses):
            recorder = ProbeRecorder(record, poses, args.fleet)
            asyncio.run(collect(args.url, recorder, seconds=args.seconds,
                                emit=lambda value: print(value, flush=True)))
    except KeyboardInterrupt:
        print('Stopped read-only recording.', flush=True)
    except OSError as exc:
        parser.exit(2, 'Cannot use recording outputs: {}\n'.format(exc))


if __name__ == '__main__':
    main()
