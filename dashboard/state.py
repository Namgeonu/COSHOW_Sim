"""Thread-safe latest telemetry, aged only using local monotonic receipt time."""
from collections import Counter, deque
import copy
import math
import os
import shutil
import threading
import time


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


class TelemetryStore:
    def __init__(self, cfg, clock=time.monotonic):
        self.cfg, self.clock = cfg, clock
        self.lock = threading.RLock()
        self.data = {name: {} for name in cfg.robots}
        self.detected = {name: {} for name in cfg.robots}
        self.frames = {}
        self.frame_serial = 0
        self._mission = self._preflight = self._ready = None
        self.events = deque(maxlen=200)
        self.run = dict(state='IDLE', since=clock(), preflight_pid=None, bt_pid=None,
                        external_bt=False, last_error=None)
        self.stack = dict(crazyflie_server='down', aideck='down', roster_hash=None,
                          applied_hash=None, radios=len(cfg.radio_counts), radio_counts=cfg.radio_counts)
        self.context = dict(interface_errors=[], nodes=[], processes={n: dict(alive=False, exited_at=None)
                                                for n in ('bt', 'preflight')},
                            receipts={n: deque(maxlen=100) for n in ('mission', 'preflight_status', 'preflight_ready')},
                            orphans=[], env={**os.environ, **cfg.raw.get('commands', {}).get('env', {})},
                            ping_available=shutil.which('ping') is not None)

    def receive(self, name, channel, value):
        with self.lock:
            if name not in self.data:
                return
            now = self.clock()
            if channel == 'frame':
                self.frame_serial += 1
                self.frames[name] = (self.frame_serial, bytes(value))
                self.data[name][channel] = (True, now)
            elif channel == 'detections':
                hold = self.cfg.raw.get('freshness_s', {}).get('detection_hold', .5)
                self.detected[name] = {key: at for key, at in self.detected[name].items() if now - at < hold}
                for marker in value:
                    self.detected[name][int(marker)] = now
            else:
                self.data[name][channel] = (copy.deepcopy(json_safe(value)), now)

    def mission(self, value):
        with self.lock:
            if not isinstance(value, dict):
                return
            now = self.clock()
            self._mission = (copy.deepcopy(json_safe(value)), now)
            self.context['receipts']['mission'].append(now)

    def preflight(self, value):
        with self.lock:
            if not isinstance(value, dict):
                return
            now = self.clock()
            self._preflight = (copy.deepcopy(json_safe(value)), now)
            self.context['receipts']['preflight_status'].append(now)

    def ready(self, value):
        with self.lock:
            now = self.clock()
            self._ready = (bool(value), now)
            self.context['receipts']['preflight_ready'].append(now)

    def nodes(self, value):
        with self.lock:
            self.context['nodes'] = list(value)
            names = Counter(('/' + ns.strip('/') + '/' + n).replace('//', '/')
                            for n, ns in self.context['nodes'])
            for key, field in (('server', 'crazyflie_server'), ('aideck', 'aideck')):
                target = '/' + self.cfg.raw.get('nodes', {}).get(key, '').strip('/')
                owned = self.context.get('stack_processes', {}).get(field, {})
                alive = bool(owned.get('alive'))
                expected = max(int(alive), owned.get('expected_nodes', 0))
                self.stack[field] = 'external' if names[target] > expected else ('up' if alive else 'down')

    def ping(self, name, value):
        self.receive(name, 'ping', value)

    def unavailable(self, name, channel, reason):
        with self.lock:
            if channel == 'interface' and isinstance(reason, dict):
                if reason not in self.context['interface_errors']:
                    self.context['interface_errors'].append(copy.deepcopy(reason))
            self.event('warning', str(name) + ' ' + channel + ': ' + str(reason))

    def set_run(self, **changes):
        with self.lock:
            if 'state' in changes and changes['state'] != self.run['state']:
                changes.setdefault('since', self.clock())
            self.run.update(copy.deepcopy(changes))

    def set_stack(self, **changes):
        with self.lock:
            self.stack.update(copy.deepcopy(changes))

    def event(self, level, text):
        with self.lock:
            # Avoid drowning the bounded event window with identical graph failures.
            if self.events and self.events[-1]['text'] == text and not text.startswith('cmd:'):
                return
            self.events.append(dict(t=time.time(), level=level, text=text))

    def reset_cached(self):
        with self.lock:
            self._mission = self._preflight = self._ready = None
            self.detected = {name: {} for name in self.cfg.robots}

    def snapshot(self):
        with self.lock:
            now = self.clock()
            def aged(item):
                return dict(item[0], age=max(0, now - item[1])) if item else None
            mission, preflight = aged(self._mission), aged(self._preflight)
            if self._ready is not None:
                if preflight is None:
                    preflight = dict(ready=self._ready[0], age=max(0, now - self._ready[1]))
                else:
                    preflight['ready'] = self._ready[0]
            robots = {}
            for name, meta in self.cfg.robots.items():
                channels = self.data[name]
                def value(key, default=None):
                    return channels[key][0] if key in channels else default
                def age(key):
                    return max(0, now - channels[key][1]) if key in channels else None
                robot = {k: meta.get(k) for k in ('kind', 'role', 'fleet_id')}
                robot.update(pose=value('pose'), pose_age=age('pose'), battery_v=None, rssi=None,
                             ping=value('ping', dict(ip=meta.get('ip'), ok=None, rtt_ms=None)))
                if meta['kind'] == 'drone':
                    robot.update(status_age=age('status'), armed=None, can_fly=None, tumbled=None, low_power=None)
                    robot.update(value('status', {}))
                    robot.update(led=(mission or {}).get('led', {}).get(name, 'off'),
                                 cmd=(mission or {}).get('cmd', {}).get(name),
                                 camera=dict(fps=value('camera_fps'), stream_ok=value('camera_ok'), frame_age=age('frame')),
                                 detections=sorted(key for key, at in self.detected[name].items()
                                                   if now - at < self.cfg.raw.get('freshness_s', {}).get('detection_hold', .5)))
                else:
                    robot.update(value('limo_status', {}))
                    robot['nav_ready'] = value('nav_ready')
                robots[name] = robot
            run = dict(self.run, elapsed_s=max(0, now - self.run['since']))
            result = dict(type='state', t=time.time(), run=run, stack=self.stack,
                          mission=mission, preflight=preflight, robots=robots,
                          checklist=[], events=list(self.events)[-30:])
            return copy.deepcopy(result)

    def latest_frames(self):
        with self.lock:
            return dict(self.frames)

    def checklist_context(self):
        with self.lock:
            return copy.deepcopy(self.context)
