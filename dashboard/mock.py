"""Shared deterministic rehearsal timeline for local mode and the ROS fixture."""
import math
import time
from pathlib import Path


def lerp(start, end, fraction):
    f = max(0.0, min(1.0, fraction))
    return [a + (b - a) * f for a, b in zip(start, end)]


def scenario(t, cfg, run_state='RUNNING', checking_s=0.0, fail=False):
    """Pure state fragment. Timeline seconds are approximate, BT ordering is fixed."""
    spec = cfg.bt.get('coshow', {})
    static = cfg.hello(True)
    bases = static['bases']
    observer = spec.get('observe_drone', next(iter(cfg.drones), None))
    searchers = [n for n in spec.get('searchers', []) if n in cfg.drones]
    finder = next(iter(searchers), None)
    point = static['observe_point'] or [0, 0]
    marker_range = spec.get('mission_marker_ids', [11, 14])
    mission_id = min(marker_range[-1], marker_range[0] + 2)
    target_id = mission_id - spec.get('marker_id_offset', 10)
    targets = cfg.field.get('markers', {}).get('targets', {})
    target = targets.get(target_id, targets.get(str(target_id), point))
    pn = list(target)
    active = run_state in ('RUNNING', 'DONE')
    elapsed = max(0, t) if active else 0
    phase = next((name for edge, name in ((20, 'observe'), (29, 'handover'),
                 (45, 'search'), (50, 'capture'), (58, 'rescue_dispatch'),
                 (68, 'rescue'), (90, 'return')) if elapsed < edge), 'done')
    ready = run_state in ('READY', 'RUNNING', 'DONE') or (run_state == 'CHECKING' and checking_s >= 6 and not fail)
    stage = min(4, int(checking_s / 1.5) + 1) if run_state == 'CHECKING' else (4 if ready else 0)
    failed = fail and run_state == 'CHECKING' and checking_s >= 4.5
    if failed:
        stage = 3
    stages = [dict(name=name, result='pass' if ready or i < stage else 'running' if i == stage else 'pending')
              for i, name in enumerate(('server_ready', 'reset_estimators', 'gate', 'arm'), 1)]
    if failed:
        stages[2]['result'] = 'fail'
    failure_drone = cfg.drones[min(2, len(cfg.drones) - 1)] if cfg.drones else None
    preflight = dict(stage=stage, stages=stages, ready=ready, abort_reason=None, drones={}, t=t)
    robots, led, cmd, progress = {}, {}, {}, {}
    spare_index = 0
    spare_names = [n for n, m in cfg.robots.items() if m['kind'] == 'drone' and m['role'] is None]
    for name, meta in cfg.robots.items():
        base = list(bases.get(name, [0, 0])) + [0.0]
        pos = list(base)
        z = spec.get('drones', {}).get(name, {}).get('altitudes', {}).get('search', spec.get('altitudes', {}).get('search', .8))
        if meta['kind'] == 'drone' and meta['role'] is None:
            area = cfg.field.get('spare_area', dict(x=[-3, -2.2], y=[-2, 2], pitch=.4))
            columns = max(1, int((area['x'][1] - area['x'][0]) / area['pitch']) + 1)
            pos = [area['x'][0] + spare_index % columns * area['pitch'],
                   area['y'][0] + spare_index // columns * area['pitch'], 0.0]
            spare_index += 1
        elif active and meta['kind'] == 'drone':
            if name == observer:
                altitude = spec.get('drones', {}).get(name, {}).get('altitudes', {}).get('observe', 1.0)
                at = list(point) + [altitude]
                if elapsed < 3:
                    pos[2] = altitude * elapsed / 3
                elif elapsed < 8:
                    pos = lerp(base[:2] + [altitude], at, (elapsed - 3) / 5)
                elif elapsed < 20:
                    pos = at
                elif elapsed < 25:
                    pos = lerp(at, base[:2] + [altitude], (elapsed - 20) / 5)
                elif elapsed < 29:
                    pos[2] = altitude * max(0, (28 - elapsed) / 3)
            elif name in searchers and elapsed >= 29:
                start = 29 + searchers.index(name) * spec.get('search', {}).get('stagger_sec', 3)
                lane = static['lanes'].get(name, [base, base])
                left, right = lane[0], lane[-1]
                distance = math.dist(left[:2], right[:2]) or 1
                travel = max(0, elapsed - start - 3) * .3 / distance
                f = 1 - abs(travel % 2 - 1)
                pos = lerp(left, right, f)
                pos[2] = z * max(0, min(1, (elapsed - start) / 3))
                if elapsed >= 45 and name == finder:
                    pos = pn + [z]
                    if elapsed >= 68:
                        pos = lerp(pos, base[:2] + [z], (elapsed - 68) / 10)
                        pos[2] = z * max(0, min(1, (88 - elapsed) / 10))
                elif elapsed >= 50:
                    pos = lerp(pos, base[:2] + [z], (elapsed - 50) / 5)
                    pos[2] = z * max(0, min(1, (58 - elapsed) / 3))
                progress[name] = int(travel) % max(1, len(lane))
        elif active and meta['kind'] == 'limo' and meta['role']:
            index = cfg.limos.index(name)
            if index == 0:
                pos = lerp(base, list(point) + [0], elapsed / 8) if elapsed < 20 else lerp(list(point) + [0], base, (elapsed - 20) / 8)
            elif index == 1 and elapsed >= 50:
                pos = lerp(base, pn + [0], (elapsed - 50) / 8) if elapsed < 68 else lerp(pn + [0], base, (elapsed - 68) / 20)
        if phase == 'done' and active and meta['role']:
            pos = base
        voltage = 4.1 - .3 * min(90, elapsed) / 90
        if meta['role'] is None and meta['kind'] == 'drone':
            voltage = 3.6 + .5 * (spare_index - 1) / max(1, len(spare_names) - 1)
        armed = bool(ready and meta['role'] and meta['kind'] == 'drone' and run_state != 'DONE')
        row = dict(pose=dict(x=pos[0], y=pos[1], z=pos[2], yaw=0.0),
                   battery_v=voltage if meta['kind'] == 'drone' else None,
                   rssi=65, armed=armed, can_fly=armed, tumbled=False, low_power=False,
                   ping=dict(ip=meta.get('ip'), ok=not (fail and name == (cfg.drones[-1] if cfg.drones else None)), rtt_ms=2.4),
                   camera=dict(fps=10.0, stream_ok=True), nav_ready=bool(meta['role']), detections=[])
        if name in spare_names:
            row['link_ok'] = name != spare_names[-1]
        if active and name == observer and 19.7 <= elapsed < 20.05:
            row['detections'] = [mission_id]
        if active and name == finder and (45 <= elapsed < 45.3 or 49.7 <= elapsed < 50.1):
            row['detections'] = [target_id]
        robots[name] = row
        if name in cfg.drones:
            led[name] = 'blue' if active and 20 <= elapsed < 68 else 'off'
            if name == finder and 50 <= elapsed < 68 and active:
                led[name] = 'red' if elapsed < 58 else 'green'
            cmd[name] = dict(kind='go_to', goal=pos, t=elapsed) if pos[2] > 0 else dict(kind='land', goal=[], t=elapsed)
            passed = ready or (stage >= 3 and not (failed and name == failure_drone))
            preflight['drones'][name] = dict(kal_ok=passed, pose_ok=passed, sup_ok=passed,
                sup_why='', armed=armed, can_fly=armed, battery_v=voltage,
                pose_err=[0.0, .31 if failed and name == failure_drone else 0.0, 0.0], kal_range=[0.0, 0.0, 0.0])
    mission = None
    if active:
        mission = dict(t=elapsed, phase=phase, mission_marker_id=mission_id if elapsed >= 20 else None,
            target_id=target_id if elapsed >= 20 else None, finder=finder if elapsed >= 45 else None,
            P_N=dict(x=pn[0], y=pn[1], z=spec.get('drones', {}).get(finder, {}).get('altitudes', {}).get('capture',
                     spec.get('altitudes', {}).get('capture', .8))) if elapsed >= 45 and finder else None, target_confirmed=elapsed >= 50,
            target_confirm_note='mock re-detection' if elapsed >= 50 else None,
            search_progress=progress, missing_pose=[], preflight_required=True, preflight_ready=True,
            cmd=cmd, led=led, rescue_done_t=68.0 if elapsed >= 68 else 0.0)
    return dict(robots=robots, mission=mission, preflight=preflight if run_state != 'IDLE' else None)


class MockWorld:
    def __init__(self, cfg, store, fail=False):
        self.cfg, self.store, self.fail = cfg, store, fail
        self.started = self.checking = None
        self.landing_until, self.after_landing = None, None
        self.land_sent_at = None
        self.last_json, self.signature = -math.inf, None
        from dashboard.ros_io import interface_specs
        specs, errors = interface_specs(cfg)
        self.channels = {(s['robot'], s['channel']) for s in specs if s['type'] is not None}
        for error in errors:
            store.unavailable(None, 'interface', error)
        root = Path(__file__).resolve().parent / 'static/mock'
        self.jpeg = {n: (root / 'camera.jpg').read_bytes() for n in cfg.drones}

    def command(self, cmd):
        with self.store.lock:
            return self._command(cmd)

    def _command(self, cmd):
        state = self.store.run['state']
        allowed = dict(IDLE=('preflight', 'reset'), CHECKING=('estop', 'reset'),
                       READY=('start', 'estop', 'reset'), RUNNING=('estop', 'reset'),
                       DONE=('estop', 'reset'), ABORTED=('reset',), LANDING=())
        if cmd not in allowed.get(state, ()):
            return False, state + '에서 허용되지 않음'
        now = self.store.clock()
        if cmd == 'preflight':
            self.checking = now
            self.store.set_run(state='CHECKING', preflight_pid=-1)
            self.store.context['processes']['preflight']['alive'] = True
        elif cmd == 'start':
            self.started = now
            self.store.set_run(state='RUNNING', bt_pid=-1)
            self.store.context['processes']['bt']['alive'] = True
        elif cmd in ('estop', 'reset') and state != 'IDLE':
            self.store.set_run(state='LANDING')
            self.store.event('info', '착륙 시퀀스 시작')
            self.land_sent_at = now + .5
            self.after_landing = 'ABORTED' if cmd == 'estop' else 'IDLE'
            self.landing_until = now + .5 + self.cfg.bt.get('coshow', {}).get('durations', {}).get('land', 8)
        if cmd == 'reset' and state == 'IDLE':
            self.store.reset_cached()
        if cmd == 'reset' and state == 'ABORTED':
            self.landing_until = now
        if cmd in ('estop', 'reset') and state != 'IDLE':
            for proc in self.store.context['processes'].values():
                proc.update(alive=False, exited_at=now)
        self.store.set_run(since=now)
        return True, 'mock'

    def tick(self):
        with self.store.lock:
            self._tick()

    def _tick(self):
        now = self.store.clock()
        run = self.store.run['state']
        if run == 'LANDING' and self.land_sent_at is not None and now >= self.land_sent_at:
            self.store.event('info', 'land 전송 {0}/{0} · MOCK'.format(len(self.cfg.drones)))
            self.land_sent_at = None
        if run == 'LANDING' and now >= self.landing_until:
            self.store.set_run(state=self.after_landing, preflight_pid=None, bt_pid=None, since=now)
            run = self.after_landing
            self.store.reset_cached()
        check_s = now - self.checking if self.checking is not None else 0
        elapsed = now - self.started if self.started is not None else 0
        if run == 'CHECKING' and check_s >= 6 and not self.fail:
            self.store.set_run(state='READY', since=now)
            run = 'READY'
        if run == 'RUNNING' and elapsed >= 90:
            self.store.set_run(state='DONE', since=now)
            run = 'DONE'
        self.apply(scenario(elapsed, self.cfg, run, check_s, self.fail), now)

    def apply(self, fragment, now):
        with self.store.lock:
            self._apply(fragment, now)

    def _apply(self, fragment, now):
        def receive(name, channel, value):
            if (name, channel) in self.channels:
                self.store.receive(name, channel, value)

        for name, row in fragment['robots'].items():
            meta = self.cfg.robots[name]
            receive(name, 'pose', row['pose'])
            self.store.ping(name, row['ping'])
            if meta['kind'] == 'drone':
                if row.get('link_ok', True):
                    receive(name, 'status', {k: row[k] for k in ('battery_v', 'rssi', 'armed', 'can_fly', 'tumbled', 'low_power')})
                if meta['role']:
                    receive(name, 'camera_fps', row['camera']['fps'])
                    receive(name, 'camera_ok', row['camera']['stream_ok'])
                    receive(name, 'frame', self.jpeg[name])
                    receive(name, 'detections', row['detections'])
            else:
                receive(name, 'nav_ready', row['nav_ready'])
        mission, preflight = fragment['mission'], fragment['preflight']
        signature = ((mission or {}).get('phase'), (preflight or {}).get('stage'), (preflight or {}).get('ready'))
        if signature != self.signature or now - self.last_json >= 1:
            if mission and (None, 'mission') in self.channels:
                self.store.mission(mission)
            if preflight:
                if (None, 'preflight') in self.channels:
                    self.store.preflight(preflight)
                if (None, 'ready') in self.channels:
                    self.store.ready(preflight['ready'])
            self.signature, self.last_json = signature, now
        self.store.set_stack(crazyflie_server='up', aideck='up', roster_hash='mock', applied_hash='mock')
        nodes = []
        for key, name in self.cfg.raw.get('nodes', {}).items():
            if key in ('server', 'aideck') or self.store.context['processes'].get(key, {}).get('alive'):
                parts = name.strip('/').split('/')
                nodes.append((parts[-1], '/' + '/'.join(parts[:-1]) if len(parts) > 1 else '/'))
        self.store.context['nodes'] = nodes
