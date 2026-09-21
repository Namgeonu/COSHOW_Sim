"""Persistent role assignments, generated configuration and owned stack children.

Only generated files are written. Operator templates and BT coordinates remain
their authoritative inputs. This module never opens radios or camera sockets.
"""
import asyncio
from contextlib import asynccontextmanager
from dataclasses import dataclass
import copy
import hashlib
import inspect
import json
import os
import math
from pathlib import Path
import re
import shlex
import shutil

import yaml

from dashboard.config import HERE, load_config, read_yaml
from dashboard.runner import command_argv


@dataclass
class GeneratedConfig:
    files: dict
    roster_hash: str
    preflight_argv: list
    radio_counts: dict
    warnings: list


_MISSING = object()


def _inventory(cfg):
    rows = cfg.raw.get('fleet', {}).get('drones', [])
    if not isinstance(rows, list) or not rows:
        raise ValueError('플릿 드론 인벤토리가 없습니다')
    by_id = {}
    for row in rows:
        if not isinstance(row, dict) or not isinstance(row.get('id'), str) or not row['id']:
            raise ValueError('플릿 기체 id 형식 오류')
        if row['id'] in by_id:
            raise ValueError('플릿 기체 id 중복: ' + row['id'])
        by_id[row['id']] = row
    return by_id


def _radio(uri, label):
    if not isinstance(uri, str):
        raise ValueError(label + ': radio URI 미설정 또는 형식 오류')
    match = re.fullmatch(r'radio://(\d+)/(\d+)/(250K|1M|2M)/([0-9A-Fa-f]{10})', uri or '')
    if not match or not 0 <= int(match.group(2)) <= 125:
        raise ValueError(label + ': radio URI 미설정 또는 형식 오류')
    return match.group(1)


def _ip(row):
    # A role's old network fallback may belong to the replaced physical camera.
    value = row.get('aideck_ip')
    if not isinstance(value, str) or not value or any(char in value for char in ',\r\n'):
        raise ValueError(row['id'] + ': 역할 카메라 IP 미설정 또는 형식 오류')
    return value


def validate_roster(cfg, roster):
    by_id = _inventory(cfg)
    if not isinstance(roster, dict) or set(roster) != set(cfg.drones):
        raise ValueError('로스터는 모든 드론 역할을 정확히 한 번 배정해야 합니다')
    if any(not isinstance(value, str) for value in roster.values()):
        raise ValueError('로스터 물리 기체 id 형식 오류')
    if len(set(roster.values())) != len(roster):
        raise ValueError('로스터에 동일 물리 기체를 중복 배정할 수 없습니다')
    for role, physical in roster.items():
        if physical not in by_id:
            raise ValueError('인벤토리에 없는 물리 기체: ' + physical)
        if not isinstance(role, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', role):
            raise ValueError('역할 이름 형식 오류: ' + str(role))
        _radio(by_id[physical].get('uri'), physical)
        _ip(by_id[physical])
    return by_id


def _number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def _bases(cfg):
    bases = []
    for role in cfg.drones:
        base = cfg.bt.get('coshow', {}).get('drones', {}).get(role, {}).get('base')
        if not isinstance(base, (list, tuple)) or len(base) != 2 or not all(_number(n) for n in base):
            raise ValueError(role + ': BT base 설정 없음 또는 형식 오류')
        bases.append([float(base[0]), float(base[1])])
    return bases


def _grid(area, count):
    if not count:
        return []
    pitch = area.get('pitch')
    if not _number(pitch) or pitch <= 0:
        raise ValueError('스페어 격자 pitch 설정 오류')
    ranges = []
    for axis in ('x', 'y'):
        bounds = area.get(axis)
        if (not isinstance(bounds, (list, tuple)) or len(bounds) != 2
                or not all(_number(n) for n in bounds) or bounds[0] > bounds[1]):
            raise ValueError('스페어 격자 범위 설정 오류')
        ranges.append([round(bounds[0] + i * pitch, 9)
                       for i in range(int(math.floor((bounds[1] - bounds[0]) / pitch + 1e-9)) + 1)])
    points = [[x, y, 0.0] for y in ranges[1] for x in ranges[0]]
    if len(points) < count:
        raise ValueError('스페어 격자 공간이 부족합니다')
    return points[:count]


def generate_config(cfg, roster=None, run_dir=None):
    roster = cfg.roster if roster is None else roster
    by_id = validate_roster(cfg, roster)
    bases = _bases(cfg)
    template_name = cfg.raw.get('crazyflies_template')
    if not template_name:
        raise ValueError('crazyflies_template 설정 없음')
    template = read_yaml(cfg.resolve(template_name))
    robot_types = template.get('robot_types')
    if not isinstance(robot_types, dict):
        raise ValueError('crazyflies_template의 robot_types 설정 없음')
    warnings = []
    fallback_type = cfg.raw.get('fleet', {}).get('robot_type', cfg.raw.get('robot_type'))
    if ('robot_type' in cfg.raw and 'robot_type' in cfg.raw.get('fleet', {})
            and cfg.raw['robot_type'] != fallback_type):
        warnings.append('robot_type과 fleet.robot_type이 달라 fleet.robot_type을 사용합니다')

    def selected_type(role=None):
        value = template.get('robots', {}).get(role, {}).get('type', fallback_type) if role else fallback_type
        if not isinstance(value, str) or value not in robot_types:
            raise ValueError('robot_types에 선택 기체 타입 없음: ' + str(value))
        return value
    camera_template = cfg.raw.get('aideck_template')
    if not isinstance(camera_template, str) or not camera_template.strip():
        raise ValueError('aideck_template 설정 없음')
    camera = read_yaml(cfg.resolve(camera_template))
    camera_keys = [key for key, value in camera.items()
                   if isinstance(value, dict) and isinstance(value.get('ros__parameters'), dict)
                   and 'drones' in value['ros__parameters']]
    if len(camera_keys) != 1:
        raise ValueError('카메라 템플릿에서 drones 파라미터 블록을 하나만 찾을 수 있어야 합니다')
    maximum = cfg.raw.get('fleet', {}).get('max_per_radio')
    if not isinstance(maximum, int) or isinstance(maximum, bool) or maximum < 1:
        raise ValueError('fleet.max_per_radio 설정 오류')
    counts, uris = {}, set()
    for physical, row in by_id.items():
        uri = row.get('uri')
        if (uri is None or isinstance(uri, str) and not uri.strip()) and physical not in roster.values():
            warnings.append(physical + ': radio URI 미설정 · 스페어 생성에서 제외')
            continue
        radio = _radio(uri, physical)
        if uri.lower() in uris:
            raise ValueError('물리 기체 radio URI 중복: ' + uri)
        uris.add(uri.lower())
        counts[radio] = counts.get(radio, 0) + 1
        if counts[radio] > maximum:
            raise ValueError('라디오당 기체 수 초과: ' + radio)
    robots = {}
    for role, base in zip(cfg.drones, bases):
        robots[role] = dict(enabled=True, uri=by_id[roster[role]]['uri'],
                            initial_position=base + [0.0], type=selected_type(role))
    unassigned = [physical for physical in by_id
                  if physical not in roster.values() and isinstance(by_id[physical].get('uri'), str)
                  and by_id[physical]['uri'].strip()]
    prefix = cfg.raw.get('spare_prefix')
    if not isinstance(prefix, str) or not prefix:
        raise ValueError('스페어 이름 접두사 설정 없음')
    for physical, position in zip(unassigned, _grid(cfg.field.get('spare_area', {}), len(unassigned))):
        name = prefix + physical
        if name in robots:
            raise ValueError('스페어 이름과 역할 이름 중복: ' + name)
        robots[name] = dict(enabled=True, uri=by_id[physical]['uri'], initial_position=position, type=selected_type())
    template['robots'] = robots
    camera[camera_keys[0]]['ros__parameters']['drones'] = [
        '{},{},{}'.format(role, _ip(by_id[roster[role]]), 5001 + index)
        for index, role in enumerate(cfg.drones)]
    values = dict(roles='[' + ','.join(cfg.drones) + ']',
                  expected_x='[' + ','.join(str(base[0]) for base in bases) + ']',
                  expected_y='[' + ','.join(str(base[1]) for base in bases) + ']')
    argv = command_argv(cfg, 'preflight', values)
    data = {'crazyflies.generated.yaml': template, 'drones.generated.yaml': camera,
            'preflight.generated.yaml': {'argv': argv}}
    files = {name: yaml.safe_dump(value, allow_unicode=True, sort_keys=False).encode('utf-8')
             for name, value in data.items()}
    digest = hashlib.sha256(b''.join(files[name] for name in sorted(files))).hexdigest()
    return GeneratedConfig(files, digest, argv, counts, warnings)


def _atomic_write(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    try:
        with temporary.open('wb') as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class FleetManager:
    def __init__(self, cfg, store, on_reconfigure=None, run_dir=None, operation_guard=None):
        self.cfg, self.store, self.on_reconfigure = cfg, store, on_reconfigure
        self.operation_guard = operation_guard
        self._run_dir_override = run_dir
        self.run_dir = self.roster_path = self.log_dir = None
        self._restore_mock_roster = cfg.mock
        self.children = {}
        self.lock = asyncio.Lock()
        self.generated = None
        self._generation_error = None
        self._closing = False
        self._retired = {kind: [] for kind in ('crazyflie_server', 'aideck')}
        self._recorded_exits = set()
        self._recovery_watchers = set()
        self._unowned = {}
        self._recovered = set()
        self.store.context.setdefault('stack_processes', {})
        self.store.context.setdefault('stack_orphans', [])
        if self.store.run['state'] == 'IDLE':
            self.regenerate()

    def _configure_paths(self):
        if not self.cfg.dashboard_ok:
            raise ValueError('대시보드 설정을 읽을 수 없어 플릿 파일을 생성할 수 없습니다')
        if self.cfg.mock:
            run_dir = Path(self._run_dir_override or HERE / 'run/mock').resolve()
            self.roster_path = run_dir / 'roster.yaml'
        else:
            value = self.cfg.raw.get('roster_file', 'run/roster.yaml')
            if not isinstance(value, str) or not value.strip():
                raise ValueError('roster_file: 비어 있지 않은 파일 경로가 필요합니다')
            self.roster_path = self.cfg.resolve(value)
            run_dir = Path(self._run_dir_override or self.roster_path.parent).resolve()
        if self.run_dir != run_dir:
            self.log_dir = run_dir.parent / 'logs'
        self.run_dir = run_dir

    def _idle(self):
        if self.store.run['state'] != 'IDLE':
            raise ValueError('플릿 변경과 스택 기동은 IDLE 상태에서만 허용됩니다')

    @asynccontextmanager
    async def _operation(self):
        async with self.lock:
            self._idle()
            if self._closing:
                raise ValueError('서버 종료 중입니다')
            if self.operation_guard is not None:
                response = self.operation_guard()
                if inspect.isawaitable(response):
                    await response
            self.store.set_stack(busy=True)
            try:
                yield
            finally:
                self.store.set_stack(busy=False)

    def regenerate(self):
        self._idle()
        if self._generation_error in self.cfg.errors:
            self.cfg.errors.remove(self._generation_error)
        try:
            self._configure_paths()
            if not self.roster_path.exists():
                _atomic_write(self.roster_path, yaml.safe_dump(self.cfg.roster, sort_keys=False).encode('utf-8'))
            elif self._restore_mock_roster:
                roster = read_yaml(self.roster_path)
                validate_roster(self.cfg, roster)
                self._apply_config(load_config(self.cfg.path, self.cfg.field_path, mock=True, roster_override=roster))
            self._restore_mock_roster = False
            generated = generate_config(self.cfg, run_dir=self.run_dir)
            for name, content in generated.files.items():
                _atomic_write(self.run_dir / name, content)
        except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError, yaml.YAMLError) as exc:
            self.generated = None
            self._generation_error = '플릿 생성: ' + str(exc)
            self.cfg.errors.append(self._generation_error)
            self.store.set_stack(roster_hash=None, generation_error=str(exc), radios=0, radio_counts={})
            self.store.event('warning', self._generation_error)
            return None
        self.generated = generated
        for warning in generated.warnings:
            if warning not in self.cfg.warnings:
                self.cfg.warnings.append(warning)
            self.store.event('warning', warning)
        self._generation_error = None
        self.store.set_stack(roster_hash=generated.roster_hash, generation_error=None,
                             radios=len(generated.radio_counts), radio_counts=generated.radio_counts)
        return generated

    async def save_roster(self, roster, expected_hash=_MISSING):
        async with self._operation():
            if expected_hash is _MISSING:
                raise ValueError('expected_hash 필수: 최신 로스터를 확인한 뒤 다시 저장하세요')
            if expected_hash != (self.generated.roster_hash if self.generated else None):
                raise ValueError('로스터가 다른 관리자 화면에서 변경되었습니다. 최신 배정을 다시 검토하세요')
            validate_roster(self.cfg, roster)
            self._configure_paths()
            fresh = load_config(self.cfg.path, self.cfg.field_path, mock=self.cfg.mock,
                                roster_override=roster)
            generated = generate_config(fresh, run_dir=self.run_dir)
            contents = {self.roster_path: yaml.safe_dump(dict(roster), sort_keys=False).encode('utf-8')}
            contents.update({self.run_dir / name: content for name, content in generated.files.items()})
            original_files = {path: path.read_bytes() if path.exists() else None for path in contents}
            old_cfg = copy.deepcopy(self.cfg.__dict__)
            old_generated, old_error = self.generated, self._generation_error
            old_restore = self._restore_mock_roster
            with self.store.lock:
                old_store = {name: copy.deepcopy(getattr(self.store, name)) for name in
                             ('data', 'detected', 'frames', '_mission', '_preflight', '_ready', 'stack')}
            try:
                for path, content in contents.items():
                    _atomic_write(path, content)
                self._restore_mock_roster = False
                self._apply_config(fresh)
                self.generated, self._generation_error = generated, None
                self.store.set_stack(roster_hash=generated.roster_hash, generation_error=None,
                                     radios=len(generated.radio_counts), radio_counts=generated.radio_counts)
                if self.on_reconfigure is not None:
                    response = self.on_reconfigure(self.cfg)
                    if inspect.isawaitable(response):
                        await response
            except BaseException:
                # The adapter callback retains its old live adapter on startup failure.
                # Restore its shared Config and the exact prior disk/store snapshot.
                with self.store.lock:
                    self.cfg.__dict__.clear()
                    self.cfg.__dict__.update(old_cfg)
                    self.store.cfg = self.cfg
                    for name, value in old_store.items():
                        setattr(self.store, name, value)
                self.generated, self._generation_error = old_generated, old_error
                self._restore_mock_roster = old_restore
                for path, content in original_files.items():
                    if content is None:
                        path.unlink(missing_ok=True)
                    elif not path.exists() or path.read_bytes() != content:
                        _atomic_write(path, content)
                raise
            for warning in generated.warnings:
                if warning not in self.cfg.warnings:
                    self.cfg.warnings.append(warning)
                self.store.event('warning', warning)
            self.store.event('info', '로스터 저장 완료 · 스택 재기동 필요')
            return dict(self.cfg.roster)

    def _apply_config(self, fresh):
        with self.store.lock:
            self.cfg.__dict__.clear()
            self.cfg.__dict__.update(fresh.__dict__)
            self.store.cfg = self.cfg
            self.store.data = {name: {} for name in self.cfg.robots}
            self.store.detected = {name: {} for name in self.cfg.robots}
            self.store.frames.clear()
            self.store.reset_cached()

    def recommend_roster(self):
        snapshot = self.store.snapshot()
        freshness = self.cfg.raw.get('freshness_s', {}).get('status', 1.0)
        candidates = []
        inventory = _inventory(self.cfg)
        for name, robot in snapshot['robots'].items():
            physical = robot.get('fleet_id')
            if physical not in inventory or robot.get('kind') != 'drone':
                continue
            age, voltage = robot.get('status_age'), robot.get('battery_v')
            if not _number(age) or age >= freshness or not _number(voltage):
                continue
            try:
                _radio(inventory[physical].get('uri'), physical)
                _ip(inventory[physical])
            except ValueError:
                continue
            candidates.append((-voltage, physical))
        candidates.sort()
        if len(candidates) < len(self.cfg.drones):
            raise ValueError('링크와 배터리가 확인된 추천 기체가 부족합니다')
        return {role: physical for role, (_, physical) in zip(self.cfg.drones, candidates)}

    @staticmethod
    def _read_json(path):
        try:
            value = json.loads(path.read_text())
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    @staticmethod
    def _pid_alive(pid):
        if not isinstance(pid, int) or pid <= 1:
            return False
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        if Path('/proc').is_dir():
            try:
                stat = (Path('/proc') / str(pid) / 'stat').read_text()
                if stat[stat.rfind(')') + 2:].split()[0] == 'Z':
                    return False
            except (OSError, IndexError):
                pass
        return True

    @staticmethod
    def _stack_command_matches(actual, configured):
        from dashboard.runner import _same_command
        if _same_command(actual, configured):
            return True
        if not configured:
            return False
        executable = Path(shutil.which(configured[0]) or configured[0]).expanduser()
        try:
            with executable.open('rb') as source:
                first_line = source.readline(512).decode('utf-8')
            if not first_line.startswith('#!'):
                return False
            interpreter = shlex.split(first_line[2:].strip())
        except (OSError, ValueError, UnicodeError):
            return False
        # Console entrypoints such as ros2 expose the interpreter in /proc.
        # Only the executable's own shebang may account for those extra tokens.
        return bool(interpreter) and _same_command(actual, interpreter + [str(executable)] + configured[1:])

    async def _persist_process(self, kind, child):
        from dashboard.runner import _process_argv
        actual = await _process_argv(child.pid)
        if self._stack_command_matches(actual, child.argv):
            child.argv = actual
        value = dict(pid=child.pid, argv=child.argv, identity=child.identity)
        _atomic_write(self.run_dir / (kind + '.process.json'), json.dumps(value).encode('utf-8'))

    def _persist_applied(self, roster_hash):
        value = dict(applied_hash=roster_hash,
                     processes={kind: dict(pid=child.pid, argv=child.argv, identity=child.identity)
                                for kind, child in self.children.items() if child.returncode is None})
        _atomic_write(self.run_dir / 'stack.applied.json', json.dumps(value).encode('utf-8'))

    async def recover_stacks(self):
        """Re-adopt only verified stack identities; never send a startup signal."""
        if self.cfg.mock:
            return
        from dashboard.runner import (OwnedProcess, _OrphanProcess, _process_argv,
                                      _process_identity, _same_command, _restore_sigint)
        try:
            self._configure_paths()
            commands = self._commands(self.generated)
        except (OSError, ValueError, TypeError):
            commands = {}
        if self.run_dir is None:
            return
        recovered_any = False
        for kind in ('crazyflie_server', 'aideck'):
            child = self.children.get(kind)
            if child is not None and child.returncode is None:
                continue
            pid_path = self.run_dir / (kind + '.pid')
            try:
                pid = int(pid_path.read_text().strip())
            except FileNotFoundError:
                self._unowned.pop(kind, None)
                continue
            except (OSError, ValueError):
                self._unowned[kind] = None
                self.store.event('warning', kind + ': pidfile 확인 불가 · 외부 스택으로 취급')
                continue
            if not self._pid_alive(pid):
                pid_path.unlink(missing_ok=True)
                self._unowned.pop(kind, None)
                continue
            argv, identity = await _process_argv(pid), await _process_identity(pid)
            receipt = self._read_json(self.run_dir / (kind + '.process.json'))
            verified = (identity is not None and receipt.get('pid') == pid
                        and receipt.get('identity') == identity
                        and isinstance(receipt.get('argv'), list)
                        and all(isinstance(part, str) for part in receipt['argv'])
                        and _same_command(argv, receipt['argv'])
                        and self._stack_command_matches(argv, commands.get(kind)))
            if not verified:
                self._unowned[kind] = pid
                self.store.event('warning', kind + ': 살아 있는 pid의 명령행/신원 미확인 · 외부 스택으로 취급')
                continue
            proc = _OrphanProcess(pid, argv, identity)
            child = OwnedProcess(proc, argv, self.log_dir / (kind + '.log'), pid_path, identity=identity)
            _restore_sigint(child)
            self.children[kind] = child
            self._unowned.pop(kind, None)
            self._recovered.add(kind)
            task = asyncio.create_task(proc.wait())
            self._recovery_watchers.add(task)
            task.add_done_callback(self._recovery_watchers.discard)
            recovered_any = True
            self.store.event('info', '{} 스택 재입양 pid={} · 종료 신호 없음'.format(kind, pid))
        if recovered_any:
            receipt = self._read_json(self.run_dir / 'stack.applied.json')
            identities = receipt.get('processes', {})
            valid = (isinstance(identities, dict)
                     and all(kind in self.children and self.children[kind].returncode is None
                             and identities.get(kind) == dict(pid=self.children[kind].pid, argv=self.children[kind].argv,
                                                              identity=self.children[kind].identity)
                             for kind in ('crazyflie_server', 'aideck')))
            value = receipt.get('applied_hash') if valid else None
            self.store.set_stack(applied_hash=value if isinstance(value, str)
                                 and re.fullmatch(r'[0-9a-f]{64}', value) else None)
            if self.store.stack['applied_hash'] is None:
                self.store.event('warning', '재입양 스택의 적용 해시 확인 불가 · 스택 재기동 필요')
        self.refresh()

    async def detach_monitors(self):
        """Detach backend polling only; ownership receipts and stack PIDs survive."""
        tasks = tuple(self._recovery_watchers)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self._recovery_watchers.clear()

    def _external(self, kind):
        if kind in self._unowned:
            pid = self._unowned[kind]
            if pid is None or self._pid_alive(pid):
                return True
            self._unowned.pop(kind, None)
        key = 'server' if kind == 'crazyflie_server' else kind
        target = '/' + self.cfg.raw.get('nodes', {}).get(key, '').strip('/')
        count = sum(('/' + namespace.strip('/') + '/' + name).replace('//', '/') == target
                    for name, namespace in self.store.context.get('nodes', []))
        child = self.children.get(kind)
        alive = child is not None and child.returncode is None
        grace = self.cfg.raw.get('external_node_grace_s', 20.0)
        self._retired[kind] = [at for at in self._retired[kind] if self.store.clock() - at < grace]
        return count > int(alive) + len(self._retired[kind])

    def refresh(self):
        with self.store.lock:
            for kind in ('crazyflie_server', 'aideck'):
                child = self.children.get(kind)
                alive = child is not None and child.returncode is None
                if child is not None and not alive and child.pid not in self._recorded_exits:
                    self._recorded_exits.add(child.pid)
                    self._retired[kind].append(self.store.clock())
                    self.store.stack['applied_hash'] = None
                    self.store.event('info' if child.sigint_sent else 'error',
                                     '{} 프로세스 종료 rc={}'.format(kind, child.returncode))
                external = self._external(kind)
                self.store.context['stack_processes'][kind] = dict(
                    alive=alive, pid=child.pid if alive else None,
                    expected_nodes=int(alive) + len(self._retired[kind]),
                    exited_at=self._retired[kind][-1] if self._retired[kind] else None)
                self.store.stack[kind] = 'external' if external else ('up' if alive else 'down')
            self.store.context['stack_orphans'] = [dict(name=kind, pid=pid)
                                                   for kind, pid in self._unowned.items()]
            self.store.context['stack_recovered'] = sorted(kind for kind in self._recovered
                                                         if self.children.get(kind) is not None
                                                         and self.children[kind].returncode is None)

    def _commands(self, generated):
        values = dict(crazyflies_yaml=str(self.run_dir / 'crazyflies.generated.yaml'),
                      drones_yaml=str(self.run_dir / 'drones.generated.yaml'))
        return {kind: command_argv(self.cfg, kind, values)
                for kind in ('crazyflie_server', 'aideck')}

    async def _start(self):
        if self.cfg.mock:
            raise ValueError('MOCK 모드에서는 실제 스택 프로세스를 기동하지 않습니다')
        from dashboard.runner import spawn_process
        await self.recover_stacks()
        self.refresh()
        if any(self.store.stack.get(kind) in ('up', 'external') for kind in ('crazyflie_server', 'aideck')):
            raise ValueError('이미 실행 중이거나 외부에서 기동한 스택이 있습니다')
        generated = self.regenerate()
        if generated is None:
            raise ValueError(self.store.stack['generation_error'])
        commands = self._commands(generated)
        started = []

        async def spawn_owned(kind, argv):
            try:
                child = await spawn_process(argv, self.cfg.bt_cwd.parent,
                                            self.cfg.raw.get('commands', {}).get('env', {}),
                                            self.log_dir / (kind + '.log'), self.run_dir / (kind + '.pid'),
                                            preserve_on_error=True)
            except BaseException as exc:
                child = getattr(exc, 'child', None)
                if child is not None:
                    self.children[kind] = child
                    started.append(child)
                    try:
                        await self._persist_process(kind, child)
                    except OSError as receipt_error:
                        self.store.event('warning', '스택 신원 영수증 저장 실패: ' + repr(receipt_error))
                raise
            self.children[kind] = child
            started.append(child)
            await self._persist_process(kind, child)

        try:
            for kind, argv in commands.items():
                # Cancellation cannot interrupt the interval between spawn and
                # ownership recording. Finish that single spawn, then propagate.
                task = asyncio.create_task(spawn_owned(kind, argv))
                try:
                    await asyncio.shield(task)
                except asyncio.CancelledError:
                    while not task.done():
                        try:
                            await asyncio.shield(task)
                        except asyncio.CancelledError:
                            continue
                        except BaseException:
                            break
                    if not task.cancelled():
                        task.exception()  # Retrieve spawn failure without losing the caller's cancellation.
                    raise
            await asyncio.sleep(.05)
            if any(child.returncode is not None for child in started):
                raise ValueError('스택 프로세스가 기동 직후 종료되었습니다')
            self._persist_applied(generated.roster_hash)
        except BaseException as exc:
            # A partially started stack remains connected until an explicit
            # operator stop/restart. Unknown application state must never be green.
            self.store.set_stack(applied_hash=None)
            try:
                self._persist_applied(None)
            except OSError as receipt_error:
                self.store.event('warning', '스택 적용 영수증 저장 실패: ' + repr(receipt_error))
            self.refresh()
            self.store.event('error', '스택 기동 실패: {} · {} · 실행된 스택 유지 · 스택 재기동 필요'.format(repr(exc), str(exc)))
            raise
        self.store.set_stack(applied_hash=generated.roster_hash)
        self.refresh()
        self.store.event('info', '스택 기동 완료')
        return dict(self.store.stack)

    async def start_stack(self):
        async with self._operation():
            return await self._start()

    async def _stop_owned(self):
        """Shared by explicit IDLE stop/restart only; shutdown never calls here."""
        if self.cfg.mock:
            raise ValueError('MOCK 모드에서는 실제 스택 프로세스를 정지하지 않습니다')
        from dashboard.runner import stop_process, _process_argv, _process_identity, _same_command
        await self.recover_stacks()
        self.refresh()
        if any(self.store.stack.get(kind) == 'external' for kind in ('crazyflie_server', 'aideck')):
            raise ValueError('외부에서 기동한 스택은 소유 터미널에서 종료하세요')
        for kind, child in self.children.items():
            if child.returncode is None and (await _process_identity(child.pid) != child.identity
                    or not _same_command(await _process_argv(child.pid), child.argv)):
                self._unowned[kind] = child.pid
                self.refresh()
                raise ValueError('외부 스택: 정지 전 프로세스 신원이 변경되었습니다')
        results = await asyncio.gather(*(stop_process(child, timeout_s=10, process_group=True)
                                         for child in self.children.values()), return_exceptions=True)
        self.refresh()
        errors = [result for result in results if isinstance(result, BaseException)]
        if errors:
            raise errors[0]
        await self.detach_monitors()
        self.children.clear()
        self.store.set_stack(applied_hash=None)
        self._persist_applied(None)
        self.refresh()

    async def stop_stack(self):
        async with self._operation():
            await self._stop_owned()
            self.store.event('info', '스택 정지 완료 · 관리자 명시적 정지')
            return dict(self.store.stack)

    async def restart_stack(self):
        async with self._operation():
            if self.cfg.mock:
                raise ValueError('MOCK 모드에서는 실제 스택 프로세스를 재기동하지 않습니다')
            # Validate every generated byte and command before stopping a live stack.
            generated = generate_config(self.cfg, run_dir=self.run_dir)
            self._commands(generated)
            await self._stop_owned()
            return await self._start()

    async def close(self):
        """Detach only after Runner's emergency sequence; stacks keep running."""
        if self._closing:
            return
        self._closing = True
        async with self.lock:
            self.refresh()
            if any(child.returncode is None for child in self.children.values()):
                self.store.event('info', '스택은 계속 실행 중 — 다음 기동에서 재입양')
            await self.detach_monitors()
            self.children.clear()
