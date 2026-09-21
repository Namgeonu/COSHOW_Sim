"""Owned process lifecycle and the ordered, server-owned emergency sequence.

No ROS types live here. Control calls cross the injected adapter boundary.
"""
import asyncio
from dataclasses import dataclass
import contextlib
import json
import math
import os
from pathlib import Path
import resource
import shlex
import shutil
import signal
import tempfile

HERE = Path(__file__).resolve().parent


def command_argv(cfg, name, substitutions=None):
    command = cfg.raw.get('commands', {}).get(name, '')
    if not isinstance(command, str) or not command.strip():
        raise ValueError('commands.{} 설정 없음'.format(name))
    tokens = shlex.split(command)
    if Path(tokens[0]).name in ('sh', 'bash', 'zsh', 'dash', 'fish', 'ksh'):
        raise ValueError('셸 래퍼는 허용하지 않음')
    if any(any(char in token for char in '|><;&$`\n') for token in tokens):
        raise ValueError('명령에 셸 문법을 사용할 수 없음')
    bases = (cfg.bt.get('coshow') or {}).get('drones') or {}
    replacements = {'roles': json.dumps(cfg.drones, separators=(',', ':'))}
    for axis, index in (('expected_x', 0), ('expected_y', 1)):
        if not any('{' + axis + '}' in token for token in tokens):
            continue
        values = []
        for role in cfg.drones:
            base = (bases.get(role) or {}).get('base')
            if not isinstance(base, (list, tuple)) or len(base) < 2 or not all(_number(value) for value in base[:2]):
                raise ValueError('{} base 좌표 설정 없음 또는 형식 오류'.format(role))
            values.append(base[index])
        replacements[axis] = json.dumps(values, separators=(',', ':'))
    replacements.update(substitutions or {})
    try:
        return [os.path.expanduser(token.format(**replacements)) for token in tokens]
    except (KeyError, ValueError) as exc:
        raise ValueError('명령 placeholder 오류: {}'.format(exc)) from exc


def _disable_core():
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


@dataclass
class OwnedProcess:
    proc: object
    argv: list
    log_path: Path
    pid_path: Path
    sigint_sent: bool = False
    orphan: bool = False
    identity: str = None

    @property
    def pid(self):
        return self.proc.pid

    @property
    def returncode(self):
        return self.proc.returncode


async def spawn_process(argv, cwd, env, log_path, pid_path, preserve_on_error=False):
    """Shared M5/M6 spawn boundary; callers supply argv, never a shell command."""
    log_path, pid_path = Path(log_path), Path(pid_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    pid_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open('ab', buffering=0) as output:
        process = await asyncio.create_subprocess_exec(*argv, cwd=str(Path(cwd).expanduser()),
            env={**os.environ, **env}, stdout=output, stderr=asyncio.subprocess.STDOUT,
            start_new_session=True, preexec_fn=_disable_core)
    child = OwnedProcess(process, list(argv), log_path, pid_path,
                         identity=await _process_identity(process.pid))
    try:
        pid_path.write_text(str(child.pid) + '\n', encoding='utf-8')
    except OSError as exc:
        if preserve_on_error:
            # A live radio stack must survive even a storage failure. Its
            # caller retains this handle and reports the missing PID receipt.
            exc.child = child
        else:
            await stop_process(child)
        raise
    return child


async def stop_process(child, timeout_s=10, process_group=True):
    """Stop an owned stack child. BT uses the explicit safety sequence instead."""
    if not isinstance(child, OwnedProcess) or child.orphan:
        raise ValueError('스택 종료는 직접 소유한 프로세스만 허용')
    def send(sig):
        if child.returncode is not None:
            return
        with contextlib.suppress(ProcessLookupError):
            if process_group:
                os.killpg(child.pid, sig)
            else:
                child.proc.send_signal(sig)
    if _remember_sigint(child):
        send(signal.SIGINT)
    try:
        await asyncio.wait_for(asyncio.shield(child.proc.wait()), timeout_s)
    except asyncio.TimeoutError:
        send(signal.SIGKILL)
        await child.proc.wait()
    _remove_pid(child)
    return child.returncode


def _remove_pid(child):
    with contextlib.suppress(OSError, ValueError):
        if int(child.pid_path.read_text().strip()) == child.pid:
            child.pid_path.unlink()
    receipt = _read_signal_receipt(child)
    if _receipt_matches(child, receipt):
        with contextlib.suppress(OSError):
            child.pid_path.with_suffix('.signal.json').unlink()


def _read_signal_receipt(child):
    try:
        value = json.loads(child.pid_path.with_suffix('.signal.json').read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def _receipt_matches(child, receipt):
    return (receipt.get('pid') == child.pid and receipt.get('identity') == child.identity
            and isinstance(receipt.get('argv'), list)
            and all(isinstance(part, str) for part in receipt['argv'])
            and _same_command(receipt['argv'], child.argv))


def _restore_sigint(child):
    receipt = _read_signal_receipt(child)
    child.sigint_sent = _receipt_matches(child, receipt) and receipt.get('sigint_sent') is True


def _remember_sigint(child):
    """Persist before signal delivery so restarting the backend cannot send twice."""
    if child.sigint_sent:
        return False
    destination = child.pid_path.with_suffix('.signal.json')
    destination.parent.mkdir(parents=True, exist_ok=True)
    value = dict(pid=child.pid, argv=child.argv, identity=child.identity, sigint_sent=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                dir=str(destination.parent), prefix=destination.name + '.', delete=False) as output:
            temporary = Path(output.name)
            json.dump(value, output)
            output.flush()
            os.fsync(output.fileno())
        temporary.replace(destination)
        child.sigint_sent = True
        return True
    finally:
        if temporary is not None:
            with contextlib.suppress(OSError):
                temporary.unlink()


async def _process_identity(pid):
    """Use boot identity and Linux start ticks to distinguish same-command PID reuse."""
    if pid <= 1:
        return None
    if Path('/proc').is_dir():
        try:
            raw = (Path('/proc') / str(pid) / 'stat').read_text()
            start_ticks = raw[raw.rfind(')') + 2:].split()[19]
            boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            return 'linux:' + boot + ':' + start_ticks
        except (OSError, IndexError):
            return None
    process = None
    try:
        process = await asyncio.create_subprocess_exec('ps', '-p', str(pid), '-o', 'lstart=',
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        output, _ = await asyncio.wait_for(process.communicate(), 1)
        value = output.decode().strip()
        return 'posix:' + value if process.returncode == 0 and value else None
    except (OSError, ValueError, asyncio.TimeoutError):
        if process is not None and process.returncode is None:
            process.kill()
            await process.communicate()
        return None


async def _process_argv(pid):
    if pid <= 1 or pid == os.getpid():
        return None
    proc_path = Path('/proc') / str(pid) / 'cmdline'
    if Path('/proc').is_dir():
        try:
            return [part.decode('utf-8', 'surrogateescape') for part in proc_path.read_bytes().split(b'\0') if part] or None
        except OSError:
            return None
    try:
        process = await asyncio.create_subprocess_exec('ps', '-p', str(pid), '-o', 'command=',
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
        stdout, _ = await asyncio.wait_for(process.communicate(), 1)
        return shlex.split(stdout.decode().strip()) or None
    except (OSError, ValueError, asyncio.TimeoutError):
        return None


def _same_command(actual, expected):
    if not actual or not expected or len(actual) != len(expected):
        return False
    def executable(name):
        return os.path.realpath(shutil.which(name) or os.path.expanduser(name))
    return executable(actual[0]) == executable(expected[0]) and actual[1:] == expected[1:]


class _OrphanProcess:
    def __init__(self, pid, argv, identity=None):
        self.pid, self.argv, self.identity, self.returncode = pid, argv, identity, None

    async def wait(self):
        while (_same_command(await _process_argv(self.pid), self.argv)
               and await _process_identity(self.pid) == self.identity):
            await asyncio.sleep(.1)
        self.returncode = 0  # Exit status is unavailable for a non-child; never a safety gate.
        return self.returncode

    def send_signal(self, sig):
        os.kill(self.pid, sig)


def _number(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


class Runner:
    def __init__(self, cfg, store, io, state_callback):
        self.cfg, self.store, self.io, self.state_callback = cfg, store, io, state_callback
        self.children = {}
        self._launches, self._watchers = set(), set()
        self._monitor_task = self._landing_task = self._close_task = None
        self._closing = False
        self._ever_ready = False
        self._read_settings()

    def _read_settings(self):
        def value(mapping, *keys):
            for key in keys:
                if not isinstance(mapping, dict):
                    return None
                mapping = mapping.get(key)
            return mapping
        self.land_height = value(self.cfg.raw, 'land', 'height')
        self.land_duration = value(self.cfg.bt, 'coshow', 'durations', 'land')
        self.landed_z = value(self.cfg.bt, 'coshow', 'tolerances', 'landed_z')
        self.pose_freshness = value(self.cfg.raw, 'freshness_s', 'pose')

    def _require_idle(self):
        """Reject mutations before any live control adapter or process is touched."""
        if (self.store.run['state'] != 'IDLE' or self._closing
                or any(self._alive(name) for name in self.children)
                or any(not task.done() for task in self._launches)
                or (self._landing_task is not None and not self._landing_task.done())):
            raise ValueError('실행기 설정은 프로세스가 없는 IDLE 상태에서만 변경할 수 있습니다')

    def reconfigure(self):
        """Reload safety settings from the current Config only while safely idle."""
        self._require_idle()
        self._read_settings()

    def _alive(self, name):
        child = self.children.get(name)
        return child is not None and child.returncode is None

    def _process_state(self, name, child, alive):
        with self.store.lock:
            self.store.context['processes'][name] = dict(alive=alive,
                exited_at=None if alive else self.store.clock())
            if alive or self.store.run.get(name + '_pid') == child.pid:
                self.store.set_run(**{name + '_pid': child.pid if alive else None})
            if not alive:
                self.store.context['orphans'] = [entry for entry in self.store.context['orphans'] if entry['pid'] != child.pid]

    def _task(self, coroutine, collection):
        task = asyncio.create_task(coroutine)
        collection.add(task)
        task.add_done_callback(collection.discard)
        return task

    async def start(self):
        if self._monitor_task is not None or self._closing:
            return
        (HERE / 'run').mkdir(parents=True, exist_ok=True)
        for name in ('bt', 'preflight'):
            path = HERE / 'run' / (name + '.pid')
            try:
                pid = int(path.read_text().strip())
                argv = command_argv(self.cfg, name)
            except (OSError, ValueError):
                continue
            if not _same_command(await _process_argv(pid), argv):
                with contextlib.suppress(OSError):
                    path.unlink()
                continue
            identity = await _process_identity(pid)
            child = OwnedProcess(_OrphanProcess(pid, argv, identity), argv,
                                 HERE / 'logs' / (name + '.log'), path, orphan=True, identity=identity)
            _restore_sigint(child)
            self.children[name] = child
            self._process_state(name, child, True)
            with self.store.lock:
                self.store.context['orphans'].append(dict(name=name, pid=pid))
            self._task(self._watch(name, child), self._watchers)
        if self.store.context['orphans']:
            self.store.set_run(state='ABORTED', last_error='고아 프로세스 감지')
            self.store.event('warning', '고아 프로세스 감지 — reset으로 정리 필요')
        self._monitor_task = asyncio.create_task(self._monitor())

    async def close(self):
        if self._close_task is None:
            self._closing = True
            self._close_task = asyncio.create_task(self._finish_close())
        await asyncio.shield(self._close_task)

    async def _finish_close(self):
        if self._monitor_task:
            self._monitor_task.cancel()
            await asyncio.gather(self._monitor_task, return_exceptions=True)
        await asyncio.gather(*tuple(self._launches), return_exceptions=True)
        if self._landing_task and not self._landing_task.done():
            await asyncio.shield(self._landing_task)
        elif any(self._alive(name) for name in self.children) or self.store.run['state'] in ('CHECKING', 'READY', 'RUNNING', 'DONE'):
            await asyncio.shield(self._begin_stop(False))
        # A preflight refusing its one SIGINT keeps its pidfile for next-start recovery.
        for task in tuple(self._watchers):
            task.cancel()
        await asyncio.gather(*tuple(self._watchers), return_exceptions=True)

    async def command(self, cmd):
        state = self.store.run['state']
        table = {'IDLE': {'preflight', 'reset'}, 'CHECKING': {'estop', 'reset'},
            'READY': {'start', 'estop', 'reset'}, 'RUNNING': {'estop', 'reset'},
            'LANDING': set(), 'DONE': {'estop', 'reset'}, 'ABORTED': {'reset'}}
        if self._closing:
            return False, '서버 종료 중'
        if state == 'LANDING':
            return False, '착륙 중'
        if cmd not in table.get(state, set()):
            return False, '{}에서 허용되지 않음'.format(state)
        if cmd in ('preflight', 'start'):
            if self.store.stack.get('busy'):
                return False, '스택 기동 또는 설정 적용 중'
            if not all(_number(value) for value in (self.land_height, self.land_duration, self.landed_z, self.pose_freshness)) or self.land_duration <= 0 or self.pose_freshness <= 0:
                return False, '착륙 안전 설정 누락 또는 형식 오류'
            name = 'preflight' if cmd == 'preflight' else 'bt'
            if self._alive(name):
                return False, '{} 이전 프로세스 종료 대기'.format(name)
            try:
                command_argv(self.cfg, name)
                if cmd == 'start' and any(row.get('blocking') and not row.get('ok') for row in self.state_callback()['checklist']):
                    return False, 'blocking 점검 항목 실패'
            except Exception as exc:
                return False, str(exc)
            if cmd == 'preflight':
                self.store.reset_cached()
                self._ever_ready = False
                self.store.set_run(state='CHECKING', last_error=None)
            else:
                self.store.set_run(state='RUNNING', last_error=None)
            self._task(self._launch(name), self._launches)
        elif cmd == 'reset' and not any(self._alive(name) for name in self.children) and state in ('IDLE', 'ABORTED'):
            self.store.reset_cached()
            self.store.set_run(state='IDLE', last_error=None)
        else:
            self._begin_stop(cmd == 'reset')
        return True, ''

    async def _launch(self, name):
        expected = 'CHECKING' if name == 'preflight' else 'RUNNING'
        if self.store.run['state'] != expected or self._closing:
            return
        try:
            child = await spawn_process(command_argv(self.cfg, name),
                self.cfg.bt_cwd if name == 'bt' else self.cfg.bt_cwd.parent,
                self.cfg.raw.get('commands', {}).get('env', {}),
                HERE / 'logs' / (name + '.log'), HERE / 'run' / (name + '.pid'))
            self.children[name] = child
            self._process_state(name, child, True)
            self._task(self._watch(name, child), self._watchers)
            self.store.event('info', '{} 시작 pid={}'.format(name, child.pid))
        except Exception as exc:
            reason = '{} 시작 실패: {}'.format(name, exc)
            self.store.set_run(last_error=reason)
            self.store.event('warning', reason)
            self._begin_stop(False, short=True)

    async def _watch(self, name, child):
        code = await child.proc.wait()
        if self.children.get(name) is not child:
            return
        self._process_state(name, child, False)
        _remove_pid(child)
        current = self.store.run['state']
        if child.sigint_sent:
            self.store.event('info', '{} 종료(rc={}, estop 요청)'.format(name.upper() if name == 'bt' else name, code))
        elif name == 'bt' and current == 'RUNNING':
            reason = 'BT exited rc={}'.format(code)
            self.store.set_run(last_error=reason)
            self.store.event('warning', reason)
            self._log_tail(child)
            self._begin_stop(False, short=True)
        elif name == 'preflight' and current in ('CHECKING', 'READY', 'RUNNING', 'DONE'):
            reason = 'preflight 프로세스 사망'
            self.store.set_run(last_error=reason)
            self.store.event('warning', reason)
            self._begin_stop(False, short=not self._alive('bt'), preflight_dead=True)
        else:
            self.store.event('info', '{} 종료(rc={})'.format(name, code))

    def _log_tail(self, child):
        try:
            with child.log_path.open('rb') as source:
                source.seek(0, 2)
                source.seek(max(0, source.tell() - 65536))
                lines = source.read().decode('utf-8', 'replace').splitlines()[-20:]
            if lines:
                self.store.event('warning', 'BT 로그 마지막 20줄:\n' + '\n'.join(lines))
        except OSError:
            pass

    async def _monitor(self):
        while True:
            try:
                snapshot = self.store.snapshot()
                current = snapshot['run']['state']
                if current in ('CHECKING', 'READY', 'RUNNING'):
                    preflight = snapshot.get('preflight') or {}
                    reason = preflight.get('abort_reason')
                    failed = any(stage.get('result') == 'fail' for stage in preflight.get('stages', []))
                    if reason is not None or failed or (self._ever_ready and preflight.get('ready') is False):
                        reason = reason if reason is not None else ('preflight 점검 실패' if failed else 'preflight ready=False')
                        self.store.set_run(last_error=reason)
                        self.store.event('warning', reason)
                        self._begin_stop(False)
                    elif preflight.get('ready') is True:
                        self._ever_ready = True
                        if current == 'CHECKING' and not any(row.get('blocking') and not row.get('ok') for row in self.state_callback()['checklist']):
                            self.store.set_run(state='READY')
                if self.store.run['state'] == 'RUNNING' and (snapshot.get('mission') or {}).get('phase') == 'done':
                    self.store.set_run(state='DONE')
                    self._task(self._disarm(), self._launches)
            except Exception as exc:
                self.store.event('warning', '실행 상태 감시 오류: {}'.format(exc))
            await asyncio.sleep(.05)

    def _begin_stop(self, reset, short=False, preflight_dead=False):
        if self._landing_task and not self._landing_task.done():
            return self._landing_task
        self.store.set_run(state='LANDING')
        self.store.event('info', '착륙 시퀀스 시작')
        self._landing_task = asyncio.create_task(self._sequence(reset, short, preflight_dead))
        return self._landing_task

    async def _signal(self, child, sig):
        if child is None or child.returncode is not None:
            return
        if child.orphan and (not _same_command(await _process_argv(child.pid), child.argv)
                             or await _process_identity(child.pid) != child.identity):
            self.store.event('warning', 'PID 명령행 변경 — 신호 전송 생략: {}'.format(child.pid))
            return
        try:
            if sig == signal.SIGINT and not _remember_sigint(child):
                return
            child.proc.send_signal(sig)
        except ProcessLookupError:
            pass
        except OSError as exc:
            self.store.event('warning', '프로세스 신호 미확인: {}'.format(exc))

    async def _control(self, method, name, *args):
        try:
            function = getattr(self.io, method, None)
            if function is None:
                raise RuntimeError('서비스 없음')
            # ROSIO owns the one-second service boundary. Allow its response and
            # accounting to finish before this defensive adapter ceiling.
            return await asyncio.wait_for(function(name, *args), 1.5)
        except (Exception, asyncio.TimeoutError) as exc:
            self.store.event('warning', '{} {} 미확인: {!r}'.format(name, method, exc))
            return None

    async def _land_and_cancel(self):
        async def land(name):
            if not _number(self.land_height) or not _number(self.land_duration) or self.land_duration <= 0:
                self.store.event('warning', '{} land 미확인: 안전 설정 없음'.format(name))
                return False
            return await self._control('land', name, self.land_height, self.land_duration)
        async def cancel(name):
            if await self._control('cancel', name) is not True:
                self.store.event('warning', '{} 취소 미확인'.format(name))
        results = await asyncio.gather(*(land(name) for name in self.cfg.drones), *(cancel(name) for name in self.cfg.limos))
        sent = sum(value is True for value in results[:len(self.cfg.drones)])
        self.store.event('info', 'land 전송 {}/{}'.format(sent, len(self.cfg.drones)))

    async def _disarm(self):
        for name in self.cfg.drones:
            # Re-read at each call; preceding service waits cannot make old pose look fresh.
            row = self.store.snapshot()['robots'].get(name, {})
            age, z = row.get('pose_age'), (row.get('pose') or {}).get('z')
            if (_number(age) and _number(self.pose_freshness) and 0 <= age < self.pose_freshness
                    and _number(z) and _number(self.landed_z) and z <= self.landed_z):
                await self._control('arm', name, False)
            else:
                self.store.event('warning', '{} 착륙 미확인 — 육안 확인 후 수동 해제'.format(name))

    async def _sequence(self, reset, short, preflight_dead):
        # Commands may arrive while subprocess creation is awaiting the OS.
        await asyncio.gather(*tuple(self._launches), return_exceptions=True)
        bt = self.children.get('bt')
        # A preflight death can race an in-flight OS spawn. Re-evaluate only after
        # that spawn is owned; direct land must never race a live BT tick loop.
        if short and bt is not None and bt.returncode is None:
            short = False
        if not short:
            await self._signal(bt, signal.SIGINT)
            await asyncio.sleep(.5)
        duration = self.land_duration if _number(self.land_duration) and self.land_duration > 0 else 0
        deadline = asyncio.get_running_loop().time() + duration + 2
        await self._land_and_cancel()
        if not short and bt is not None and bt.returncode is None:
            try:
                await asyncio.wait_for(asyncio.shield(bt.proc.wait()), max(0, deadline - asyncio.get_running_loop().time()))
            except asyncio.TimeoutError:
                await self._signal(bt, signal.SIGKILL)
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(asyncio.shield(bt.proc.wait()), 1)
        preflight = self.children.get('preflight')
        if preflight_dead:
            self.store.event('info', 'preflight 이미 종료')
        elif not short and preflight is not None and preflight.returncode is None:
            await self._signal(preflight, signal.SIGINT)
            try:
                await asyncio.wait_for(asyncio.shield(preflight.proc.wait()), 1)
            except asyncio.TimeoutError:
                self.store.event('warning', 'preflight 종료 미확인 — PID 유지')
        await self._disarm()
        # A recovered process waiter can finish before its watcher wakes. Clear
        # completed PID/receipt files before exposing the final reset state.
        for child in self.children.values():
            if child.returncode is not None:
                _remove_pid(child)
        if reset:
            self.store.reset_cached()
            self.store.set_run(state='IDLE', last_error=None)
        else:
            self.store.set_run(state='ABORTED')
