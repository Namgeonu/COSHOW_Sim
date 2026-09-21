"""Lifecycle tests use process-boundary doubles plus real isolated child processes."""
import asyncio
import json
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace

import pytest

from dashboard import runner as module
from dashboard.runner import Runner, command_argv
from dashboard.state import TelemetryStore


class Process:
    next_pid = 50000

    def __init__(self, name, events, stubborn=False, requested_rc=0):
        Process.next_pid += 1
        self.pid, self.name, self.events = Process.next_pid, name, events
        self.returncode, self.stubborn, self.requested_rc = None, stubborn, requested_rc
        self.finished = asyncio.Event()

    async def wait(self):
        await self.finished.wait()
        return self.returncode

    def exit(self, code):
        self.returncode = code
        self.finished.set()

    def send_signal(self, value):
        self.events.append(('signal', self.name, value, time.monotonic()))
        if value == signal.SIGKILL or not self.stubborn:
            self.exit(-signal.SIGKILL if value == signal.SIGKILL else self.requested_rc)


class Controls:
    def __init__(self, events, store):
        self.events, self.store = events, store
        self.cancel_result = True

    async def land(self, name, height, duration):
        self.events.append(('land', name, height, duration, time.monotonic()))
        # New telemetry, not an assumed landing. Preserve each supplied altitude.
        row = self.store.snapshot()['robots'][name]
        if row['pose'] is not None:
            self.store.receive(name, 'pose', row['pose'])
        return True

    async def arm(self, name, value):
        self.events.append(('arm', name, value, time.monotonic()))
        return True

    async def cancel(self, name):
        self.events.append(('cancel', name, time.monotonic()))
        if self.cancel_result == 'hang':
            await asyncio.Event().wait()
        return self.cancel_result


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(module, 'HERE', tmp_path / 'dashboard')
    cfg = SimpleNamespace(raw={'commands': {'bt': 'python3 mission.py', 'preflight': 'python3 gate.py', 'env': {}},
        'land': {'height': 0}, 'freshness_s': {'pose': .8}},
        bt={'coshow': {'durations': {'land': .02}, 'tolerances': {'landed_z': .1},
                       'drones': {'alpha': {'base': [1, 2]}, 'beta': {'base': [3, 4]}}}},
        bt_cwd=tmp_path / 'bt', drones=['alpha', 'beta'], limos=['carrier'], radio_counts={},
        robots={'alpha': {'kind': 'drone', 'role': 'alpha'}, 'beta': {'kind': 'drone', 'role': 'beta'},
                'reserve': {'kind': 'drone', 'role': None}, 'carrier': {'kind': 'limo', 'role': 'carrier'}})
    cfg.bt_cwd.mkdir()
    store = TelemetryStore(cfg)
    for name in cfg.robots:
        store.receive(name, 'pose', dict(x=0, y=0, z=0))
    events, processes, spawn_options = [], {}, []

    async def spawn(*argv, **kwargs):
        name = 'bt' if 'mission.py' in argv else 'preflight'
        process = Process(name, events)
        processes[name] = process
        spawn_options.append((argv, kwargs))
        return process

    monkeypatch.setattr(module.asyncio, 'create_subprocess_exec', spawn)
    io = Controls(events, store)
    runner = Runner(cfg, store, io, store.snapshot)
    return runner, cfg, store, io, events, processes, spawn_options


async def until(predicate, timeout=4):
    deadline = time.monotonic() + timeout
    while not predicate():
        assert time.monotonic() < deadline, 'timed out waiting for lifecycle transition'
        await asyncio.sleep(.01)


async def launch(setup, ready=True):
    runner, cfg, store, io, events, processes, options = setup
    await runner.start()
    assert (await runner.command('preflight'))[0]
    await until(lambda: 'preflight' in processes)
    if ready:
        store.preflight({'ready': True, 'abort_reason': None})
        await until(lambda: store.run['state'] == 'READY')
        assert (await runner.command('start'))[0]
        await until(lambda: 'bt' in processes)
    return runner


ACCEPTED = {'IDLE': {'preflight', 'reset'}, 'CHECKING': {'estop', 'reset'},
    'READY': {'start', 'estop', 'reset'}, 'RUNNING': {'estop', 'reset'},
    'LANDING': set(), 'DONE': {'estop', 'reset'}, 'ABORTED': {'reset'}}


def test_reconfigure_reloads_current_safety_settings_and_rejects_missing_values(setup):
    runner, cfg, store, *_ = setup
    cfg.raw['land']['height'] = .03
    cfg.raw['freshness_s']['pose'] = .25
    cfg.bt['coshow']['durations']['land'] = 1.5
    cfg.bt['coshow']['tolerances']['landed_z'] = .04
    runner.reconfigure()
    assert (runner.land_height, runner.land_duration, runner.landed_z, runner.pose_freshness) == (.03, 1.5, .04, .25)
    cfg.raw['land'] = 'invalid mapping'
    cfg.bt['coshow']['durations'] = None
    runner.reconfigure()
    assert runner.land_height is None and runner.land_duration is None
    accepted, reason = asyncio.run(runner.command('preflight'))
    assert not accepted and '안전 설정' in reason
    assert store.run['state'] == 'IDLE'


@pytest.mark.parametrize('state', [state for state in ACCEPTED if state != 'IDLE'])
def test_reconfigure_cannot_change_inflight_safety_thresholds(setup, state):
    runner, cfg, store, *_ = setup
    old_height = runner.land_height
    cfg.raw['land']['height'] = .9
    store.set_run(state=state)
    with pytest.raises(ValueError, match='IDLE'):
        runner.reconfigure()
    assert runner.land_height == old_height


@pytest.mark.parametrize('pending', ['live_child', 'launch', 'landing', 'closing'])
def test_reconfigure_requires_idle_process_and_task_state(setup, pending):
    async def exercise():
        runner, cfg, _, *_ = setup
        cfg.raw['land']['height'] = .9
        task = None
        if pending == 'live_child':
            runner.children['bt'] = SimpleNamespace(returncode=None)
        elif pending == 'closing':
            runner._closing = True
        else:
            task = asyncio.create_task(asyncio.Event().wait())
            if pending == 'launch':
                runner._launches.add(task)
            else:
                runner._landing_task = task
        try:
            with pytest.raises(ValueError):
                runner.reconfigure()
            assert runner.land_height == 0
        finally:
            if task:
                task.cancel()
                await asyncio.gather(task, return_exceptions=True)
    asyncio.run(exercise())


@pytest.mark.parametrize('state', list(ACCEPTED))
@pytest.mark.parametrize('cmd', ['preflight', 'start', 'estop', 'reset'])
def test_entire_command_acceptance_table(setup, state, cmd):
    async def exercise():
        runner, _, store, *_ = setup
        store.set_run(state=state)
        accepted, reason = await runner.command(cmd)
        assert accepted == (cmd in ACCEPTED[state]), (state, cmd, reason)
        if state == 'LANDING':
            assert '착륙 중' in reason
        await runner.close()
    asyncio.run(exercise())


def test_start_claims_state_before_spawn_and_survives_request_cancellation(setup):
    async def exercise():
        runner, _, store, _, _, processes, _ = setup
        store.set_run(state='READY')
        assert (await runner.command('start'))[0]
        assert store.run['state'] == 'RUNNING'
        assert not (await runner.command('start'))[0]
        await until(lambda: 'bt' in processes)
        assert store.run['bt_pid'] == processes['bt'].pid
        await runner.close()
    asyncio.run(exercise())


def test_start_rechecks_blocking_rows_and_reports_exact_abort_reason(setup):
    async def exercise():
        runner, _, store, *_ = setup
        runner.state_callback = lambda: {'checklist': [{'blocking': True, 'ok': False}]}
        store.set_run(state='READY')
        assert not (await runner.command('start'))[0]
        assert store.run['state'] == 'READY'
        await runner.start()
        store.preflight({'ready': False, 'abort_reason': 'beta: /status 끊김'})
        await until(lambda: store.run['state'] == 'ABORTED')
        assert store.run['last_error'] == 'beta: /status 끊김'
        assert any(e['text'] == 'beta: /status 끊김' for e in store.events)
        await runner.close()
    asyncio.run(exercise())


def test_estop_order_one_sigint_roles_only_and_fresh_low_disarm(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, events, processes, _ = setup
        store.receive('beta', 'pose', dict(x=0, y=0, z=1))
        requested = time.monotonic()
        assert (await runner.command('estop'))[0]
        assert store.run['state'] == 'LANDING'
        assert not (await runner.command('estop'))[0]
        await until(lambda: store.run['state'] == 'ABORTED')
        kinds = [e[:2] for e in events]
        assert kinds[0] == ('signal', 'bt')
        assert events[0][2] == signal.SIGINT
        lands = [e for e in events if e[0] == 'land']
        assert {e[1] for e in lands} == {'alpha', 'beta'}
        assert all(e[2:4] == (0, .02) and e[-1] - requested >= .49 for e in lands)
        assert ('cancel', 'carrier') in kinds
        assert kinds.index(('signal', 'preflight')) > kinds.index(('land', 'beta'))
        assert [e[1] for e in events if e[0] == 'arm'] == ['alpha']
        assert all(e[1] != 'reserve' for e in events)
        assert sum(e[:3] == ('signal', 'bt', signal.SIGINT) for e in events) == 1
        assert any('beta 착륙 미확인' in e['text'] for e in store.events)
        assert store.context['processes']['bt']['exited_at'] is not None
        await runner.close()
    asyncio.run(exercise())


def test_stubborn_bt_is_killed_after_deadline_without_second_sigint(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, events, processes, _ = setup
        processes['bt'].stubborn = True
        await runner.command('estop')
        await until(lambda: store.run['state'] == 'ABORTED')
        signals = [e for e in events if e[:2] == ('signal', 'bt')]
        assert [e[2] for e in signals] == [signal.SIGINT, signal.SIGKILL]
        assert signals[1][-1] - signals[0][-1] >= 2.0
        await runner.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('code', [0, 1, -6])
def test_any_unrequested_bt_exit_while_running_falls_back_to_land_cancel_disarm(setup, code):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, events, processes, _ = setup
        started = time.monotonic()
        processes['bt'].exit(code)
        await until(lambda: store.run['state'] == 'ABORTED')
        assert store.run['last_error'] == 'BT exited rc={}'.format(code)
        assert {e[1] for e in events if e[0] == 'land'} == {'alpha', 'beta'}
        assert any(e[:2] == ('cancel', 'carrier') for e in events)
        assert not any(e[:2] == ('signal', 'bt') for e in events)
        assert min(e[-1] for e in events if e[0] == 'land') - started < .4
        await runner.close()
    asyncio.run(exercise())


def test_requested_sigabrt_exit_is_info_and_not_last_error(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, _, processes, _ = setup
        processes['bt'].requested_rc = -6
        await runner.command('estop')
        await until(lambda: store.run['state'] == 'ABORTED')
        assert store.run['last_error'] is None
        assert any('rc=-6' in e['text'] and 'estop 요청' in e['text'] and e['level'] == 'info' for e in store.events)
        await runner.close()
    asyncio.run(exercise())


def test_preflight_death_stops_live_bt_before_direct_land(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, events, processes, _ = setup
        processes['preflight'].exit(0)
        await until(lambda: store.run['state'] == 'ABORTED')
        assert store.run['last_error'] == 'preflight 프로세스 사망'
        assert events[0][:3] == ('signal', 'bt', signal.SIGINT)
        assert not any(e[:2] == ('signal', 'preflight') for e in events)
        assert any('preflight 이미 종료' in e['text'] for e in store.events)
        await runner.close()
    asyncio.run(exercise())


def test_initial_false_is_checking_but_true_to_false_aborts(setup):
    async def exercise():
        runner = await launch(setup, False)
        _, _, store, *_ = setup
        store.ready(False)
        await asyncio.sleep(.15)
        assert store.run['state'] == 'CHECKING'
        store.ready(True)
        await until(lambda: store.run['state'] == 'READY')
        store.ready(False)
        await until(lambda: store.run['state'] == 'ABORTED')
        await runner.close()
    asyncio.run(exercise())


def test_done_disarms_without_stopping_bt_and_self_exit_preserves_done(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, events, processes, _ = setup
        pid = processes['bt'].pid
        store.mission({'phase': 'done'})
        await until(lambda: store.run['state'] == 'DONE')
        await until(lambda: any(e[0] == 'arm' for e in events))
        assert store.run['bt_pid'] == pid
        assert not any(e[0] in ('signal', 'land', 'cancel') for e in events)
        processes['bt'].exit(1)
        await until(lambda: store.run['bt_pid'] is None)
        assert store.run['state'] == 'DONE'
        assert store.run['last_error'] is None
        await runner.close()
    asyncio.run(exercise())


def test_reset_clears_mission_caches_but_keeps_live_telemetry(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, *_ = setup
        store.mission({'phase': 'search', 'led': {'alpha': 'red'}, 'cmd': {'alpha': {'kind': 'go_to'}}})
        store.receive('alpha', 'detections', [3])
        await runner.command('reset')
        await until(lambda: store.run['state'] == 'IDLE')
        snapshot = store.snapshot()
        assert snapshot['mission'] is None and snapshot['preflight'] is None
        assert snapshot['robots']['alpha']['detections'] == []
        assert snapshot['robots']['alpha']['led'] == 'off'
        assert snapshot['robots']['alpha']['cmd'] is None
        assert snapshot['robots']['alpha']['pose'] is not None
        await runner.close()
    asyncio.run(exercise())


def test_cancel_timeout_and_missing_controls_do_not_stall_sequence(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, io, *_ = setup
        io.cancel_result = 'hang'
        await runner.command('estop')
        await until(lambda: store.run['state'] == 'ABORTED', timeout=3)
        assert any('carrier 취소 미확인' in e['text'] for e in store.events)
        runner.io = None
        store.set_run(state='READY')
        await runner.command('reset')
        await until(lambda: store.run['state'] == 'IDLE')
        await runner.close()
    asyncio.run(exercise())


def test_shutdown_and_immediate_spawn_estop_share_one_sequence(setup):
    async def exercise():
        runner, _, store, _, events, processes, _ = setup
        store.set_run(state='READY')
        await runner.command('start')
        await asyncio.sleep(0)
        await runner.command('estop')
        await asyncio.gather(runner.close(), runner.close())
        assert sum(e[:3] == ('signal', 'bt', signal.SIGINT) for e in events) <= 1
        assert not any(p.returncode is None for p in processes.values())
    asyncio.run(exercise())


def test_disarm_requires_strict_freshness_and_landed_threshold(setup):
    async def exercise():
        runner, cfg, store, _, events, *_ = setup
        now = store.clock()
        with store.lock:
            store.data['alpha']['pose'] = ({'x': 0, 'y': 0, 'z': .1}, now - .8)
            store.data['beta']['pose'] = ({'x': 0, 'y': 0, 'z': .10001}, now)
        store.set_run(state='RUNNING')
        store.mission({'phase': 'done'})
        await runner.start()
        await until(lambda: store.run['state'] == 'DONE')
        await asyncio.sleep(.05)
        assert not any(e[0] == 'arm' for e in events)
        await runner.close()
    asyncio.run(exercise())


def test_command_expansion_keeps_role_arrays_one_argument_and_rejects_shell_syntax(setup):
    _, cfg, *_ = setup
    cfg.raw['commands']['preflight'] = 'python3 gate.py --ros-args -p drones:={roles} -p expected_x:={expected_x}'
    argv = command_argv(cfg, 'preflight')
    assert 'drones:=["alpha","beta"]' in argv
    assert 'expected_x:=[1,3]' in argv
    for command in ['bash -c "python3 mission.py"', 'python3 mission.py | cat', 'python3 $SCRIPT', 'python3 mission.py > out']:
        cfg.raw['commands']['bt'] = command
        with pytest.raises(ValueError):
            command_argv(cfg, 'bt')


def test_spawn_options_merge_env_detach_session_and_disable_cores(setup, monkeypatch):
    async def exercise():
        runner, cfg, _, _, _, processes, options = setup
        monkeypatch.setenv('RUNNER_INHERITED', 'kept')
        cfg.raw['commands']['env']['RUNNER_OVERRIDE'] = 'configured'
        await launch(setup)
        for argv, kwargs in options:
            assert kwargs['start_new_session'] is True
            assert kwargs['env']['RUNNER_INHERITED'] == 'kept'
            assert kwargs['env']['RUNNER_OVERRIDE'] == 'configured'
            assert kwargs['stdout'] is not asyncio.subprocess.PIPE
            assert kwargs['stderr'] == asyncio.subprocess.STDOUT
            assert callable(kwargs['preexec_fn'])
            assert kwargs['cwd'] == str(cfg.bt_cwd if 'mission.py' in argv else cfg.bt_cwd.parent)
        assert (module.HERE / 'run/bt.pid').read_text().strip() == str(processes['bt'].pid)
        await runner.close()
    asyncio.run(exercise())


def test_preflight_death_during_bt_spawn_still_signals_new_bt_before_land(setup, monkeypatch):
    async def exercise():
        runner = await launch(setup, False)
        _, _, store, _, events, processes, _ = setup
        entered, release = asyncio.Event(), asyncio.Event()
        original = module.asyncio.create_subprocess_exec
        async def delayed(*argv, **kwargs):
            if 'mission.py' in argv:
                entered.set()
                await release.wait()
            return await original(*argv, **kwargs)
        monkeypatch.setattr(module.asyncio, 'create_subprocess_exec', delayed)
        store.ready(True)
        await until(lambda: store.run['state'] == 'READY')
        await runner.command('start')
        await entered.wait()
        processes['preflight'].exit(0)
        await until(lambda: store.run['state'] == 'LANDING')
        release.set()
        await until(lambda: store.run['state'] == 'ABORTED')
        assert events[0][:3] == ('signal', 'bt', signal.SIGINT)
        assert processes['bt'].returncode is not None
        await runner.close()
    asyncio.run(exercise())


def test_cancel_wait_is_inside_bt_exit_deadline_not_added_to_it(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, io, events, processes, _ = setup
        processes['bt'].stubborn = True
        io.cancel_result = 'hang'
        await runner.command('estop')
        await until(lambda: store.run['state'] == 'ABORTED', timeout=5)
        land_at = next(e[-1] for e in events if e[0] == 'land')
        killed_at = next(e[-1] for e in events if e[:3] == ('signal', 'bt', signal.SIGKILL))
        assert killed_at - land_at < 2.2
        await runner.close()
    asyncio.run(exercise())


def test_stack_restart_and_flight_commands_cannot_overlap(setup):
    async def exercise():
        runner, _, store, *_ = setup
        store.set_stack(busy=True)
        for current, cmd in [('IDLE', 'preflight'), ('READY', 'start')]:
            store.set_run(state=current)
            assert not (await runner.command(cmd))[0]
            assert store.run['state'] == current
        store.set_stack(busy=False)
        await runner.close()
    asyncio.run(exercise())


DUMMY = '''import json, os, resource, signal, sys, time
def interrupted(sig, frame):
    print("SIGINT", flush=True)
    if "abort" in sys.argv:
        os.abort()
    raise SystemExit(0)
signal.signal(signal.SIGINT, interrupted)
print(json.dumps(dict(pid=os.getpid(), sid=os.getsid(0), core=resource.getrlimit(resource.RLIMIT_CORE),
    cwd=os.getcwd(), inherited=os.environ.get("RUNNER_PARENT"), override=os.environ.get("RUNNER_CHILD"))), flush=True)
while True:
    time.sleep(.02)
'''


def real_setup(tmp_path, monkeypatch):
    monkeypatch.setattr(module, 'HERE', tmp_path / 'dashboard')
    cfg = SimpleNamespace(raw={'commands': {'env': {}}, 'land': {'height': 0}, 'freshness_s': {'pose': .8}},
        bt={'coshow': {'durations': {'land': .02}, 'tolerances': {'landed_z': .1}, 'drones': {'alpha': {'base': [0, 0]}}}},
        bt_cwd=tmp_path / 'bt', drones=['alpha'], limos=['carrier'], radio_counts={},
        robots={'alpha': {'kind': 'drone', 'role': 'alpha'}, 'reserve': {'kind': 'drone', 'role': None},
                'carrier': {'kind': 'limo', 'role': 'carrier'}})
    cfg.bt_cwd.mkdir()
    script = tmp_path / 'dummy.py'
    script.write_text(DUMMY)
    cfg.raw['commands'].update(bt='{} {} bt abort'.format(sys.executable, script),
                               preflight='{} {} preflight'.format(sys.executable, script))
    store = TelemetryStore(cfg)
    store.receive('alpha', 'pose', dict(x=0, y=0, z=0))
    events = []
    io = Controls(events, store)
    return Runner(cfg, store, io, store.snapshot), cfg, store, events


def test_real_process_spawn_core_limit_session_environment_and_requested_sigabrt(tmp_path, monkeypatch):
    async def exercise():
        runner, cfg, store, events = real_setup(tmp_path, monkeypatch)
        monkeypatch.setenv('RUNNER_PARENT', 'inherited')
        cfg.raw['commands']['env']['RUNNER_CHILD'] = 'configured'
        await runner.start()
        try:
            await runner.command('preflight')
            gate_log = module.HERE / 'logs/preflight.log'
            await until(lambda: gate_log.exists() and gate_log.stat().st_size > 0)
            store.ready(True)
            await until(lambda: store.run['state'] == 'READY')
            await runner.command('start')
            bt_log = module.HERE / 'logs/bt.log'
            await until(lambda: bt_log.exists() and bt_log.stat().st_size > 0)
            observed = json.loads(bt_log.read_text().splitlines()[0])
            assert observed['sid'] == observed['pid'] != os.getsid(0)
            assert observed['core'] == [0, 0]
            assert observed['inherited'] == 'inherited' and observed['override'] == 'configured'
            assert observed['cwd'] == str(cfg.bt_cwd)
            await runner.command('estop')
            await until(lambda: store.run['state'] == 'ABORTED')
            assert bt_log.read_text().splitlines().count('SIGINT') == 1
            assert runner.children['bt'].returncode == -signal.SIGABRT
            assert store.run['last_error'] is None
            assert not list(cfg.bt_cwd.glob('core*'))
            assert not (module.HERE / 'run/bt.pid').exists()
            assert any('rc=-6' in e['text'] and 'estop 요청' in e['text'] for e in store.events)
            print('REAL_SPAWN', json.dumps(observed), 'requested_rc=-6 SIGINT=1 core_files=0')
        finally:
            await runner.close()
    asyncio.run(exercise())


def test_real_orphan_matching_recovers_owned_pid_and_never_signals_unrelated_pid(tmp_path, monkeypatch):
    async def exercise():
        runner, cfg, store, events = real_setup(tmp_path, monkeypatch)
        owned = await module.spawn_process(command_argv(cfg, 'bt'), cfg.bt_cwd, {},
            module.HERE / 'logs/bt.log', module.HERE / 'run/bt.pid')
        unrelated = await module.spawn_process([sys.executable, str(tmp_path / 'dummy.py'), 'unrelated'], cfg.bt_cwd, {},
            tmp_path / 'unrelated.log', tmp_path / 'unrelated.pid')
        (module.HERE / 'run/preflight.pid').write_text(str(unrelated.pid))
        await until(lambda: (tmp_path / 'unrelated.log').stat().st_size > 0 and (module.HERE / 'logs/bt.log').stat().st_size > 0)
        try:
            await runner.start()
            assert store.run['state'] == 'ABORTED'
            assert store.context['orphans'] == [{'name': 'bt', 'pid': owned.pid}]
            assert not (await runner.command('preflight'))[0]
            await runner.command('reset')
            await until(lambda: store.run['state'] == 'IDLE')
            await owned.proc.wait()
            assert unrelated.returncode is None
            assert 'SIGINT' not in (tmp_path / 'unrelated.log').read_text()
            assert store.context['orphans'] == []
            assert not (module.HERE / 'run/bt.pid').exists()
            print('REAL_ORPHAN matched=1 recovered=1 unrelated_signals=0 pidfiles_cleared=true')
        finally:
            await runner.close()
            await module.stop_process(owned)
            await module.stop_process(unrelated)
    asyncio.run(exercise())


def test_preflight_coordinate_placeholders_reject_missing_base_without_crashing(setup):
    _, cfg, *_ = setup
    cfg.raw['commands']['preflight'] += ' -p expected_x:={expected_x}'
    cfg.bt['coshow']['drones']['alpha']['base'] = None
    with pytest.raises(ValueError, match='base'):
        command_argv(cfg, 'preflight')
    # An unused coordinate placeholder cannot break parsing an independent BT command.
    assert command_argv(cfg, 'bt') == ['python3', 'mission.py']


def test_spawn_failure_does_not_leave_running_and_does_call_direct_fallback(setup, monkeypatch):
    async def exercise():
        runner, _, store, _, events, *_ = setup
        async def unavailable(*args, **kwargs):
            raise FileNotFoundError('missing executable')
        monkeypatch.setattr(module.asyncio, 'create_subprocess_exec', unavailable)
        store.set_run(state='READY')
        assert (await runner.command('start'))[0]
        await until(lambda: store.run['state'] == 'ABORTED')
        assert 'bt 시작 실패' in store.run['last_error']
        assert {e[1] for e in events if e[0] == 'land'} == {'alpha', 'beta'}
        assert any(e[:2] == ('cancel', 'carrier') for e in events)
        assert store.run['bt_pid'] is None
        await runner.close()
    asyncio.run(exercise())


def test_unrequested_exit_reports_only_last_twenty_log_lines(setup):
    async def exercise():
        runner = await launch(setup)
        _, _, store, _, _, processes, _ = setup
        (module.HERE / 'logs/bt.log').write_text('\n'.join('line-{}'.format(i) for i in range(40)))
        processes['bt'].exit(0)
        await until(lambda: store.run['state'] == 'ABORTED')
        report = next(event['text'] for event in store.events if '로그 마지막 20줄' in event['text'])
        assert report.splitlines()[1:] == ['line-{}'.format(i) for i in range(20, 40)]
        await runner.close()
    asyncio.run(exercise())


def test_orphan_recovery_restores_first_sigint_before_reset_without_second_signal(tmp_path, monkeypatch):
    async def exercise():
        old_runner, cfg, _, _ = real_setup(tmp_path, monkeypatch)
        (tmp_path / 'dummy.py').write_text('''import signal, time
signal.signal(signal.SIGINT, lambda sig, frame: print('SIGINT', flush=True))
print('READY', flush=True)
while True:
    time.sleep(.02)
''')
        owned = await module.spawn_process(command_argv(cfg, 'bt'), cfg.bt_cwd, {},
            module.HERE / 'logs/bt.log', module.HERE / 'run/bt.pid')
        await until(lambda: 'READY' in owned.log_path.read_text())
        restarted_store = TelemetryStore(cfg)
        restarted = Runner(cfg, restarted_store, None, restarted_store.snapshot)
        try:
            await old_runner._signal(owned, signal.SIGINT)
            await until(lambda: 'SIGINT' in owned.log_path.read_text())
            await restarted.start()
            assert restarted.children['bt'].pid == owned.pid
            assert restarted.children['bt'].sigint_sent is True, 'orphan lost already-sent SIGINT state'
            assert restarted_store.run['state'] == 'ABORTED'
            await restarted.command('reset')
            await until(lambda: restarted_store.run['state'] == 'IDLE', timeout=5)
            await owned.proc.wait()
            assert owned.log_path.read_text().splitlines().count('SIGINT') == 1
            assert not list((module.HERE / 'run').glob('*.signal.json'))
        finally:
            await restarted.close()
            if owned.returncode is None:
                owned.proc.kill()
            await owned.proc.wait()
    asyncio.run(exercise())


@pytest.mark.parametrize('mismatch', ['pid', 'argv', 'identity'])
def test_sigint_receipt_does_not_apply_to_different_process_identity(tmp_path, mismatch):
    assert hasattr(module, '_restore_sigint'), 'persistent SIGINT receipt support missing'
    process = Process('bt', [])
    child = module.OwnedProcess(process, ['python3', 'mission.py'], tmp_path / 'bt.log', tmp_path / 'bt.pid')
    child.identity = 'linux:boot:123'
    receipt = {'pid': child.pid, 'argv': child.argv, 'identity': child.identity, 'sigint_sent': True}
    receipt[mismatch] = {'pid': child.pid + 1, 'argv': ['python3', 'different.py'],
                         'identity': 'linux:boot:124'}[mismatch]
    child.pid_path.with_suffix('.signal.json').write_text(json.dumps(receipt))
    module._restore_sigint(child)
    assert child.sigint_sent is False


def test_sigint_receipt_is_durable_before_signal_delivery(setup, monkeypatch):
    async def exercise():
        runner = await launch(setup)
        child = runner.children['bt']
        original = child.proc.send_signal
        observed = []

        def delivered(sig):
            if sig == signal.SIGINT:
                path = child.pid_path.with_suffix('.signal.json')
                assert path.exists(), 'SIGINT was delivered before its receipt was persisted'
                receipt = json.loads(path.read_text())
                assert receipt['pid'] == child.pid and receipt['argv'] == child.argv
                assert receipt['sigint_sent'] is True
                observed.append(True)
            original(sig)

        monkeypatch.setattr(child.proc, 'send_signal', delivered)
        await runner.command('estop')
        await until(lambda: runner.store.run['state'] == 'ABORTED')
        assert observed == [True]
        await runner.close()
    asyncio.run(exercise())


def test_landing_progress_has_one_current_sequence_boundary(setup):
    async def exercise():
        runner = await launch(setup)
        runner.store.event('info', 'land 전송 4/4')
        await runner.command('estop')
        await runner.command('estop')
        boundaries = [event for event in runner.store.events if event['text'] == '착륙 시퀀스 시작']
        assert len(boundaries) == 1
        assert runner.store.events[-1]['text'] == '착륙 시퀀스 시작'
        await until(lambda: runner.store.run['state'] == 'ABORTED')
        assert sum(event['text'] == '착륙 시퀀스 시작' for event in runner.store.events) == 1
        await runner.close()
    asyncio.run(exercise())
