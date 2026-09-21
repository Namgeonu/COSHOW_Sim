"""Regressions for the final review's adapter and shutdown boundaries."""
import asyncio
import signal
from types import SimpleNamespace

import pytest

from dashboard.config import load_config
from dashboard.server import Dashboard
from dashboard.runner import Runner
from dashboard import ros_io
from dashboard.tests.test_runner import setup, launch, until


def test_reconfigure_live_child_is_rejected_before_old_adapter_is_closed(monkeypatch):
    async def run():
        app = Dashboard(load_config(mock=True))
        calls = []
        class Adapter:
            async def close(self): calls.append('old.close')
        app.io = Adapter()
        app.runner = Runner(app.cfg, app.store, app.io, app.state)
        app.runner.children['preflight'] = SimpleNamespace(returncode=None)
        monkeypatch.setattr(ros_io, 'ROSIO', lambda *args: calls.append('new.create'))
        with pytest.raises(ValueError, match='IDLE'):
            await app.reconfigure(app.cfg)
        assert calls == []
        assert app.runner.io is app.io
    asyncio.run(run())


@pytest.mark.parametrize('failure', ['start', 'close', None])
def test_reconfigure_failure_keeps_runner_on_a_live_adapter(monkeypatch, failure):
    async def run():
        app = Dashboard(load_config(mock=True))
        calls = []
        class Adapter:
            def __init__(self, label): self.label, self.closed = label, False
            async def start(self):
                calls.append(self.label + '.start')
                if failure == 'start': raise RuntimeError('new adapter startup failed')
            async def close(self):
                calls.append(self.label + '.close')
                self.closed = True
                if self.label == 'old' and failure == 'close':
                    raise RuntimeError('old adapter cleanup failed')
        old, new = Adapter('old'), Adapter('new')
        app.io = old
        app.runner = Runner(app.cfg, app.store, old, app.state)
        monkeypatch.setattr(ros_io, 'ROSIO', lambda *args: new)
        try:
            if failure == 'start':
                with pytest.raises(RuntimeError, match='startup failed'):
                    await app.reconfigure(app.cfg)
                assert not old.closed and app.io is old
            else:
                await app.reconfigure(app.cfg)
                assert old.closed and app.io is new
                assert calls.index('new.start') < calls.index('old.close')
            assert app.runner.io is app.io and not app.runner.io.closed
        finally:
            for task in app.tasks: task.cancel()
            await asyncio.gather(*app.tasks, return_exceptions=True)
    asyncio.run(run())


def test_shutdown_installs_both_signal_guards_before_awaiting_runner(monkeypatch):
    async def run():
        app = Dashboard(load_config(mock=True))
        handlers = {}
        monkeypatch.setattr(asyncio.get_running_loop(), 'add_signal_handler',
                            lambda sig, callback, *args: handlers.update({sig: (callback, args)}))
        class ClosingRunner:
            async def close(self):
                assert set(handlers) == {signal.SIGINT, signal.SIGTERM}
                for callback, args in handlers.values(): callback(*args)
        app.runner = ClosingRunner()
        await app.shutdown(None)
        warnings = [row['text'] for row in app.store.events if row['level'] == 'warning']
        assert len(warnings) == 2 and all('종료 중' in text for text in warnings)
    asyncio.run(run())


@pytest.mark.parametrize('cancellations', [1, 2])
def test_reconfigure_cancellation_after_swap_finishes_the_owned_transaction(monkeypatch, cancellations):
    async def run():
        app = Dashboard(load_config(mock=True))
        closing, release = asyncio.Event(), asyncio.Event()
        class Old:
            async def close(self):
                closing.set()
                await release.wait()
        class New:
            async def start(self): pass
            async def close(self): pass
        app.io = Old()
        app.runner = Runner(app.cfg, app.store, app.io, app.state)
        replacement = New()
        monkeypatch.setattr(ros_io, 'ROSIO', lambda *args: replacement)
        request = asyncio.create_task(app.reconfigure(app.cfg))
        await closing.wait()
        try:
            for _ in range(cancellations):
                request.cancel()
                await asyncio.sleep(.01)
            assert not request.done(), 'adapter replacement must finish before request cancellation returns'
            release.set()
            await request
            assert app.io is app.runner.io is replacement
            assert app.ping_task is not None
        finally:
            release.set()
            await asyncio.gather(request, return_exceptions=True)
            for task in app.tasks: task.cancel()
            await asyncio.gather(*app.tasks, return_exceptions=True)
    asyncio.run(run())


def test_control_allows_ros_adapter_to_finish_its_one_second_boundary(setup):
    async def run():
        runner, cfg, store, *_ = setup
        async def delivered(*args):
            await asyncio.sleep(1.03)
            return True
        runner.io = SimpleNamespace(land=delivered)
        assert await runner._control('land', cfg.drones[0], 0, 1) is True
        async def timed_out(*args): raise asyncio.TimeoutError()
        runner.io = SimpleNamespace(land=timed_out)
        assert await runner._control('land', cfg.drones[0], 0, 1) is None
        assert 'TimeoutError()' in store.events[-1]['text']
    asyncio.run(run())


def test_preflight_death_in_done_signals_live_bt_before_direct_land(setup):
    async def run():
        runner = await launch(setup)
        _, _, store, _, events, processes, _ = setup
        store.set_run(state='DONE')
        processes['preflight'].exit(1)
        try:
            await until(lambda: store.run['state'] == 'ABORTED', timeout=2)
            assert store.run['last_error'] == 'preflight 프로세스 사망'
            assert events[0][:3] == ('signal', 'bt', signal.SIGINT)
            assert sum(event[:3] == ('signal', 'bt', signal.SIGINT) for event in events) == 1
            assert sum(event[0] == 'land' for event in events) == 2
        finally:
            await runner.close()
    asyncio.run(run())



def test_repeated_cancellation_preserves_adapter_start_failure_for_fleet_rollback(monkeypatch, tmp_path):
    from dashboard.fleet import FleetManager
    import yaml
    async def run():
        app = Dashboard(load_config(mock=True))
        starting, release = asyncio.Event(), asyncio.Event()
        class Old:
            closed = False
            async def close(self): self.closed = True
        class Candidate:
            closed = False
            async def start(self):
                starting.set()
                await release.wait()
                raise RuntimeError('candidate startup failed')
            async def close(self): self.closed = True
        old, candidate = Old(), Candidate()
        app.io = old
        app.runner = Runner(app.cfg, app.store, old, app.state)
        monkeypatch.setattr(ros_io, 'ROSIO', lambda *args: candidate)
        manager = FleetManager(app.cfg, app.store, on_reconfigure=app.reconfigure,
                               run_dir=tmp_path, operation_guard=app.runner._require_idle)
        original_roster = dict(app.cfg.roster)
        original_files = {path: path.read_bytes() for path in tmp_path.iterdir() if path.is_file()}
        original_hash = manager.generated.roster_hash
        replacement = dict(original_roster)
        replacement[app.cfg.drones[0]] = app.cfg.raw['fleet']['drones'][6]['id']
        request = asyncio.create_task(manager.save_roster(replacement, expected_hash=original_hash))
        await starting.wait()
        try:
            assert yaml.safe_load(manager.roster_path.read_text()) == replacement
            for _ in range(2):
                request.cancel()
                await asyncio.sleep(.01)
            assert not request.done()
            release.set()
            with pytest.raises(RuntimeError, match='candidate startup failed'):
                await request
            assert app.runner.io is app.io is old
            assert not old.closed and candidate.closed
            assert app.cfg.roster == original_roster
            assert manager.generated.roster_hash == original_hash
            assert all(path.read_bytes() == content for path, content in original_files.items())
        finally:
            release.set()
            await asyncio.gather(request, return_exceptions=True)
            for task in app.tasks: task.cancel()
            await asyncio.gather(*app.tasks, return_exceptions=True)
    asyncio.run(run())
