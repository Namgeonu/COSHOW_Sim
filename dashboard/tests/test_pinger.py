"""System ping is bounded, shell-free, and reaps children on cancellation."""
import asyncio
from types import SimpleNamespace

import pytest

from dashboard import pinger


class Process:
    def __init__(self, output=b'64 bytes: time=2.34 ms\n', returncode=0, stall=False):
        self.output, self.result_code, self.stall = output, returncode, stall
        self.returncode = None
        self.killed = False
        self.communications = 0

    async def communicate(self):
        self.communications += 1
        if self.stall and not self.killed:
            await asyncio.Future()
        self.returncode = -9 if self.killed else self.result_code
        return self.output, b''

    def kill(self):
        self.killed = True
        self.returncode = -9


@pytest.mark.parametrize('ip', [None, '', '-f', '127.0.0.1; touch example'])
def test_missing_or_invalid_ip_never_spawns(monkeypatch, ip):
    async def forbidden(*args, **kwargs):
        raise AssertionError('invalid IP reached subprocess')
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', forbidden)
    result = asyncio.run(pinger.ping(ip))
    assert result['ok'] is (None if not ip else False)
    assert result['rtt_ms'] is None


@pytest.mark.parametrize('system,wait', [('Linux', '1'), ('Darwin', '1000')])
def test_ping_uses_platform_timeout_and_no_shell(monkeypatch, system, wait):
    calls = []
    async def spawn(*args, **kwargs):
        calls.append((args, kwargs))
        return Process()
    monkeypatch.setattr(pinger.platform, 'system', lambda: system)
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', spawn)
    result = asyncio.run(pinger.ping('127.0.0.1'))
    assert result == {'ip': '127.0.0.1', 'ok': True, 'rtt_ms': 2.34}
    assert calls[0][0] == ('ping', '-c', '1', '-W', wait, '127.0.0.1')
    assert 'shell' not in calls[0][1]


def test_darwin_ipv6_uses_outer_timeout_without_ipv4_wait_option(monkeypatch):
    calls = []
    async def spawn(*args, **kwargs):
        calls.append(args)
        return Process()
    monkeypatch.setattr(pinger.platform, 'system', lambda: 'Darwin')
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', spawn)
    assert asyncio.run(pinger.ping('::1'))['ok'] is True
    # Apple's ping6 -W is a flag, unlike ping -W <milliseconds>.
    assert calls == [('ping6', '-c', '1', '::1')]


@pytest.mark.parametrize('output,code,rtt', [
    (b'request timeout', 1, None), (b'time<1 ms', 0, 1.), (b'localized output', 0, None)])
def test_exit_code_is_authoritative_even_when_rtt_is_missing(monkeypatch, output, code, rtt):
    async def spawn(*args, **kwargs):
        return Process(output, code)
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', spawn)
    result = asyncio.run(pinger.ping('127.0.0.1'))
    assert result['ok'] is (code == 0)
    assert result['rtt_ms'] == rtt


def test_missing_ping_executable_is_a_failed_observation(monkeypatch):
    async def spawn(*args, **kwargs):
        raise FileNotFoundError('ping')
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', spawn)
    assert asyncio.run(pinger.ping('127.0.0.1'))['ok'] is False


def test_timeout_kills_and_reaps_ping(monkeypatch):
    process = Process(stall=True)
    async def spawn(*args, **kwargs):
        return process
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', spawn)
    monkeypatch.setattr(pinger, 'PING_TIMEOUT_S', .01)
    assert asyncio.run(pinger.ping('127.0.0.1'))['ok'] is False
    assert process.killed and process.communications == 2


def test_cancellation_kills_and_reaps_ping(monkeypatch):
    process = Process(stall=True)
    async def spawn(*args, **kwargs):
        return process
    monkeypatch.setattr(pinger.asyncio, 'create_subprocess_exec', spawn)
    async def exercise():
        task = asyncio.create_task(pinger.ping('127.0.0.1'))
        while not process.communications:
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(exercise())
    assert process.killed and process.communications == 2


def test_poller_observes_every_configured_robot_and_cancels_cleanly(monkeypatch):
    observations = {}
    async def fake_ping(ip):
        return {'ip': ip, 'ok': bool(ip), 'rtt_ms': None}
    monkeypatch.setattr(pinger, 'ping', fake_ping)
    cfg = SimpleNamespace(raw={'network': {'ping_period_s': 2.}},
                          robots={'role': {'ip': '127.0.0.1'}, 'spare': {'ip': None}})
    store = SimpleNamespace(ping=lambda name, result: observations.update({name: result}))
    async def exercise():
        task = asyncio.create_task(pinger.Pinger(cfg, store).run())
        while len(observations) < 2:
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    asyncio.run(exercise())
    assert observations['role']['ok'] is True
    assert observations['spare']['ip'] is None
