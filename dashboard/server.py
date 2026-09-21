#!/usr/bin/env python3
"""Offline dashboard HTTP/WS and owned run lifecycle."""
import argparse
import asyncio
import contextlib
import json
import os
from pathlib import Path
import signal
import sys
import time

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from aiohttp import web, WSMsgType
from dashboard.config import load_config
from dashboard.state import TelemetryStore
from dashboard.checklist import evaluate, external_observation


class SocketSlots:
    """A single independent sender consumes replaceable, bounded latest slots."""
    def __init__(self, ws, drones, timeout=5.0, on_error=None):
        self.ws, self.count, self.timeout = ws, drones, timeout
        self.state, self.frames = None, {}
        self.on_error = on_error
        self.wake = asyncio.Event()

    def offer(self, state, frames):
        self.state = state
        self.frames.update({i: data for i, data in frames.items() if 0 <= i < self.count})
        self.wake.set()

    async def run(self):
        try:
            while not self.ws.closed:
                await self.wake.wait()
                self.wake.clear()
                state, frames = self.state, self.frames
                self.state, self.frames = None, {}
                if state is not None:
                    await asyncio.wait_for(self.ws.send_str(state), self.timeout)
                for payload in frames.values():
                    await asyncio.wait_for(self.ws.send_bytes(payload), self.timeout)
        except Exception as exc:
            try:
                if self.on_error:
                    self.on_error('WebSocket 송신 종료: {}: {}'.format(type(exc).__name__, exc))
            finally:
                with contextlib.suppress(Exception):
                    await asyncio.wait_for(self.ws.close(), self.timeout)


class Dashboard:
    def __init__(self, cfg, mock=False, mock_fail=False):
        self.cfg, self.mock = cfg, mock
        self.store = TelemetryStore(cfg)
        self.sockets, self.tasks = set(), []
        self.io, self.world = None, None
        self.runner = None
        self.fleet = None
        self.ping_task = None
        self.mock_fail = mock_fail
        self.mock_applied_hash = None
        self.mock_stack_running = True
        if mock:
            from dashboard.mock import MockWorld
            self.world = MockWorld(cfg, self.store, mock_fail)

    def state(self):
        snapshot = self.store.snapshot()
        ctx = self.store.checklist_context()
        now = self.store.clock()
        external = external_observation('bt', self.cfg, ctx, now)
        snapshot['run']['external_bt'] = external['active']
        snapshot['checklist'] = evaluate(snapshot, self.cfg, ctx, now)
        return snapshot

    async def aggregate(self):
        next_tick, last_frames, frame_at = time.monotonic(), {}, {}
        warned = False
        while True:
            try:
                if self.world:
                    self.world.tick()
                if self.fleet:
                    if self.mock:
                        self.store.set_stack(roster_hash=self.fleet.generated.roster_hash if self.fleet.generated else None,
                                             applied_hash=self.mock_applied_hash,
                                             crazyflie_server='up' if self.mock_stack_running else 'down',
                                             aideck='up' if self.mock_stack_running else 'down')
                    else:
                        self.fleet.refresh()
                state = json.dumps(self.state(), ensure_ascii=False, allow_nan=False)
                frames, now = {}, time.monotonic()
                period = 1 / max(.1, float(self.cfg.raw.get('frame_forward_max_fps', 15)))
                for name, (serial, jpeg) in self.store.latest_frames().items():
                    if name not in self.cfg.drones:
                        continue
                    if serial != last_frames.get(name) and now - frame_at.get(name, -float('inf')) >= period:
                        frames[self.cfg.drones.index(name)] = bytes([self.cfg.drones.index(name)]) + jpeg
                        last_frames[name], frame_at[name] = serial, now
                for slots in tuple(self.sockets):
                    slots.offer(state, frames)
            except Exception as exc:
                if not warned:
                    self.store.event('warning', '상태 집계 오류 (다음 tick 재시도): {}'.format(exc))
                    warned = True
            next_tick = max(next_tick + .1, time.monotonic())
            await asyncio.sleep(max(0, next_tick - time.monotonic()))

    async def command(self, payload, admin):
        cmd = payload.get('cmd') if isinstance(payload, dict) else None
        if not isinstance(cmd, str) or cmd not in ('preflight', 'start', 'estop', 'reset'):
            self.store.event('warning', 'cmd:unknown rejected(명령 형식 오류)')
            return
        accepted, reason = False, '관리자 연결만 허용'
        if admin:
            if self.store.stack.get('busy') and cmd in ('preflight', 'start'):
                self.store.event('warning', 'cmd:{} rejected(스택 기동 또는 설정 적용 중)'.format(cmd))
                return
            if self.world:
                if cmd == 'start' and any(r['blocking'] and not r['ok'] for r in self.state()['checklist']):
                    reason = 'blocking 점검 항목 실패'
                else:
                    accepted, reason = self.world.command(cmd)
            elif self.runner:
                accepted, reason = await self.runner.command(cmd)
            else:
                reason = '실행 제어 초기화 대기'
        text = 'cmd:{} {}'.format(cmd, 'accepted' if accepted else 'rejected(' + reason + ')')
        self.store.event('info' if accepted else 'warning', text)

    async def websocket(self, request):
        role = request.query.get('role', 'visitor')
        if role == 'admin':
            self.require_local(request)
        ws = web.WebSocketResponse(max_msg_size=4096, heartbeat=20)
        await ws.prepare(request)
        slots, sender = SocketSlots(ws, len(self.cfg.drones),
                                     on_error=lambda text: self.store.event('warning', text)), None
        try:
            await asyncio.wait_for(ws.send_json(self.cfg.hello(self.mock)), 5)
            self.sockets.add(slots)
            sender = asyncio.create_task(slots.run())
            sender.add_done_callback(lambda task: self.sockets.discard(slots))
            async for message in ws:
                if message.type == WSMsgType.TEXT:
                    try:
                        payload = json.loads(message.data)
                    except (json.JSONDecodeError, ValueError):
                        payload = None
                    await self.command(payload, role == 'admin')
        finally:
            self.sockets.discard(slots)
            if sender:
                sender.cancel()
                await asyncio.gather(sender, return_exceptions=True)
        return ws

    async def startup(self, app):
        if not self.mock:
            from dashboard.ros_io import ROSIO
            from dashboard.pinger import Pinger
            try:
                self.io = ROSIO(self.cfg, self.store)
                await self.io.start()
            except Exception as exc:
                self.store.unavailable(None, 'ros', 'ROS 시작 실패: {}'.format(exc))
            self.ping_task = asyncio.create_task(Pinger(self.cfg, self.store).run())
            self.tasks.append(self.ping_task)
            from dashboard.runner import Runner
            self.runner = Runner(self.cfg, self.store, self.io, self.state)
            await self.runner.start()
        from dashboard.fleet import FleetManager
        self.fleet = FleetManager(self.cfg, self.store, self.reconfigure,
                                  operation_guard=self.runner._require_idle if self.runner else None)
        if not self.mock:
            await self.fleet.recover_stacks()
        if self.mock and self.fleet.generated:
            self.mock_applied_hash = self.fleet.generated.roster_hash
        self.tasks.append(asyncio.create_task(self.aggregate()))

    async def shutdown(self, app):
        # aiohttp's default second signal raises GracefulExit. Once shutdown has
        # begun, only the server-owned safety task may decide when to finish.
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self.store.event, 'warning',
                '서버 종료 중 {} 무시 — 착륙 시퀀스 계속'.format(sig.name))
        # on_shutdown runs before aiohttp waits for open WebSocket handlers.
        # Keep ROS, receipt aging and telemetry alive throughout the sequence.
        if self.runner:
            await self.runner.close()
        if self.fleet:
            await self.fleet.close()
        await asyncio.gather(*(s.ws.close() for s in tuple(self.sockets)), return_exceptions=True)

    async def cleanup(self, app):
        if self.runner:
            await self.runner.close()
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        await asyncio.gather(*(s.ws.close() for s in tuple(self.sockets)), return_exceptions=True)
        if self.io:
            await self.io.close()

    @staticmethod
    def require_local(request):
        if request.remote not in ('127.0.0.1', '::1'):
            raise web.HTTPForbidden(text='관리자는 로컬 연결만 허용합니다')
        origin = request.headers.get('Origin')
        if origin and origin != '{}://{}'.format(request.scheme, request.host):
            raise web.HTTPForbidden(text='다른 사이트에서 보낸 관리자 요청은 허용하지 않습니다')

    async def admin_settings(self, request):
        self.require_local(request)
        return web.json_response(dict(landed_z=(self.cfg.bt.get('coshow', {}).get('tolerances') or {}).get('landed_z'),
            fleet=self.cfg.raw.get('fleet', {}), roster=self.cfg.roster,
            robots=self.cfg.robots, mock=self.mock,
            roster_hash=self.fleet.generated.roster_hash if self.fleet and self.fleet.generated else None))

    async def reconfigure(self, cfg):
        if self.runner:
            self.runner._require_idle()
        # Once Fleet has staged its files, finish or roll back as one operation.
        # A cancelled request must not restore old files after the adapter swap.
        transaction = asyncio.create_task(self._reconfigure(cfg))
        while True:
            try:
                return await asyncio.shield(transaction)
            except asyncio.CancelledError:
                # Repeated caller cancellation must not reach an adapter that
                # has already replaced the old one. Preserve the actual outcome.
                if transaction.done():
                    return transaction.result()

    async def _reconfigure(self, cfg):
        old_io, candidate = self.io, None
        if not self.mock:
            from dashboard.ros_io import ROSIO
            candidate = ROSIO(cfg, self.store)
            try:
                await candidate.start()
                # A shutdown may have started while ROS initialization awaited.
                self.runner.reconfigure()
            except BaseException:
                with contextlib.suppress(Exception):
                    await candidate.close()
                self.runner.io = self.io
                raise
            # No await between publishing the new adapter and the Runner's
            # reference. Even failure while closing the old one keeps control.
            self.io = candidate
            self.runner.io = self.io
        if self.ping_task:
            self.ping_task.cancel()
            await asyncio.gather(self.ping_task, return_exceptions=True)
            if self.ping_task in self.tasks:
                self.tasks.remove(self.ping_task)
        try:
            if old_io:
                await old_io.close()
        except Exception as exc:
            self.store.event('warning', '이전 ROS 어댑터 정리 미확인: {!r}'.format(exc))
        finally:
            if self.runner:
                self.runner.io = self.io
        # Old executor/ping callbacks can finish during shutdown; clear those
        # receipts after both are stopped so a replacement never inherits them.
        with self.store.lock:
            self.store.data = {name: {} for name in cfg.robots}
            self.store.frames.clear()
            self.store.reset_cached()
            self.store.context['interface_errors'].clear()
        if self.mock:
            from dashboard.mock import MockWorld
            self.world = MockWorld(cfg, self.store, self.mock_fail)
        else:
            from dashboard.pinger import Pinger
            self.ping_task = asyncio.create_task(Pinger(cfg, self.store).run())
            self.tasks.append(self.ping_task)
        # Reconnect gives each sender a new camera-index count and hello.
        await asyncio.gather(*(slots.ws.close() for slots in tuple(self.sockets)), return_exceptions=True)

    async def fleet_action(self, request):
        self.require_local(request)
        if not self.fleet:
            raise web.HTTPServiceUnavailable(text='플릿 제어 초기화 대기')
        action = request.match_info['action']
        try:
            if action == 'recommend':
                result = self.fleet.recommend_roster()
            elif action == 'roster':
                payload = await request.json()
                if not isinstance(payload, dict) or 'expected_hash' not in payload:
                    raise ValueError('expected_hash가 필요합니다. 최신 배정을 다시 검토하세요')
                result = await self.fleet.save_roster(payload.get('roster'), payload.get('expected_hash'))
            elif action in ('start', 'restart', 'stop'):
                if self.mock:
                    async with self.fleet._operation():
                        if action == 'stop':
                            self.mock_stack_running = False
                            self.mock_applied_hash = None
                        else:
                            generated = self.fleet.regenerate()
                            if generated is None:
                                raise ValueError(self.store.stack.get('generation_error'))
                            self.mock_stack_running = True
                            self.mock_applied_hash = generated.roster_hash
                        self.store.set_stack(applied_hash=self.mock_applied_hash,
                            crazyflie_server='up' if self.mock_stack_running else 'down',
                            aideck='up' if self.mock_stack_running else 'down')
                        self.store.event('info', 'MOCK 스택 {} 완료'.format(action))
                        result = self.store.snapshot()['stack']
                else:
                    operation = {'start': self.fleet.start_stack, 'restart': self.fleet.restart_stack,
                                 'stop': self.fleet.stop_stack}[action]
                    result = await operation()
            else:
                raise web.HTTPNotFound()
        except (ValueError, TypeError, AttributeError, OSError) as exc:
            self.store.event('warning', '플릿 요청 거부: {}'.format(exc))
            return web.json_response(dict(ok=False, error=str(exc)), status=409)
        return web.json_response(dict(ok=True, result=result))

    def app(self):
        app = web.Application()
        app.router.add_get('/ws', self.websocket)
        async def visitor(request):
            return web.FileResponse(Path(__file__).resolve().parent / 'static/visitor.html')
        app.router.add_get('/visitor.html', visitor)
        async def admin(request):
            self.require_local(request)
            return web.FileResponse(Path(__file__).resolve().parent / 'static/admin.html')
        app.router.add_get('/admin.html', admin)
        app.router.add_get('/api/admin', self.admin_settings)
        app.router.add_get('/api/fleet/{action:recommend}', self.fleet_action)
        app.router.add_put('/api/fleet/{action:roster}', self.fleet_action)
        app.router.add_post('/api/fleet/{action:start|restart|stop}', self.fleet_action)
        async def health(request):
            return web.json_response(dict(milestone='M8', mock=self.mock, websocket='/ws', visitor='/visitor.html',
                                           admin='/admin.html', mode='mock' if self.mock else 'operations'))
        app.router.add_get('/', health)
        app.router.add_static('/static/', Path(__file__).resolve().parent / 'static', show_index=False)
        app.on_startup.append(self.startup)
        app.on_shutdown.append(self.shutdown)
        app.on_cleanup.append(self.cleanup)
        return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', help='dashboard.yaml path')
    parser.add_argument('--field', help='field.yaml path')
    parser.add_argument('--mock', action='store_true')
    parser.add_argument('--mock-fail', action='store_true')
    parser.add_argument('--check-config', action='store_true')
    parser.add_argument('--check-timeout', type=float, default=3)
    parser.add_argument('--host')
    parser.add_argument('--port', type=int)
    args = parser.parse_args()
    cfg = load_config(args.config, args.field, mock=args.mock)
    host = args.host or cfg.raw.get('http', {}).get('host', '127.0.0.1')
    port = args.port if args.port is not None else cfg.raw.get('http', {}).get('port', 8080)
    print('ROS_DOMAIN_ID={} RMW_IMPLEMENTATION={} bind={}:{} mock={}'.format(
        os.environ.get('ROS_DOMAIN_ID', '(default)'), os.environ.get('RMW_IMPLEMENTATION', '(default)'),
        host, port, args.mock), flush=True)
    if args.check_config:
        from dashboard.ros_io import check_config
        from dashboard.fleet import generate_config
        generation_failed = False
        try:
            generated = generate_config(cfg)
            print('PASS\tgeneration\t{}\t{} files'.format(generated.roster_hash, len(generated.files)))
            for warning in generated.warnings:
                if warning not in cfg.warnings:
                    cfg.warnings.append(warning)
        except Exception as exc:
            generation_failed = True
            print('FAIL\tgeneration\t{!r}'.format(exc))
        rows = check_config(cfg, args.check_timeout)
        for row in rows:
            print('{}\t{}\t{}\t{}\t{}'.format(row.get('status', 'pass' if row['ok'] else 'fail').upper(), row['kind'],
                row['name'], row['expected_type'], row['detail']))
        for error in cfg.errors:
            print('FAIL\tconfiguration\t' + error)
        for warning in cfg.warnings:
            print('WARN\tconfiguration\t' + warning)
        return 1 if generation_failed or cfg.errors or any(not r['ok'] and r.get('status') != 'skip' for r in rows) else 0
    web.run_app(Dashboard(cfg, args.mock, args.mock_fail).app(), host=host, port=port)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
