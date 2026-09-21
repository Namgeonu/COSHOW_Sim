"""Role-only emergency controls exercised against isolated real DDS services."""
import asyncio
import threading
import time
from types import SimpleNamespace

import pytest

from dashboard.ros_io import ROSIO, interface_specs
from dashboard.tests.test_ros_io import config, Store, wait_until


class ControlStore(Store):
    def __init__(self):
        super().__init__()
        self.events = []

    def event(self, level, text):
        self.events.append((level, text))


@pytest.fixture
def controls(config):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rosidl_runtime_py.utilities import get_service

    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node(config.raw['nodes']['server'], context=context)
    records, behavior = [], {}
    services = []
    group = ReentrantCallbackGroup()

    def handler(robot, channel):
        def receive(request, response):
            records.append((robot, channel, request, time.monotonic(), threading.get_ident()))
            delay, reject = behavior.get((robot, channel), (0, False))
            if delay:
                time.sleep(delay)
            if channel == 'cancel':
                response.return_code = response.ERROR_REJECTED if reject else response.ERROR_NONE
            return response
        return receive

    for spec in interface_specs(config)[0]:
        if spec['kind'] == 'services':
            services.append(node.create_service(get_service(spec['type']), spec['name'],
                            handler(spec['robot'], spec['channel']), callback_group=group))
        elif spec['kind'] == 'actions':
            services.append(node.create_service(get_service('action_msgs/srv/CancelGoal'),
                            spec['name'].rstrip('/') + '/_action/cancel_goal',
                            handler(spec['robot'], 'cancel'), callback_group=group))
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    executor.add_node(node)
    stop = threading.Event()

    def spin():
        while not stop.is_set():
            executor.spin_once(timeout_sec=.02)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    yield SimpleNamespace(records=records, behavior=behavior, node=node, services=services)
    stop.set()
    thread.join(2)
    executor.shutdown(timeout_sec=3)
    node.destroy_node()
    context.shutdown()


async def ready(io):
    await io.start()
    await wait_until(lambda: all(c.service_is_ready() for c in io.service_clients.values()))


def test_control_methods_are_available_without_ros_import_at_module_load(config):
    io = ROSIO(config, ControlStore())
    assert all(callable(getattr(io, name, None)) for name in ('land', 'arm', 'cancel'))


def test_real_requests_use_configured_role_names_and_correct_wire_shapes(config, controls):
    async def exercise():
        io = ROSIO(config, ControlStore())
        await ready(io)
        try:
            assert await io.land('observer', .04, 2.125) is True
            assert await io.land('seeker', .02, .9999999996) is True
            assert await io.arm('observer', False) is True
            assert await io.cancel('carrier') is True
            requests = {(robot, channel): request for robot, channel, request, _, _ in controls.records}
            first = requests[('observer', 'land')]
            assert first.height == pytest.approx(.04)
            assert first.group_mask == 0
            assert (first.duration.sec, first.duration.nanosec) == (2, 125000000)
            rounded = requests[('seeker', 'land')].duration
            assert (rounded.sec, rounded.nanosec) == (1, 0)
            assert requests[('observer', 'arm')].arm is False
            goal = requests[('carrier', 'cancel')].goal_info
            assert list(goal.goal_id.uuid) == [0] * 16
            assert (goal.stamp.sec, goal.stamp.nanosec) == (0, 0)
            assert all(tid != threading.get_ident() for *_, tid in controls.records)
            assert not any(robot in ('reserve', 'reserve_car') for robot, _ in io.service_clients)
        finally:
            await io.close()
    asyncio.run(exercise())


def test_spares_wrong_kinds_and_arming_cannot_send_control_requests(config, controls):
    async def exercise():
        store = ControlStore()
        io = ROSIO(config, store)
        await ready(io)
        try:
            assert await io.land('reserve', .04, 2) is False
            assert await io.land('carrier', .04, 2) is False
            assert await io.arm('reserve', False) is False
            assert await io.arm('observer', True) is False
            assert await io.arm('observer', 0) is False
            assert await io.cancel('reserve_car') is False
            assert await io.cancel('observer') is False
            assert controls.records == []
            assert len(store.events) == 7
        finally:
            await io.close()
    asyncio.run(exercise())


@pytest.mark.parametrize('height,duration', [
    (-1, 2), (float('nan'), 2), (True, 2), (1, 0), (1, -1),
    (1, float('inf')), (1, '2'), (1, 2147483648),
])
def test_invalid_land_parameters_never_reach_ros(config, controls, height, duration):
    async def exercise():
        io = ROSIO(config, ControlStore())
        await ready(io)
        try:
            assert await io.land('observer', height, duration) is False
            assert controls.records == []
        finally:
            await io.close()
    asyncio.run(exercise())


def test_cancel_rejection_warns_and_does_not_interrupt_other_controls(config, controls):
    async def exercise():
        store = ControlStore()
        io = ROSIO(config, store)
        controls.behavior[('carrier', 'cancel')] = (0, True)
        await ready(io)
        try:
            assert await io.cancel('carrier') is False
            assert any(level == 'warning' and 'carrier 취소 미확인' in text for level, text in store.events)
            assert await io.land('observer', .04, 2) is True
        finally:
            await io.close()
    asyncio.run(exercise())


def test_timeout_is_bounded_and_keeps_asyncio_responsive_then_late_reply_is_safe(config, controls):
    async def exercise():
        store = ControlStore()
        io = ROSIO(config, store)
        controls.behavior[('carrier', 'cancel')] = (1.3, False)
        await ready(io)
        try:
            ticks = []
            async def heartbeat():
                for _ in range(24):
                    ticks.append(time.monotonic())
                    await asyncio.sleep(.025)
            started = time.monotonic()
            result, _ = await asyncio.gather(io.cancel('carrier'), heartbeat())
            assert result is False
            assert .9 <= time.monotonic() - started <= 1.15
            assert len(ticks) == 24 and max(b - a for a, b in zip(ticks, ticks[1:])) < .12
            assert any('carrier 취소 미확인' in text for _, text in store.events)
            await asyncio.sleep(.4)
            assert await io.land('observer', .04, 2) is True
        finally:
            await io.close()
    asyncio.run(exercise())


def test_missing_client_unavailable_service_and_closed_adapter_skip_with_warning(config):
    async def exercise():
        store = ControlStore()
        io = ROSIO(config, store)
        await io.start()
        try:
            started = time.monotonic()
            assert await io.land('observer', .04, 2) is None
            io.service_clients.pop(('observer', 'arm'))
            assert await io.arm('observer', False) is None
            assert await io.cancel('carrier') is None
            assert time.monotonic() - started < .3
            assert len(store.events) == 3
        finally:
            await io.close()
        assert await io.land('observer', .04, 2) is None
        assert len(store.events) == 4
    asyncio.run(exercise())


def test_service_call_exception_returns_false_and_asyncio_cancel_removes_pending_request(
        config, controls, monkeypatch):
    async def exercise():
        store = ControlStore()
        io = ROSIO(config, store)
        await ready(io)
        try:
            client = io.service_clients[('observer', 'land')]
            def fail(request):
                raise RuntimeError('DDS send failed')
            monkeypatch.setattr(client, 'call_async', fail)
            assert await io.land('observer', .04, 2) is False
            assert any('DDS send failed' in text for _, text in store.events)
            controls.behavior[('carrier', 'cancel')] = (1.3, False)
            request = asyncio.create_task(io.cancel('carrier'))
            await wait_until(lambda: bool(controls.records))
            request.cancel()
            with pytest.raises(asyncio.CancelledError):
                await request
            assert not io._control_pending
            await asyncio.sleep(.02)
        finally:
            await io.close()
    asyncio.run(exercise())


def test_close_while_request_pending_is_idempotent_and_unblocks_waiter(config, controls):
    async def exercise():
        io = ROSIO(config, ControlStore())
        controls.behavior[('carrier', 'cancel')] = (1.3, False)
        await ready(io)
        try:
            request = asyncio.create_task(io.cancel('carrier'))
            await wait_until(lambda: bool(controls.records))
            await asyncio.gather(io.close(), io.close())
            assert await request is None
            assert not io.thread.is_alive()
        finally:
            await io.close()
    asyncio.run(exercise())
