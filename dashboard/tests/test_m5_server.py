import asyncio
from types import SimpleNamespace
import pytest
from aiohttp import web
from dashboard.config import load_config
from dashboard.server import Dashboard

def test_real_commands_delegate_only_for_admin_and_shutdown_precedes_adapter_close():
    async def run():
        app=Dashboard(load_config(mock=True))
        calls=[]
        class Runner:
            async def command(self,cmd):calls.append(cmd);return True,''
            async def close(self):calls.append('runner.close')
        class Adapter:
            async def close(self):calls.append('io.close')
        app.runner=Runner();app.io=Adapter()
        await app.command({'cmd':'preflight'},False)
        assert calls==[]
        await app.command({'cmd':'preflight'},True)
        assert calls==['preflight']
        await app.shutdown(None)
        await app.cleanup(None)
        assert calls.index('runner.close')<calls.index('io.close')
    asyncio.run(run())

def test_repeated_noop_commands_each_have_an_ack_event():
    async def run():
        app=Dashboard(load_config(mock=True),mock=True)
        await app.command({'cmd':'reset'},True)
        await app.command({'cmd':'reset'},True)
        assert sum(e['text']=='cmd:reset accepted' for e in app.store.events)==2
    asyncio.run(run())

def test_admin_settings_are_local_and_preserve_hello_schema():
    async def run():
        app=Dashboard(load_config(mock=True),mock=True)
        for request in [SimpleNamespace(remote='10.0.0.2',headers={},scheme='http',host='127.0.0.1'),
                        SimpleNamespace(remote='127.0.0.1',headers={'Origin':'https://elsewhere.invalid'},scheme='http',host='127.0.0.1')]:
            with pytest.raises(web.HTTPForbidden):await app.admin_settings(request)
        response=await app.admin_settings(SimpleNamespace(remote='127.0.0.1',headers={},scheme='http',host='127.0.0.1'))
        assert response.status==200
        assert len(app.cfg.hello())==16
    asyncio.run(run())
