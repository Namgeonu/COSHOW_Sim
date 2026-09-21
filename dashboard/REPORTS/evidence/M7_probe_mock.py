"""Development harness: controls only a verified --mock server, not stage_probe."""
import asyncio
import json
from pathlib import Path
import sys
import time

import aiohttp


async def main():
    project = Path('/home/coshow/COSHOW')
    logs = project / 'dashboard/logs'
    with (logs / 'm7_probe_stdout.log').open('w') as output:
        process = await asyncio.create_subprocess_exec(
            sys.executable, str(project / 'dashboard/tests/stage_probe.py'),
            '--fleet', '--seconds', '106', '--record', str(logs / 'm7_stage_final.jsonl'),
            '--dump-poses', str(logs / 'm7_poses_final.csv'), stdout=output, stderr=asyncio.subprocess.STDOUT)
        try:
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect('http://127.0.0.1:8080/ws?role=admin') as socket:
                    hello = await socket.receive_json()
                    assert hello['mock'] is True, 'This test harness may only control a --mock server'
                    async def until_run(target):
                        deadline = time.monotonic() + 15
                        while True:
                            message = await socket.receive(timeout=2)
                            assert time.monotonic() < deadline, 'Mock state timed out: ' + target
                            if message.type != aiohttp.WSMsgType.TEXT:
                                continue
                            state = json.loads(message.data)
                            if state['type'] == 'state' and state['run']['state'] == target:
                                return
                    await socket.send_json({'cmd': 'reset'})
                    await until_run('IDLE')
                    await socket.send_json({'cmd': 'preflight'})
                    await until_run('READY')
                    await socket.send_json({'cmd': 'start'})
                    print('TEST HARNESS: verified mock server; preflight/start sent by harness only.', flush=True)
            assert await process.wait() == 0
        finally:
            if process.returncode is None:
                process.terminate()
                await process.wait()
    print('PASS genuine --mock timeline recorded through read-only stage_probe CLI', flush=True)


asyncio.run(main())
