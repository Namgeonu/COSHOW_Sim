"""Bounded system ping observations; no raw sockets and no shell commands."""
import asyncio
import ipaddress
import platform
import re


PING_TIMEOUT_S = 2.0


async def _reap(process):
    if process.returncode is None:
        try:
            process.kill()
        except ProcessLookupError:
            pass
    await process.communicate()


async def ping(ip):
    """Return a nullable observation for an unconfigured IP, otherwise a result."""
    result = {'ip': ip, 'ok': None if not ip else False, 'rtt_ms': None}
    if not ip:
        return result
    try:
        address = ipaddress.ip_address(ip)
    except ValueError:
        return result
    darwin = platform.system() == 'Darwin'
    wait = '1000' if darwin else '1'
    ipv6_darwin = darwin and address.version == 6
    command = ['ping6' if ipv6_darwin else 'ping', '-c', '1']
    # macOS ping6 has no numeric -W timeout; the outer deadline still bounds it.
    if not ipv6_darwin:
        command.extend(['-W', wait])
    command.append(str(address))
    process = None
    try:
        process = await asyncio.create_subprocess_exec(
            *command,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE)
        output, _ = await asyncio.wait_for(process.communicate(), timeout=PING_TIMEOUT_S)
        result['ok'] = process.returncode == 0
        match = re.search(r'time\s*[=<]\s*([0-9]+(?:\.[0-9]+)?)\s*ms', output.decode('utf-8', errors='replace'))
        if result['ok'] and match:
            result['rtt_ms'] = float(match.group(1))
    except asyncio.TimeoutError:
        await _reap(process)
    except asyncio.CancelledError:
        if process is not None:
            await asyncio.shield(_reap(process))
        raise
    except OSError:
        if process is not None:
            await _reap(process)
    return result


class Pinger:
    """Refresh every configured fleet IP without serializing slow peers."""
    def __init__(self, cfg, store):
        self.cfg = cfg
        self.store = store

    async def _one(self, name, ip):
        self.store.ping(name, await ping(ip))

    async def run(self):
        loop = asyncio.get_running_loop()
        period = float(self.cfg.raw.get('network', {}).get('ping_period_s', 2.0))
        period = max(0.1, period)
        while True:
            started = loop.time()
            tasks = [asyncio.create_task(self._one(name, metadata.get('ip')))
                     for name, metadata in self.cfg.robots.items()]
            try:
                await asyncio.gather(*tasks)
            finally:
                for task in tasks:
                    if not task.done():
                        task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)
            await asyncio.sleep(max(0.0, period - (loop.time() - started)))
