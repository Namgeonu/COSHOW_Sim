"""A failed stack PID write must expose ownership without killing the process."""
import asyncio
import os
from pathlib import Path
import sys

import pytest

from dashboard.runner import spawn_process


def test_stack_pid_write_failure_retains_owned_process(tmp_path, monkeypatch):
    async def run():
        pid_path = tmp_path / 'stack.pid'
        original = Path.write_text
        def write(path, *args, **kwargs):
            if path == pid_path:
                raise OSError('fixture pid storage unavailable')
            return original(path, *args, **kwargs)
        monkeypatch.setattr(Path, 'write_text', write)
        child = None
        try:
            with pytest.raises(OSError, match='pid storage') as raised:
                await spawn_process([sys.executable, '-c', 'import time; time.sleep(60)'],
                    tmp_path, {}, tmp_path / 'stack.log', pid_path, preserve_on_error=True)
            child = raised.value.child
            assert child.returncode is None and not child.sigint_sent
            os.kill(child.pid, 0)
        finally:
            # Test-only cleanup of the exact process this test created.
            if child is not None and child.returncode is None:
                child.proc.kill()
                await child.proc.wait()
    asyncio.run(run())
