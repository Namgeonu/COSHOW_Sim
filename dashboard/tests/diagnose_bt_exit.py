#!/usr/bin/env python3
"""Compare SIGINT teardown on unmodified and instrumented BT in Docker only."""
import argparse
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
import yaml

from verify_m1_ros import EvidenceNode


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bt-dir', required=True)
    parser.add_argument('--repeat', type=int, default=3)
    args = parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    bt = Path(args.bt_dir).resolve()
    config = yaml.safe_load((bt / 'scenarios/coshow/configs/coshow_rehearsal.yaml').read_text())
    config['bt_runner']['bt_visualiser']['enabled'] = False
    env = dict(os.environ, SDL_VIDEODRIVER='dummy', SDL_AUDIODRIVER='dummy',
               PYGAME_HIDE_SUPPORT_PROMPT='1', PYTHONUNBUFFERED='1')
    rclpy.init()
    node = EvidenceNode(config)
    node.publish_poses = True
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    with tempfile.TemporaryDirectory(prefix='bt-exit-diagnostic-') as temporary:
        temp = Path(temporary)
        config_file = temp / 'bt.yaml'
        config_file.write_text(yaml.safe_dump(config))
        for attempt in range(args.repeat):
            with (temp / 'bt.log').open('w+') as log:
                process = subprocess.Popen(
                    [sys.executable, 'main.py', '--config', str(config_file)],
                    cwd=str(bt), env=env, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                time.sleep(3)
                start = time.monotonic()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=float(config['coshow']['durations']['land']) + 4)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                log.seek(0)
                print('bt={} attempt={} rc={} after_SIGINT={:.3f}s'.format(
                    bt, attempt + 1, process.returncode, time.monotonic() - start), flush=True)
                print(log.read(), flush=True)
    executor.shutdown()
    thread.join(timeout=2)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
