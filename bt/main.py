import asyncio
import argparse
import cProfile
import signal

from modules.utils import set_config

# Parse command line arguments
parser = argparse.ArgumentParser(description='py_bt_ros')
parser.add_argument('--config', type=str, default='scenarios/coshow/configs/coshow_sim.yaml', help='Path to the configuration file (default: --config=scenarios/coshow/configs/coshow_sim.yaml)')
parser.add_argument('--ns', type=str, default=None, help='Override agent namespace, e.g. --ns /Fire_UGV_2')
# 투입 기체 선택 (coshow slots). 지정하지 않으면 config 의 roster.enabled 에 따라 4칸 프롬프트를 띄운다.
parser.add_argument('--observe', type=str, default=None, help='Mission Drone, 예: --observe cf235')
parser.add_argument('--searchers', type=str, default=None, help='Search Drone a,b,c (쉼표 구분), 예: --searchers cf230,cf232,cf237')
parser.add_argument('--roster', action='store_true', help='config 와 무관하게 기체 선택 프롬프트를 띄운다')
parser.add_argument('--no-roster', action='store_true', help='프롬프트를 띄우지 않고 slots.default 명단을 쓴다')
args = parser.parse_args()

# Load configuration and initialize the environment
set_config(args.config)
from modules.utils import config
if args.ns is not None:
    config['agent']['namespaces'] = args.ns
# 기체 명단은 반드시 BTRunner import 전에 확정한다 (bt_nodes 가 import 시점에 config 를 읽는다).
# asyncio 루프 밖이라 input() 이 tick 을 막지 않는다.
from scenarios.coshow.roster import apply_roster
apply_roster(config,
             observe=args.observe,
             searchers=args.searchers.split(',') if args.searchers else None,
             interactive=not args.no_roster,
             force_prompt=args.roster)
# 사전점검 게이트 (preflight.gate). 점검표 브리핑 → y/n 승인 → 결함 승인 시 config 에
# degraded 를 채워 bt_nodes 가 t=0 부터 재배치를 발동한다. BTRunner import 전이어야 한다.
from scenarios.coshow.preflight_gate import run_gate
run_gate(config)
from modules.bt_runner import BTRunner
bt_runner = BTRunner(config)


async def loop():
    # SIGTERM → running=False → loop 종료 → finally에서 close() 호출
    asyncio.get_event_loop().add_signal_handler(
        signal.SIGTERM, lambda: setattr(bt_runner, 'running', False)
    )
    try:
        while bt_runner.running:
            bt_runner.handle_keyboard_events()
            if not bt_runner.paused:
                await bt_runner.step()
            bt_runner.render()
    finally:
        bt_runner.close()  # halt_tree() → cancel active ROS Action goals


if __name__ == "__main__":
    if config['bt_runner']['profiling_mode']:
        cProfile.run('main()', sort='cumulative')
    else:
        try:
            asyncio.run(loop())
        except KeyboardInterrupt:
            pass  # bt_runner.close()는 loop() finally에서 이미 호출됨