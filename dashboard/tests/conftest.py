"""M1 tests import the production BT module with its real ROS dependencies."""
import importlib
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'bt'))


@pytest.fixture(scope='session')
def bt_nodes():
    from modules import utils

    utils.set_config(str(ROOT / 'bt/scenarios/coshow/configs/coshow_rehearsal.yaml'))
    return importlib.import_module('scenarios.coshow.bt_nodes')


@pytest.fixture
def home_bb(bt_nodes):
    """All robots at configured bases; mission has not yet been read."""
    return {
        'now': 100.0,
        'pose': {name: {'x': cfg['base'][0], 'y': cfg['base'][1], 'z': 0.0}
                 for name, cfg in bt_nodes.DRONES.items()},
        'missing_pose': [], 'preflight_ready': True,
        'mission_marker': {'found': False}, 'target_marker': {'found': False},
        'target_confirmed': False, 'rescue_done_t': 0.0,
        'limo_arrived': {name: {'goal': tuple(cfg['base']), 't': 0.0}
                         for name, cfg in bt_nodes.LIMOS.items()},
    }
