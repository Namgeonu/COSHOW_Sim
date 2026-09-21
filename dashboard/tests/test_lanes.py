"""The standalone dashboard path helper must stay identical to BT paths."""
import copy
import importlib
import importlib.util

import pytest


@pytest.fixture
def lanes():
    if importlib.util.find_spec('dashboard.lanes') is not None:
        return importlib.import_module('dashboard.lanes')
    return None


def test_rehearsal_matches_actual_bt_lawnmower(bt_nodes, lanes):
    assert lanes is not None, 'dashboard.lanes is missing'
    config = copy.deepcopy(bt_nodes.config)
    before = copy.deepcopy(config)
    actual = lanes.lanes_from_config(config)
    search = config['coshow']['search']
    expected = {name: bt_nodes._lawnmower(search['zones'][name], search['lane_spacing'],
                                        search['lane_axis'], robot=name)
                for name in config['coshow']['searchers']}
    assert actual == expected
    assert config == before
    assert [actual[name][0][2] for name in expected] == [0.4, 0.8, 1.2]
    assert all(len(points) == 2 for points in actual.values())


@pytest.mark.parametrize('axis', ['x', 'y'])
@pytest.mark.parametrize('spacing', [0.4, 0.5, 2.0, 9.0])
@pytest.mark.parametrize('robot', [None, 'cf231', 'cf232', 'cf233'])
def test_paths_match_original_with_axes_rounding_and_overrides(bt_nodes, lanes, axis, spacing, robot):
    assert lanes is not None, 'dashboard.lanes is missing'
    zone = {'x': [-2.0, 3.0], 'y': [-1.0, 1.2]}
    expected = bt_nodes._lawnmower(zone, spacing, axis, robot)
    altitude = bt_nodes._alt('search', robot)
    assert lanes.lawnmower(zone, spacing, axis, altitude) == expected


def test_lanes_config_global_fallback_and_axis(bt_nodes, lanes, monkeypatch):
    assert lanes is not None, 'dashboard.lanes is missing'
    config = copy.deepcopy(bt_nodes.config)
    cfg = config['coshow']
    cfg['search']['lane_axis'] = 'x'
    del cfg['drones']['cf231']['altitudes']['search']
    cfg['altitudes']['search'] = 1.7
    monkeypatch.setattr(bt_nodes, 'C', cfg)
    monkeypatch.setattr(bt_nodes, 'DRONES', cfg['drones'])
    expected = {name: bt_nodes._lawnmower(cfg['search']['zones'][name],
                                        cfg['search']['lane_spacing'], 'x', name)
                for name in cfg['searchers']}
    assert lanes.lanes_from_config(config) == expected


def test_lanes_module_imports_without_ros_or_bt(tmp_path):
    """A fresh isolated Python process cannot borrow the test fixture's imports."""
    import os
    from pathlib import Path
    import subprocess
    import sys

    module = Path(__file__).resolve().parents[1] / 'lanes.py'
    assert module.is_file(), 'dashboard.lanes is missing'
    code = (
        'import runpy, sys; '
        'mod = runpy.run_path(sys.argv[1]); '
        "assert not any(n.startswith(('rclpy', 'modules', 'scenarios')) for n in sys.modules); "
        "assert mod['lawnmower']({'x':[0,1],'y':[0,1]},1,'y',0.8) == [(0.0,0.5,0.8),(1.0,0.5,0.8)]"
    )
    subprocess.run([sys.executable, '-I', '-S', '-c', code, str(module)], check=True,
                   cwd=str(tmp_path), env={'PATH': os.environ.get('PATH', '')})
