"""The CLI must not report a green graph over an unusable generated stack."""
import sys

from dashboard import server, ros_io
from dashboard.config import load_config


def test_check_config_generation_failure_sets_exit_status(monkeypatch, capsys):
    cfg = load_config(mock=True)
    cfg.raw['crazyflies_template'] = '/nonexistent/coshow-template.yaml'
    monkeypatch.setattr(server, 'load_config', lambda *a, **kw: cfg)
    monkeypatch.setattr(ros_io, 'check_config', lambda *a: [])
    monkeypatch.setattr(sys, 'argv', ['server.py', '--check-config'])
    assert server.main() == 1
    assert 'FAIL\tgeneration' in capsys.readouterr().out


def test_check_config_reports_successful_generation(monkeypatch, capsys):
    cfg = load_config(mock=True)
    monkeypatch.setattr(server, 'load_config', lambda *a, **kw: cfg)
    monkeypatch.setattr(ros_io, 'check_config', lambda *a: [])
    monkeypatch.setattr(sys, 'argv', ['server.py', '--check-config'])
    assert server.main() == 0
    assert 'PASS\tgeneration' in capsys.readouterr().out


def test_check_config_reports_spare_exclusion_warning(monkeypatch, capsys):
    cfg = load_config(mock=True)
    spare = next(row for row in cfg.raw['fleet']['drones'] if row['id'] not in cfg.roster.values())
    spare['uri'] = None
    monkeypatch.setattr(server, 'load_config', lambda *a, **kw: cfg)
    monkeypatch.setattr(ros_io, 'check_config', lambda *a: [])
    monkeypatch.setattr(sys, 'argv', ['server.py', '--check-config'])
    assert server.main() == 0
    output = capsys.readouterr().out
    assert '스페어 생성에서 제외' in output and spare['id'] in output
