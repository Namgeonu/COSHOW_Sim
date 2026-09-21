#!/usr/bin/env python3
"""Check the bundled real configuration with an isolated, absent roster file.

No IP/URI/template/robot field is changed. ROS graph and unfilled LIMO IPs may
fail; the generated stack must succeed and spare exclusions must remain WARN.
"""
from pathlib import Path
import subprocess
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from dashboard.config import load_config
from dashboard.fleet import generate_config


with tempfile.TemporaryDirectory(prefix='m8-bundled-') as directory:
    root = Path(directory)
    raw = yaml.safe_load((ROOT / 'dashboard/config/dashboard.yaml').read_text())
    raw['roster_file'] = str(root / 'absent-roster.yaml')
    config = root / 'dashboard.yaml'
    config.write_text(yaml.safe_dump(raw, sort_keys=False))
    field = ROOT / 'dashboard/config/field.yaml'
    cfg = load_config(config, field)
    generated = generate_config(cfg)
    robots = yaml.safe_load(generated.files['crazyflies.generated.yaml'])['robots']
    assert sorted(robots) == sorted(cfg.drones)
    assert len(cfg.roster) == 4
    assert len([w for w in generated.warnings if '스페어 생성에서 제외' in w]) == 6
    print('FIXTURE: only roster_file redirected to an absent temporary file; bundled hardware fields unchanged', flush=True)
    print('BOOTSTRAP', cfg.roster, 'GENERATED', sorted(generated.files), flush=True)
    command = [sys.executable, str(ROOT / 'dashboard/server.py'), '--config', str(config),
               '--field', str(field), '--check-config', '--check-timeout', '.2']
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(result.stdout, end='', flush=True)
    assert 'PASS\tgeneration' in result.stdout and 'FAIL\tgeneration' not in result.stdout
    assert '로스터 역할 미배정' not in result.stdout
    assert sum('WARN\tconfiguration' in line and '스페어 생성에서 제외' in line
               for line in result.stdout.splitlines()) == 6
    errors = [line for line in result.stdout.splitlines() if line.startswith('FAIL\tconfiguration')]
    assert len(errors) == 2 and all('LIMO IP 미설정' in line for line in errors), errors
    assert result.returncode == 1
    print('PASS B2/B3: roles4 generated; URI-less spares6 WARN only. CLI rc1 is expected for absent ROS graph and two unfilled role LIMO IPs.')
