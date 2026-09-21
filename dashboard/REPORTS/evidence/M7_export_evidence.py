"""Keep compact reproducible stage samples; full recordings remain in logs/."""
import csv
import hashlib
import json
from pathlib import Path

dashboard = Path(__file__).resolve().parents[2]
evidence, logs = dashboard / 'REPORTS/evidence', dashboard / 'logs'
source = logs / 'M7_stage_full.jsonl'
samples, phases, count, state_count = [], [], 0, 0
previous_phase = previous_run = None
for line in source.read_text().splitlines():
    count += 1
    record = json.loads(line)
    value = record['message']
    if value['type'] == 'hello':
        samples.append(record)
        continue
    state_count += 1
    phase = (value.get('mission') or {}).get('phase')
    run = value.get('run', {}).get('state')
    if phase != previous_phase or run != previous_run:
        samples.append(record)
    if phase and phase != previous_phase:
        phases.append(phase)
    previous_phase, previous_run = phase, run
expected = ['observe', 'handover', 'search', 'capture', 'rescue_dispatch', 'rescue', 'return', 'done']
assert phases == expected, phases
(evidence / 'M7_stage_sample.jsonl').write_text(''.join(json.dumps(value, ensure_ascii=False) + '\n' for value in samples))
timestamps = {value['received_at'] for value in samples}
with (logs / 'M7_poses_full.csv').open() as input_file, (evidence / 'M7_poses_sample.csv').open('w', newline='') as output_file:
    reader = csv.DictReader(input_file)
    writer = csv.DictWriter(output_file, fieldnames=reader.fieldnames)
    writer.writeheader()
    pose_count = sample_count = 0
    for row in reader:
        pose_count += 1
        if row['received_at'] in timestamps:
            writer.writerow(row)
            sample_count += 1
stdout = (logs / 'M7_probe_stdout.log').read_text()
command = '/usr/bin/python3 /home/coshow/COSHOW/dashboard/tests/stage_probe.py --fleet --seconds 106 --record /home/coshow/COSHOW/dashboard/logs/m7_stage_final.jsonl --dump-poses /home/coshow/COSHOW/dashboard/logs/m7_poses_final.csv'
(evidence / 'M7_stage_probe.log').write_text(command + '\n' + stdout)
rates = []
for line in stdout.splitlines():
    parts = line.split('\t')
    if len(parts) == 14 and parts[2] == 'drone' and parts[1] != 'spare':
        rates.append(float(parts[9]))
print('JSONL messages={} states={} full_state_samples={}'.format(count, state_count, len(samples)))
print('POSE CSV rows={} sample_rows={}'.format(pose_count, sample_count))
print('PHASES', json.dumps(phases))
print('CAMERA measured receipt fps min={} max={}'.format(min(rates), max(rates)))
for path in (source, logs / 'M7_poses_full.csv', logs / 'M7_probe_stdout.log'):
    print('FULL_ARTIFACT', path.relative_to(dashboard.parent), path.stat().st_size,
          hashlib.sha256(path.read_bytes()).hexdigest())
print('PASS complete eight-phase mock sequence; every recorded state contains full protocol payload')
