"""Archive the unchanged M7 full mock evidence with reproducible gzip bytes."""
import gzip
import hashlib
from pathlib import Path

dashboard = Path(__file__).resolve().parents[2]
rows = [
    ('m7_stage_final.jsonl', 'M7_stage_full.jsonl',
     'f35d640d0e845f9a8da2dc1523c3f7e99161293f86f50ed3fd0e2dc2218d1091'),
    ('m7_poses_final.csv', 'M7_poses_full.csv',
     '5c1a6428571ebf828538e63f8518351dfa7d77ba7e9c9255e167951117c4b364'),
]
for original_name, retained_name, expected in rows:
    source = dashboard / 'logs' / retained_name
    data = source.read_bytes()
    assert hashlib.sha256(data).hexdigest() == expected, source
    archive = dashboard / 'REPORTS/evidence' / (retained_name + '.gz')
    with archive.open('wb') as output:
        with gzip.GzipFile(filename='', mode='wb', fileobj=output, mtime=0) as compressed:
            compressed.write(data)
    assert gzip.decompress(archive.read_bytes()) == data
    print('ORIGINAL_CONTAINER /home/coshow/COSHOW/dashboard/logs/' + original_name)
    print('LOCAL_GITIGNORED dashboard/logs/' + retained_name)
    print('TRACKED_ARCHIVE dashboard/REPORTS/evidence/' + archive.name)
    print('UNCOMPRESSED_BYTES', len(data), 'SHA256', expected)
    print('COMPRESSED_BYTES', archive.stat().st_size, 'SHA256', hashlib.sha256(archive.read_bytes()).hexdigest())
print('M7_stage_analysis.log used ORIGINAL_CONTAINER m7_stage_final.jsonl; its bytes match the archived M7_stage_full.jsonl.')
print('M7_stage_sample.jsonl and M7_poses_sample.csv are transition subsets, not full copies.')
