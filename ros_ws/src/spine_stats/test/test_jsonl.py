import json
import math

from spine_stats.jsonl import JsonlLog, json_safe


def read_lines(path):
    with open(path, encoding='utf-8') as f:
        return [json.loads(line) for line in f]


def test_records_carry_run_wall_and_mono(tmp_path):
    log = JsonlLog(str(tmp_path), run_id='run-1')
    log.write('snapshot', orin={'posesSent': 3})
    log.write('event', kind='RIO_RESTART', controller_timestamp=12.5)
    log.close()
    recs = read_lines(log.path)
    assert [r['type'] for r in recs] == ['snapshot', 'event']
    for r in recs:
        assert r['run'] == 'run-1'
        assert r['wall'].endswith('+00:00')
        assert isinstance(r['mono'], float)
    assert recs[0]['orin'] == {'posesSent': 3}
    assert recs[1]['controller_timestamp'] == 12.5


def test_non_finite_floats_become_null():
    assert json_safe({'a': math.nan, 'b': [math.inf, 1.5], 'c': 'x'}) == {
        'a': None, 'b': [None, 1.5], 'c': 'x'}


def test_rotation_keeps_backup_count(tmp_path):
    log = JsonlLog(str(tmp_path), max_bytes=2000, backup_count=3, run_id='r')
    for i in range(400):
        log.write('snapshot', i=i, pad='x' * 50)
    log.close()
    files = sorted(p.name for p in tmp_path.iterdir())
    assert files == ['spine-orin.jsonl', 'spine-orin.jsonl.1', 'spine-orin.jsonl.2',
                     'spine-orin.jsonl.3']
    # Every line in every file is a complete JSON record.
    for name in files:
        for rec in read_lines(tmp_path / name):
            assert rec['type'] == 'snapshot'
