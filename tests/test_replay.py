import json
from wake.recording.replay import replay_records
def test_deterministic_replay(tmp_path):
    source=tmp_path/"telemetry.jsonl";records=[{"sequence":1},{"sequence":2}];source.write_text("".join(json.dumps(v)+"\n" for v in records));assert list(replay_records(source))==list(replay_records(source))==records
