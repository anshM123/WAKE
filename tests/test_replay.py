import json
from wake.recording.replay import ReplayPipeline,replay_records
def test_deterministic_replay(tmp_path):
    source=tmp_path/"telemetry.jsonl";records=[{"sequence":1},{"sequence":2}];source.write_text("".join(json.dumps(v)+"\n" for v in records));assert list(replay_records(source))==list(replay_records(source))==records

def test_same_session_produces_same_map_hash(tmp_path):
    session=tmp_path/"session";session.mkdir();telemetry={"drone_id":"wake-01","sequence":1,"drone_timestamp_us":1000,"host_receive_timestamp_ns":1_000_100,"accel_body_g":[0,0,1],"gyro_body":[0,0,0],"attitude_rpy_rad":[0,0,0],"motors":[1000]*4,"battery_v":4,"validity":31,"protocol_version":2};pose={"drone_id":"wake-01","sequence":1,"timestamp_ns":1_000_000,"position_world_m":[0,0,1],"rotation_world_from_body":[1,0,0,0],"tracking_confidence":1,"reprojection_error":0,"tag_id":0};row={"telemetry":telemetry,"pose":pose,"synchronization_error_ms":0,"interpolation_gap_ms":1,"pose_age_ms":0,"telemetry_latency_ms":0};(session/"synchronized_samples.jsonl").write_text(json.dumps(row)+"\n")
    config={"models":{"free_air_path":None,"surface_path":None},"mapping":{"voxel_size_m":.05,"log_odds_min":-4,"log_odds_max":4},"filtering":{"persistence_samples":3},"synchronization":{"clock_max_rtt_ms":30,"max_pose_gap_ms":50,"max_pose_age_ms":100}}
    first=ReplayPipeline(session,config).run(session/"first.json");second=ReplayPipeline(session,config).run(session/"second.json");assert first.map_hash==second.map_hash
