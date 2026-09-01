from dataclasses import dataclass
from pathlib import Path
import json
import numpy as np
from wake.protocol.messages import telemetry_from_mapping,pose_from_mapping
from wake.types import SynchronizedSample
from wake.estimation.features import instantaneous_features
@dataclass(frozen=True)
class DatasetSplit:train_session_ids:list[str];validation_session_ids:list[str];test_session_ids:list[str]
def validate_session_split(split:DatasetSplit)->None:
    groups=[set(split.train_session_ids),set(split.validation_session_ids),set(split.test_session_ids)]
    if groups[0]&groups[1] or groups[0]&groups[2] or groups[1]&groups[2]:raise ValueError("recording sessions must not leak across splits")

def temporal_windows(features:np.ndarray,targets:np.ndarray,window_size:int)->tuple[np.ndarray,np.ndarray]:
    if window_size<2:raise ValueError("window_size must be at least 2")
    output,labels=[],[]
    for index in range(window_size-1,len(features)):
        window=features[index-window_size+1:index+1];output.append(np.concatenate([window[-1],window.mean(axis=0),window.std(axis=0),window[-1]-window[0]]));labels.append(targets[index])
    return np.asarray(output),np.asarray(labels)

def load_synchronized_session(session:str|Path,window_size:int=10)->tuple[np.ndarray,np.ndarray]:
    rows=[]
    with (Path(session)/"synchronized_samples.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            raw=json.loads(line);t=raw["telemetry"];p=raw["pose"];telemetry=telemetry_from_mapping({"protocol_version":2,"type":"telemetry","drone_id":t["drone_id"],"sequence":t["sequence"],"timestamp_us":t["drone_timestamp_us"],"accel_body_g":t["accel_body_g"],"gyro_body":t["gyro_body"],"attitude_rpy_rad":t["attitude_rpy_rad"],"motors":t["motors"],"battery_v":t["battery_v"],"validity":t["validity"]},t["host_receive_timestamp_ns"]);pose=pose_from_mapping({"type":"pose","drone_id":p["drone_id"],"sequence":p["sequence"],"timestamp_ns":p["timestamp_ns"],"position_world_m":p["position_world_m"],"rotation_world_from_body":p["rotation_world_from_body"],"tracking_confidence":p["tracking_confidence"],"reprojection_error":p["reprojection_error"],"tag_id":p["tag_id"]});rows.append(SynchronizedSample(telemetry,pose,raw["synchronization_error_ms"],raw["interpolation_gap_ms"],raw["pose_age_ms"],raw["telemetry_latency_ms"]))
    base=np.asarray([instantaneous_features(row) for row in rows]);targets=np.asarray([[*row.telemetry.accel_body_g,*row.telemetry.gyro_body] for row in rows]);return temporal_windows(base,targets,window_size)

def split_sessions(session_paths:list[str|Path],validation_fraction:float=.2,test_fraction:float=.2)->DatasetSplit:
    identifiers=sorted(Path(path).name for path in session_paths);count=len(identifiers)
    if count<3:raise ValueError("at least three independent sessions are required")
    test=max(1,round(count*test_fraction));validation=max(1,round(count*validation_fraction));return DatasetSplit(identifiers[:count-validation-test],identifiers[count-validation-test:count-test],identifiers[count-test:])
