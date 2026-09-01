from __future__ import annotations
import time, numpy as np
from wake.pose.transforms import slerp
from wake.telemetry.buffers import TimestampBuffer
from wake.types import PoseSample, SynchronizedSample, TelemetrySample

class SynchronizationError(ValueError): pass

class SampleSynchronizer:
    def __init__(self, max_pose_gap_ms: float = 50.0, max_pose_age_ms: float = 100.0) -> None:
        self.max_pose_gap_ms=max_pose_gap_ms; self.max_pose_age_ms=max_pose_age_ms; self.poses=TimestampBuffer(lambda p:p.timestamp_ns)
    def add_pose(self, pose: PoseSample) -> None: self.poses.add(pose)
    def synchronize(self, telemetry: TelemetrySample, telemetry_timestamp_ns: int | None = None) -> SynchronizedSample:
        target = telemetry.drone_timestamp_us*1000 if telemetry_timestamp_ns is None else telemetry_timestamp_ns
        pair=self.poses.bracket(target)
        if pair is None: raise SynchronizationError("pose does not bracket telemetry timestamp")
        before, after=pair; gap_ms=(after.timestamp_ns-before.timestamp_ns)/1e6
        if gap_ms > self.max_pose_gap_ms: raise SynchronizationError("pose interpolation gap exceeds limit")
        age_ms=min(target-before.timestamp_ns, after.timestamp_ns-target)/1e6
        if age_ms > self.max_pose_age_ms: raise SynchronizationError("pose is stale")
        f=(target-before.timestamp_ns)/(after.timestamp_ns-before.timestamp_ns); p=(1-f)*np.asarray(before.position_world_m)+f*np.asarray(after.position_world_m)
        pose=PoseSample(telemetry.drone_id,after.sequence,target,tuple(p.tolist()),slerp(before.rotation_world_from_body,after.rotation_world_from_body,f),min(before.tracking_confidence,after.tracking_confidence),max(before.reprojection_error,after.reprojection_error),before.tag_id)
        latency=(telemetry.host_receive_timestamp_ns-target)/1e6
        return SynchronizedSample(telemetry,pose,age_ms,gap_ms,age_ms,latency)
