from __future__ import annotations
import time, numpy as np
from wake.pose.transforms import slerp
from wake.telemetry.buffers import TimestampBuffer
from wake.types import PoseSample, SynchronizedSample, TelemetrySample
from wake.telemetry.clock_sync import ClockModel

class SynchronizationError(ValueError): pass

class SampleSynchronizer:
    def __init__(self, max_pose_gap_ms: float = 50.0, max_pose_age_ms: float = 100.0, clock_model: ClockModel | None = None) -> None:
        self.max_pose_gap_ms=max_pose_gap_ms; self.max_pose_age_ms=max_pose_age_ms; self.poses=TimestampBuffer(lambda p:p.timestamp_ns); self.clock_model=clock_model
    def add_pose(self, pose: PoseSample) -> None: self.poses.add(pose)
    def synchronize(self, telemetry: TelemetrySample, telemetry_timestamp_ns: int | None = None) -> SynchronizedSample:
        if telemetry_timestamp_ns is None:
            if self.clock_model is None:
                raise SynchronizationError("CLOCK_UNSYNCED")
            try:
                target = self.clock_model.to_host_ns(telemetry.imu_timestamp_us or telemetry.drone_timestamp_us)
            except RuntimeError as exc:
                raise SynchronizationError(str(exc)) from exc
        else:
            target = telemetry_timestamp_ns
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
