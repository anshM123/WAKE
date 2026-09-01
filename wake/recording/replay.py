"""Deterministic full-pipeline session replay."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator
import hashlib
import json
import time
import numpy as np

from wake.estimation.features import instantaneous_features
from wake.estimation.motion import PoseMotionEstimator
from wake.estimation.free_air import BaselineFreeAirModel, LearnedFreeAirModel
from wake.estimation.residual import ResidualEstimator
from wake.estimation.surface_model import BaselineSurfaceModel, CalibratedSurfaceModel
from wake.mapping.evidence import MapEvidence
from wake.mapping.export import export_json
from wake.mapping.ray_fusion import fuse_surface_evidence
from wake.mapping.voxel_map import SparseVoxelMap
from wake.pose.transforms import quaternion_to_matrix
from wake.protocol.messages import pose_from_mapping, telemetry_from_mapping
from wake.telemetry.clock_sync import ClockExchange, ClockModel
from wake.telemetry.synchronizer import SampleSynchronizer, SynchronizationError
from wake.types import PoseSample, SynchronizedSample, TelemetrySample


def read_jsonl(path: str | Path) -> Iterator[dict]:
    with Path(path).open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid JSON at {path}:{line_number}") from exc


def replay_records(path: str | Path, speed: float = 0) -> Iterator[dict]:
    previous = None
    for record in read_jsonl(path):
        timestamp = record.get("timestamp_ns") or record.get("host_receive_timestamp_ns")
        if speed > 0 and previous is not None and timestamp is not None:
            time.sleep(max(0, (timestamp - previous) / 1e9 / speed))
        if timestamp is not None:
            previous = timestamp
        yield record


def _telemetry(raw: dict) -> TelemetrySample:
    return telemetry_from_mapping({
        "protocol_version": raw.get("protocol_version", 2), "type": "telemetry",
        "drone_id": raw["drone_id"], "sequence": raw["sequence"],
        "timestamp_us": raw["drone_timestamp_us"], "accel_body_g": raw["accel_body_g"],
        "gyro_body": raw["gyro_body"], "attitude_rpy_rad": raw["attitude_rpy_rad"],
        "motors": raw["motors"], "battery_v": raw["battery_v"],
        "validity": raw["validity"], "imu_timestamp_us": raw.get("imu_timestamp_us"),
        "motors_timestamp_us": raw.get("motors_timestamp_us"),
        "attitude_timestamp_us": raw.get("attitude_timestamp_us"),
        "battery_timestamp_us": raw.get("battery_timestamp_us"),
    }, raw["host_receive_timestamp_ns"])


def _pose(raw: dict) -> PoseSample:
    return pose_from_mapping({"type": "pose", **raw})


def _synchronized(raw: dict) -> SynchronizedSample:
    return SynchronizedSample(_telemetry(raw["telemetry"]), _pose(raw["pose"]), raw["synchronization_error_ms"], raw["interpolation_gap_ms"], raw["pose_age_ms"], raw["telemetry_latency_ms"])


@dataclass(frozen=True)
class ReplayResult:
    synchronized_count: int
    surface_count: int
    voxel_count: int
    map_hash: str
    output_path: Path
    failures: tuple[str, ...]


class ReplayPipeline:
    def __init__(self, session: str | Path, config: dict, *, free_air_model=None, surface_model=None) -> None:
        self.session = Path(session); self.config = config
        self.free_air_model = free_air_model or (LearnedFreeAirModel.load(config["models"]["free_air_path"]) if config["models"].get("free_air_path") else BaselineFreeAirModel())
        self.surface_model = surface_model or (CalibratedSurfaceModel.load(config["models"]["surface_path"]) if config["models"].get("surface_path") else BaselineSurfaceModel())
        mapping = config["mapping"]
        self.map = SparseVoxelMap(mapping["voxel_size_m"], mapping["log_odds_min"], mapping["log_odds_max"])
        self.residual = ResidualEstimator(config["filtering"]["persistence_samples"])
        self.feature_history: list[np.ndarray] = []
        self.motion=PoseMotionEstimator()

    def synchronized_samples(self) -> Iterator[SynchronizedSample]:
        exchanges = self.session / "clock_exchange.jsonl"; poses = self.session / "filtered_pose.jsonl"; telemetry = self.session / "telemetry.jsonl"
        if exchanges.exists() and poses.exists() and telemetry.exists() and exchanges.stat().st_size:
            sync_cfg = self.config["synchronization"]
            clock = ClockModel(minimum_samples=1, max_rtt_ms=sync_cfg["clock_max_rtt_ms"], stale_after_ms=float("inf"))
            for raw in read_jsonl(exchanges): clock.add(ClockExchange(**raw))
            synchronizer = SampleSynchronizer(sync_cfg["max_pose_gap_ms"], sync_cfg["max_pose_age_ms"], clock)
            for raw in read_jsonl(poses): synchronizer.add_pose(_pose(raw))
            for raw in read_jsonl(telemetry):
                try: yield synchronizer.synchronize(_telemetry(raw))
                except SynchronizationError: continue
            return
        recorded = self.session / "synchronized_samples.jsonl"
        if not recorded.exists(): raise ValueError("session contains neither raw clock-domain streams nor synchronized samples")
        for raw in read_jsonl(recorded): yield _synchronized(raw)

    def run(self, output: str | Path | None = None, speed: float = 0) -> ReplayResult:
        synchronized_count = surface_count = 0; failures = [] ; previous_ns = None
        for sample in self.synchronized_samples():
            if speed > 0 and previous_ns is not None: time.sleep(max(0, (sample.pose.timestamp_ns - previous_ns) / 1e9 / speed))
            previous_ns = sample.pose.timestamp_ns; synchronized_count += 1
            base = instantaneous_features(sample,self.motion.update(sample.pose)); self.feature_history = (self.feature_history + [base])[-10:]; features = self._features(base)
            observed = np.asarray([*sample.telemetry.accel_body_g, *sample.telemetry.gyro_body]); expected = self.free_air_model.predict(features); residual = self.residual.calculate(observed, expected, np.asarray(sample.telemetry.motors)); surface = self.surface_model.estimate(residual)
            self._safe_corridor(sample.pose.position_world_m)
            if surface and surface.calibrated:
                direction = quaternion_to_matrix(sample.pose.rotation_world_from_body) @ np.asarray(surface.normal_body)
                fuse_surface_evidence(self.map, MapEvidence(sample.pose.position_world_m, tuple(direction.tolist()), surface.distance_m, surface.distance_sigma_m, surface.angular_sigma_rad, surface.confidence), surface.confidence >= .8,sample.pose.timestamp_ns); surface_count += 1
        if not self.free_air_model.calibrated: failures.append("MODEL_UNCALIBRATED: free-air")
        if not self.surface_model.calibrated: failures.append("MODEL_UNCALIBRATED: surface")
        output_path = Path(output) if output else self.session / "replay_map.json"; export_json(self.map, output_path); canonical=json.dumps(json.loads(output_path.read_text()),sort_keys=True,separators=(",",":"));digest=hashlib.sha256(canonical.encode()).hexdigest()
        return ReplayResult(synchronized_count,surface_count,len(self.map.voxels),digest,output_path,tuple(failures))

    def _features(self, base: np.ndarray) -> np.ndarray:
        expected_length = len(getattr(getattr(self.free_air_model, "artifact", None), "features", base))
        if expected_length == len(base): return base
        window=np.asarray(self.feature_history); expanded=np.concatenate([window[-1],window.mean(axis=0),window.std(axis=0),window[-1]-window[0]])
        if len(expanded)!=expected_length: raise ValueError("free-air model feature shape does not match replay pipeline")
        return expanded

    def _safe_corridor(self, position) -> None:
        center=self.map.index(position)
        for x in range(center[0]-1,center[0]+2):
            for y in range(center[1]-1,center[1]+2):
                for z in range(center[2]-1,center[2]+2): self.map.update((x,y,z),-.8,1.,timestamp_ns=1)
