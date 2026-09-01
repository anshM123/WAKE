"""Bounded-worker end-to-end WAKE acquisition and mapping runtime."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from queue import Empty, Full, Queue
from threading import Event, Lock, Thread
import logging
import time
from typing import Any
import numpy as np

from wake.config import load_yaml
from wake.estimation.features import instantaneous_features
from wake.estimation.free_air import BaselineFreeAirModel, LearnedFreeAirModel
from wake.estimation.residual import ResidualEstimator
from wake.estimation.surface_model import BaselineSurfaceModel, CalibratedSurfaceModel, SurfaceModel
from wake.mapping.evidence import MapEvidence
from wake.mapping.export import export_json
from wake.mapping.ray_fusion import fuse_surface_evidence
from wake.mapping.voxel_map import SparseVoxelMap
from wake.pose.apriltag_pose import AprilTagPoseProvider
from wake.pose.transforms import quaternion_to_matrix
from wake.recording.recorder import SessionRecorder
from wake.telemetry.clock_sync import ClockModel, ClockSynchronizer
from wake.telemetry.receiver import TelemetryReceiver
from wake.telemetry.synchronizer import SampleSynchronizer, SynchronizationError
from wake.types import SurfaceEstimate, SystemHealth
from wake.visualization.live import LatestOnlyQueue

LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class MapSnapshot:
    timestamp_ns: int
    voxels: tuple[tuple[tuple[int, int, int], float, float, int], ...]
    trajectory: tuple[tuple[float, float, float], ...]
    current_position: tuple[float, float, float] | None
    current_rotation: tuple[float, float, float, float] | None
    surface: SurfaceEstimate | None
    health: SystemHealth


def _put_latest(queue: Queue, value: Any) -> None:
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        queue.put_nowait(value)


class WakeRuntime:
    """Own workers; the mapper is the sole mutable-map writer."""

    def __init__(self, config_path: str = "config/wake.yaml", *, xiao_host: str = "192.168.1.2", enable_camera: bool = True) -> None:
        self.config_path = config_path
        self.config = load_yaml(config_path)
        self.camera_config = load_yaml("config/camera.yaml")
        network = self.config["network"]
        sync = self.config["synchronization"]
        self.stop_event = Event()
        self.telemetry_queue: Queue = Queue(maxsize=2048)
        self.sync_queue: Queue = Queue(maxsize=2048)
        self.map_queue: Queue = Queue(maxsize=1024)
        self.ui_queue = LatestOnlyQueue()
        self.health = SystemHealth()
        self.health_lock = Lock()
        self.telemetry_receiver = TelemetryReceiver(network["bind"], int(network["telemetry_port"]))
        self.clock_model = ClockModel(max_rtt_ms=float(sync["clock_max_rtt_ms"]), stale_after_ms=float(sync["max_clock_model_age_ms"]))
        self.clock_sync = ClockSynchronizer(xiao_host, int(network["clock_port"]), self.clock_model)
        self.synchronizer = SampleSynchronizer(float(sync["max_pose_gap_ms"]), float(sync["max_pose_age_ms"]), self.clock_model)
        self.pose_provider = AprilTagPoseProvider(self.camera_config) if enable_camera else None
        self.voxel_map = SparseVoxelMap(**{key: self.config["mapping"][key] for key in ("voxel_size_m", "log_odds_min", "log_odds_max")})
        self.trajectory: list[tuple[float, float, float]] = []
        self.recorder = SessionRecorder(self.config["recording"]["root"], int(self.config["recording"]["queue_size"]))
        self.free_air_model = self._load_free_air_model()
        self.surface_model: SurfaceModel = BaselineSurfaceModel()
        surface_path = self.config["models"].get("surface_path")
        if surface_path:
            self.surface_model = CalibratedSurfaceModel.load(surface_path)
        self.residual_estimator = ResidualEstimator(int(self.config["filtering"]["persistence_samples"]))
        self.feature_history: list[np.ndarray] = []
        self.threads: list[Thread] = []
        self._last_map_snapshot = 0.0

    def _load_free_air_model(self):
        path = self.config["models"].get("free_air_path")
        return LearnedFreeAirModel.load(path) if path else BaselineFreeAirModel()

    def start(self) -> None:
        self.recorder.start({"software_version": "0.3.0", "drone_id": self.config["drone_id"], "mode": self.config["mode"]}, [self.config_path, "config/camera.yaml", "config/safety.yaml", "config/calibration.yaml"])
        self.recorder.record("events", {"event": "SESSION_START", "monotonic_ns": time.monotonic_ns()})
        targets = [self._telemetry_worker, self._clock_worker, self._sync_worker, self._estimator_worker, self._mapper_worker]
        if self.pose_provider is not None:
            targets.append(self._pose_worker)
        self.threads = [Thread(target=target, daemon=True, name=target.__name__) for target in targets]
        for thread in self.threads:
            thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=1.0)
        self.telemetry_receiver.close()
        self.clock_sync.close()
        if self.pose_provider:
            self.pose_provider.close()
        self.recorder.record("events", {"event": "SESSION_END", "monotonic_ns": time.monotonic_ns()})
        export_json(self.voxel_map, self.recorder.path / "final_map.json")
        self.recorder.close()

    def _telemetry_worker(self) -> None:
        while not self.stop_event.is_set():
            sample = self.telemetry_receiver.receive_once()
            if sample is None:
                continue
            self.recorder.record("telemetry", sample)
            _put_latest(self.telemetry_queue, sample)

    def _clock_worker(self) -> None:
        was_healthy = False
        while not self.stop_event.is_set():
            self.clock_sync.exchange_once()
            status = self.clock_model.status()
            healthy = self.clock_model.healthy()
            self.recorder.record("clock", asdict(status))
            if healthy != was_healthy:
                self.recorder.record("events", {"event": "CLOCK_SYNC_RECOVERED" if healthy else "CLOCK_SYNC_BAD", "monotonic_ns": time.monotonic_ns()})
            was_healthy = healthy
            with self.health_lock:
                self.health.clock_sync_rtt_ms = status.rtt_ms
                self.health.clock_model_age_ms = status.model_age_ms
                self.health.clock_sync_confidence = status.confidence
            self.stop_event.wait(.75)

    def _pose_worker(self) -> None:
        had_pose = False
        while not self.stop_event.is_set():
            pose = self.pose_provider.capture_once()
            raw = self.pose_provider.raw_pose
            if raw:
                self.recorder.record("raw_pose", raw)
            if pose:
                self.recorder.record("filtered_pose", pose)
                self.synchronizer.add_pose(pose)
            visible = pose is not None
            if visible != had_pose:
                self.recorder.record("events", {"event": "TAG_RECOVERED" if visible else "TAG_LOST", "monotonic_ns": time.monotonic_ns()})
            had_pose = visible
            with self.health_lock:
                self.health.tag_visible = visible
                self.health.reprojection_error_px = self.pose_provider.metrics.reprojection_error_px

    def _sync_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                telemetry = self.telemetry_queue.get(timeout=.25)
            except Empty:
                continue
            try:
                synchronized = self.synchronizer.synchronize(telemetry)
            except SynchronizationError as exc:
                with self.health_lock:
                    self.health.failure_modes = sorted(set(self.health.failure_modes + [str(exc)]))
                continue
            self.recorder.record("synchronized_samples", synchronized)
            self.sync_queue.put(synchronized)
            with self.health_lock:
                self.health.sync_error_ms = synchronized.synchronization_error_ms
                self.health.synchronization_gap_ms = synchronized.interpolation_gap_ms
                self.health.pose_age_ms = synchronized.pose_age_ms
                self.health.telemetry_age_ms = synchronized.telemetry_latency_ms
                self.health.battery_v = telemetry.battery_v

    def _estimator_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                sample = self.sync_queue.get(timeout=.25)
            except Empty:
                continue
            started = time.perf_counter_ns()
            base_features = instantaneous_features(sample)
            self.feature_history = (self.feature_history + [base_features])[-10:]
            features = self._free_air_features(base_features)
            observed = np.asarray([*sample.telemetry.accel_body_g, *sample.telemetry.gyro_body])
            expected = self.free_air_model.predict(features)
            residual = self.residual_estimator.calculate(observed, expected, np.asarray(sample.telemetry.motors))
            surface = self.surface_model.estimate(residual)
            self.recorder.record("preprocessed_features", features)
            self.recorder.record("free_air_prediction", expected)
            self.recorder.record("residual", residual)
            if surface:
                self.recorder.record("surface_estimate", surface)
            self.map_queue.put((sample, surface))
            with self.health_lock:
                self.health.estimator_latency_ms = (time.perf_counter_ns() - started) / 1e6
                self.health.model_calibrated = self.free_air_model.calibrated and self.surface_model.calibrated
                self.health.model_in_operational_envelope = bool(getattr(self.free_air_model,"in_operational_envelope",lambda _:False)(features)) and bool(getattr(self.surface_model,"last_in_envelope",False))

    def _free_air_features(self,base:np.ndarray)->np.ndarray:
        expected_length = len(getattr(getattr(self.free_air_model,"artifact",None),"features",base))
        if expected_length == len(base):
            return base
        window=np.asarray(self.feature_history)
        expanded=np.concatenate([window[-1],window.mean(axis=0),window.std(axis=0),window[-1]-window[0]])
        if len(expanded)!=expected_length:
            raise RuntimeError(f"free-air feature mismatch: model expects {expected_length}, runtime produced {len(expanded)}")
        return expanded

    def _mapper_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                sample, surface = self.map_queue.get(timeout=.25)
            except Empty:
                continue
            started = time.perf_counter_ns()
            position = sample.pose.position_world_m
            self.trajectory.append(position)
            self._mark_safe_corridor(position)
            if surface and surface.calibrated:
                rotation = quaternion_to_matrix(sample.pose.rotation_world_from_body)
                direction = tuple((rotation @ np.asarray(surface.normal_body)).tolist())
                evidence = MapEvidence(position, direction, surface.distance_m, surface.distance_sigma_m, surface.angular_sigma_rad, surface.confidence)
                fuse_surface_evidence(self.voxel_map, evidence, allow_free_space=surface.confidence >= .8)
                self.recorder.record("map_updates", evidence)
            with self.health_lock:
                self.health.mapper_latency_ms = (time.perf_counter_ns() - started) / 1e6
                health_copy = SystemHealth(**asdict(self.health))
            self.ui_queue.put(self.snapshot(sample.pose, surface, health_copy))
            self._periodic_snapshot()

    def _mark_safe_corridor(self, position: tuple[float, float, float]) -> None:
        center = self.voxel_map.index(position)
        for x in range(center[0] - 1, center[0] + 2):
            for y in range(center[1] - 1, center[1] + 2):
                for z in range(center[2] - 1, center[2] + 2):
                    self.voxel_map.update((x, y, z), -.8, 1.0)

    def snapshot(self, pose, surface, health) -> MapSnapshot:
        voxels = tuple((index, voxel.occupancy_probability, voxel.confidence, voxel.observation_count) for index, voxel in self.voxel_map.voxels.items())
        return MapSnapshot(time.monotonic_ns(), voxels, tuple(self.trajectory), pose.position_world_m, pose.rotation_world_from_body, surface, health)

    def _periodic_snapshot(self) -> None:
        interval = float(self.config["mapping"]["snapshot_interval_s"])
        now = time.monotonic()
        if now - self._last_map_snapshot < interval:
            return
        directory = self.recorder.path / "map_snapshots"
        directory.mkdir(exist_ok=True)
        count = len(list(directory.glob("map_snapshot_*.json"))) + 1
        export_json(self.voxel_map, directory / f"map_snapshot_{count:04d}.json")
        self._last_map_snapshot = now
