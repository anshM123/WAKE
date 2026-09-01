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
from wake import __version__

from wake.config import autonomy_blockers,config_hash,load_yaml
from wake.estimation.features import instantaneous_features
from wake.estimation.motion import PoseMotionEstimator
from wake.estimation.free_air import BaselineFreeAirModel, LearnedFreeAirModel
from wake.estimation.residual import ResidualEstimator
from wake.estimation.surface_model import BaselineSurfaceModel, CalibratedSurfaceModel, SurfaceModel
from wake.mapping.evidence import MapEvidence
from wake.mapping.export import export_json
from wake.mapping.ray_fusion import fuse_surface_evidence
from wake.mapping.voxel_map import SparseVoxelMap
from wake.mapping.safe_corridor import mark_swept_corridor
from wake.mapping.planes import Plane, extract_planes
from wake.mapping.mesh import export_obj, reconstruct_planar_mesh
from wake.mapping.export import export_planes, export_ply
from wake.pose.apriltag_pose import AprilTagPoseProvider
from wake.pose.transforms import quaternion_to_matrix
from wake.recording.recorder import SessionRecorder
from wake.telemetry.clock_sync import ClockModel, ClockSynchronizer
from wake.telemetry.receiver import TelemetryReceiver
from wake.telemetry.synchronizer import SampleSynchronizer, SynchronizationError
from wake.types import SurfaceEstimate, SystemHealth
from wake.visualization.live import LatestOnlyQueue
from wake.control.supervisor import SafetySupervisor

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
    planes: tuple[Plane, ...] = ()


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
        if self.config.get("mode")=="AUTONOMOUS":
            calibration=load_yaml("config/calibration.yaml");safety=load_yaml("config/safety.yaml");metrics=None;surface_path=self.config.get("models",{}).get("surface_path")
            if surface_path and Path(surface_path).exists():metrics=__import__("json").loads(Path(surface_path).read_text(encoding="utf-8")).get("validation_metrics")
            blockers=autonomy_blockers(self.config,safety,self.camera_config,calibration,model_metrics=metrics)
            if blockers:raise RuntimeError("AUTONOMOUS BLOCKED:\n- "+"\n- ".join(blockers))
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
        self.planes: list[Plane] = []
        self.recorder = SessionRecorder(self.config["recording"]["root"], int(self.config["recording"]["queue_size"]))
        self.free_air_model = self._load_free_air_model()
        self.surface_model: SurfaceModel = BaselineSurfaceModel()
        surface_path = self.config["models"].get("surface_path")
        if surface_path:
            self.surface_model = CalibratedSurfaceModel.load(surface_path)
        self.residual_estimator = ResidualEstimator(int(self.config["filtering"]["persistence_samples"]))
        self.motion_estimator=PoseMotionEstimator()
        self.safety_config=load_yaml("config/safety.yaml");self.safety_supervisor=SafetySupervisor(self.safety_config)
        self.feature_history: list[np.ndarray] = []
        self.threads: list[Thread] = []
        self._last_map_snapshot = 0.0
        self._last_safety_action: str | None = None
        self._previous_corridor_position: tuple[float,float,float] | None = None
        self._telemetry_rate_start=time.monotonic();self._telemetry_count=0;self._pose_rate_start=time.monotonic();self._pose_count=0

    def _load_free_air_model(self):
        path = self.config["models"].get("free_air_path")
        return LearnedFreeAirModel.load(path) if path else BaselineFreeAirModel()

    def start(self) -> None:
        snapshots=[self.config_path,"config/camera.yaml","config/safety.yaml","config/calibration.yaml"]
        if Path("config/reference_plane.yaml").exists():snapshots.append("config/reference_plane.yaml")
        camera_calibration=load_yaml(self.camera_config["camera"]["calibration_file"]) if Path(self.camera_config["camera"]["calibration_file"]).exists() else {}
        metadata={"software_version":__version__,"firmware_version":"0.3.0","drone_id":self.config["drone_id"],"mode":self.config["mode"],"tag_id":self.camera_config["tag"]["id"],"tag_size_m":self.camera_config["tag"]["size_m"],"camera_calibration_hash":config_hash(camera_calibration),"config_hash":config_hash(self.config,self.camera_config),"free_air_model_version":getattr(getattr(self.free_air_model,"artifact",None),"model_version","UNCALIBRATED"),"surface_model_version":getattr(getattr(self.surface_model,"artifact",None),"model_version","UNCALIBRATED")}
        self.recorder.start(metadata, snapshots)
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
        export_ply(self.voxel_map,self.recorder.path/"final_map.ply",float(self.config["mapping"]["occupied_threshold"]))
        export_planes(self.planes,self.recorder.path/"planes.json")
        export_obj(reconstruct_planar_mesh(self.planes,float(self.config["mapping"]["plane_min_confidence"])),self.recorder.path/"supported_surfaces.obj")
        self.recorder.close()

    def _telemetry_worker(self) -> None:
        while not self.stop_event.is_set():
            sample = self.telemetry_receiver.receive_once()
            if sample is None:
                continue
            self.recorder.record("telemetry", sample)
            self._telemetry_count+=1;elapsed=time.monotonic()-self._telemetry_rate_start
            if elapsed>=1:
                with self.health_lock:
                    self.health.telemetry_hz=self._telemetry_count/elapsed
                    self.health.imu_hz=sample.source_health.get("imu_hz",self.health.telemetry_hz)
                    self.health.motor_hz=sample.source_health.get("motor_hz",self.health.telemetry_hz)
                    self.health.dropped_packets=self.telemetry_receiver.dropped_packets
                    self.health.reordered_packets=self.telemetry_receiver.reordered
                    total=max(1,self._telemetry_count+self.telemetry_receiver.dropped_packets)
                    self.health.details["packet_loss_percent"]=100*self.telemetry_receiver.dropped_packets/total
                self._telemetry_count=0;self._telemetry_rate_start=time.monotonic()
            _put_latest(self.telemetry_queue, sample)

    def _clock_worker(self) -> None:
        was_healthy = False
        while not self.stop_event.is_set():
            self.clock_sync.exchange_once()
            status = self.clock_model.status()
            healthy = self.clock_model.healthy()
            self.recorder.record("clock", asdict(status))
            if self.clock_model.samples:
                self.recorder.record("clock_exchange", asdict(self.clock_model.samples[-1]))
            if healthy != was_healthy:
                self.recorder.record("events", {"event": "CLOCK_SYNC_RECOVERED" if healthy else "CLOCK_SYNC_BAD", "monotonic_ns": time.monotonic_ns()})
            was_healthy = healthy
            with self.health_lock:
                self.health.clock_sync_rtt_ms = status.rtt_ms
                self.health.clock_model_age_ms = status.model_age_ms
                self.health.clock_sync_confidence = status.confidence
                self.health.clock_residual_ms = status.residual_ms
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
                self._pose_count+=1;elapsed=time.monotonic()-self._pose_rate_start
                if elapsed>=1:
                    with self.health_lock:self.health.pose_hz=self._pose_count/elapsed
                    self._pose_count=0;self._pose_rate_start=time.monotonic()
            visible = pose is not None
            if visible != had_pose:
                self.recorder.record("events", {"event": "TAG_RECOVERED" if visible else "TAG_LOST", "monotonic_ns": time.monotonic_ns()})
            had_pose = visible
            with self.health_lock:
                self.health.tag_visible = visible
                self.health.reprojection_error_px = self.pose_provider.metrics.reprojection_error_px
                if not self.pose_provider.metrics.latency_calibrated:self.health.failure_modes=sorted(set(self.health.failure_modes+["CAMERA_LATENCY_UNCALIBRATED"]))

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
            motion = self.motion_estimator.update(sample.pose)
            base_features = instantaneous_features(sample,motion)
            self.feature_history = (self.feature_history + [base_features])[-10:]
            features = self._free_air_features(base_features)
            observed = np.asarray([*sample.telemetry.accel_body_g, *sample.telemetry.gyro_body])
            expected = self.free_air_model.predict(features)
            residual = self.residual_estimator.calculate(observed, expected, np.asarray(sample.telemetry.motors))
            surface = self.surface_model.estimate(residual)
            model_validation=getattr(getattr(self.surface_model,"artifact",None),"validation_metrics",None)
            decision=self.safety_supervisor.evaluate(self.health,surface,current_speed_mps=float(np.linalg.norm(motion.world_velocity_mps)),moving_into_unknown=True,model_validation=model_validation)
            with self.health_lock:self.health.details["safety_action"]=decision.action.value;self.health.details["safety_reason"]=decision.reason
            if decision.action.value!=self._last_safety_action and decision.action.value in {"CAUTION","HOLD","BACK_OFF","RETURN_HOME","LAND","EMERGENCY_STOP"}:self.recorder.record("events",{"event":f"SAFETY_{decision.action.value}","reason":decision.reason,"monotonic_ns":time.monotonic_ns()})
            self._last_safety_action=decision.action.value
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
            self._mark_safe_corridor(position,sample.pose.timestamp_ns)
            if surface and surface.calibrated:
                rotation = quaternion_to_matrix(sample.pose.rotation_world_from_body)
                direction = tuple((rotation @ np.asarray(surface.direction_body)).tolist())
                evidence = MapEvidence(position, direction, surface.distance_m, surface.distance_sigma_m, surface.angular_sigma_rad, surface.confidence)
                fuse_surface_evidence(self.voxel_map, evidence, allow_free_space=surface.confidence >= .8,timestamp_ns=sample.pose.timestamp_ns)
                self.recorder.record("map_updates", evidence)
            with self.health_lock:
                self.health.mapper_latency_ms = (time.perf_counter_ns() - started) / 1e6
                health_copy = SystemHealth(**asdict(self.health))
            self.ui_queue.put(self.snapshot(sample.pose, surface, health_copy))
            self._periodic_snapshot()

    def _mark_safe_corridor(self,position:tuple[float,float,float],timestamp_ns:int)->None:
        radius=self.safety_config.get("drone_radius_m");padding=self.safety_config.get("safe_corridor_padding_m")
        if radius is None or padding is None:
            with self.health_lock:self.health.failure_modes=sorted(set(self.health.failure_modes+["SAFE_CORRIDOR_UNCALIBRATED"]))
            self._previous_corridor_position=position;return
        start=self._previous_corridor_position or position;mark_swept_corridor(self.voxel_map,start,position,drone_radius_m=float(radius),safety_padding_m=float(padding),timestamp_ns=timestamp_ns);self._previous_corridor_position=position

    def snapshot(self, pose, surface, health) -> MapSnapshot:
        voxels = tuple((index, voxel.occupancy_probability, voxel.confidence, voxel.observation_count) for index, voxel in self.voxel_map.voxels.items())
        return MapSnapshot(time.monotonic_ns(), voxels, tuple(self.trajectory), pose.position_world_m, pose.rotation_world_from_body, surface, health, tuple(self.planes))

    def _periodic_snapshot(self) -> None:
        interval = float(self.config["mapping"]["snapshot_interval_s"])
        now = time.monotonic()
        if now - self._last_map_snapshot < interval:
            return
        directory = self.recorder.path / "map_snapshots"
        directory.mkdir(exist_ok=True)
        count = len(list(directory.glob("map_snapshot_*.json"))) + 1
        export_json(self.voxel_map, directory / f"map_snapshot_{count:04d}.json")
        occupied=[(tuple((axis+.5)*self.voxel_map.voxel_size_m for axis in index),voxel.confidence) for index,voxel in self.voxel_map.voxels.items() if voxel.occupancy_probability>=float(self.config["mapping"]["occupied_threshold"])]
        if len(occupied)>=30:
            self.planes=extract_planes(np.asarray([point for point,_ in occupied]),confidences=np.asarray([confidence for _,confidence in occupied]),minimum_support=30,distance_threshold_m=self.voxel_map.voxel_size_m*1.5)
            export_planes(self.planes,directory/f"planes_{count:04d}.json")
        self._last_map_snapshot = now
