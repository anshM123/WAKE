"""Validated, frame-explicit data shared by WAKE subsystems."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, IntFlag
from typing import Any
import math
import numpy as np

Vec3 = tuple[float, float, float]
Vec4 = tuple[float, float, float, float]
Quaternion = tuple[float, float, float, float]  # w, x, y, z


class Frame(str, Enum):
    """Right-handed frames. WORLD: +X east, +Y north, +Z up.

    DRONE_BODY and FC_BODY default to +X forward, +Y left, +Z up. CAMERA
    follows its calibrated camera convention until transformed to WORLD.
    APRILTAG follows the detector convention and is never used directly.
    """
    WORLD = "WORLD"
    CAMERA = "CAMERA"
    TAG = "TAG"
    DRONE_BODY = "DRONE_BODY"
    FC_BODY = "FC_BODY"


class Validity(IntFlag):
    NONE = 0
    ACCEL = 1
    GYRO = 2
    ATTITUDE = 4
    MOTORS = 8
    BATTERY = 16
    ALL = ACCEL | GYRO | ATTITUDE | MOTORS | BATTERY


def _finite_tuple(value: Any, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise ValueError(f"{name} must contain exactly {length} values")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} values must be finite")
    return result


@dataclass(frozen=True)
class TelemetrySample:
    drone_id: str
    sequence: int
    drone_timestamp_us: int
    host_receive_timestamp_ns: int
    accel_body_g: Vec3
    gyro_body: Vec3
    attitude_rpy_rad: Vec3
    motors: Vec4
    battery_v: float
    validity: Validity = Validity.ALL
    packet_age_ms: float = 0.0
    protocol_version: int = 2
    imu_timestamp_us: int | None = None
    motors_timestamp_us: int | None = None
    attitude_timestamp_us: int | None = None
    battery_timestamp_us: int | None = None
    host_timestamp_ns: int | None = None
    source_health: dict[str, float] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.drone_id: raise ValueError("drone_id is required")
        if not 0 <= self.sequence <= 0xFFFFFFFF: raise ValueError("sequence must be uint32")
        if self.drone_timestamp_us < 0 or self.host_receive_timestamp_ns < 0: raise ValueError("timestamps must be nonnegative")
        object.__setattr__(self, "accel_body_g", _finite_tuple(self.accel_body_g, 3, "accel_body_g"))
        object.__setattr__(self, "gyro_body", _finite_tuple(self.gyro_body, 3, "gyro_body"))
        object.__setattr__(self, "attitude_rpy_rad", _finite_tuple(self.attitude_rpy_rad, 3, "attitude_rpy_rad"))
        object.__setattr__(self, "motors", _finite_tuple(self.motors, 4, "motors"))
        if not math.isfinite(self.battery_v) or self.battery_v < 0: raise ValueError("battery_v must be nonnegative and finite")
        for name in ("imu_timestamp_us", "motors_timestamp_us", "attitude_timestamp_us", "battery_timestamp_us"):
            value = getattr(self, name)
            if value is not None and value < 0:
                raise ValueError(f"{name} must be nonnegative")


@dataclass(frozen=True)
class PoseSample:
    drone_id: str
    sequence: int
    timestamp_ns: int
    position_world_m: Vec3
    rotation_world_from_body: Quaternion
    tracking_confidence: float
    reprojection_error: float
    tag_id: int

    def __post_init__(self) -> None:
        if not self.drone_id: raise ValueError("drone_id is required")
        object.__setattr__(self, "position_world_m", _finite_tuple(self.position_world_m, 3, "position_world_m"))
        q = np.asarray(_finite_tuple(self.rotation_world_from_body, 4, "rotation_world_from_body"))
        norm = float(np.linalg.norm(q))
        if norm < 1e-9: raise ValueError("quaternion cannot be zero")
        object.__setattr__(self, "rotation_world_from_body", tuple((q / norm).tolist()))
        if not 0 <= self.tracking_confidence <= 1: raise ValueError("tracking_confidence must be in [0, 1]")
        if self.reprojection_error < 0 or not math.isfinite(self.reprojection_error): raise ValueError("invalid reprojection_error")


@dataclass(frozen=True)
class SynchronizedSample:
    telemetry: TelemetrySample
    pose: PoseSample
    synchronization_error_ms: float
    interpolation_gap_ms: float
    pose_age_ms: float
    telemetry_latency_ms: float

    def __post_init__(self) -> None:
        if self.telemetry.drone_id != self.pose.drone_id: raise ValueError("drone IDs do not match")


@dataclass(frozen=True)
class SurfaceEstimate:
    nearby_probability: float
    distance_m: float
    normal_body: Vec3
    distance_sigma_m: float
    angular_sigma_rad: float
    confidence: float
    calibrated: bool = False

    def __post_init__(self) -> None:
        for name in ("nearby_probability", "confidence"):
            if not 0 <= getattr(self, name) <= 1: raise ValueError(f"{name} must be in [0, 1]")
        if self.distance_m <= 0 or self.distance_sigma_m < 0 or self.angular_sigma_rad < 0: raise ValueError("invalid distance or uncertainty")
        normal = np.asarray(_finite_tuple(self.normal_body, 3, "normal_body"))
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9: raise ValueError("normal_body cannot be zero")
        object.__setattr__(self, "normal_body", tuple((normal / norm).tolist()))


@dataclass
class SystemHealth:
    telemetry_hz: float = 0.0
    pose_hz: float = 0.0
    dropped_packets: int = 0
    reordered_packets: int = 0
    pose_age_ms: float = math.inf
    telemetry_age_ms: float = math.inf
    sync_error_ms: float = math.inf
    battery_v: float = 0.0
    model_calibrated: bool = False
    disk_queue_depth: int = 0
    details: dict[str, Any] = field(default_factory=dict)
    imu_hz: float = 0.0
    motor_hz: float = 0.0
    tag_visible: bool = False
    reprojection_error_px: float = math.inf
    clock_sync_rtt_ms: float = math.inf
    clock_model_age_ms: float = math.inf
    clock_sync_confidence: float = 0.0
    synchronization_gap_ms: float = math.inf
    estimator_latency_ms: float = math.inf
    mapper_latency_ms: float = math.inf
    ui_latency_ms: float = math.inf
    model_in_operational_envelope: bool = False
    failure_modes: list[str] = field(default_factory=list)
