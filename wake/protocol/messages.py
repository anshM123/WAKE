from __future__ import annotations
from typing import Any
import json, time
from wake.types import PoseSample, TelemetrySample, Validity

PROTOCOL_VERSION = 2

def telemetry_from_mapping(message: dict[str, Any], received_ns: int | None = None) -> TelemetrySample:
    required = {"protocol_version", "type", "drone_id", "sequence", "timestamp_us", "accel_body_g", "gyro_body", "attitude_rpy_rad", "motors", "battery_v", "validity"}
    missing = required - message.keys()
    if missing: raise ValueError(f"missing telemetry fields: {sorted(missing)}")
    if message["type"] != "telemetry": raise ValueError("not a telemetry packet")
    if int(message["protocol_version"]) != PROTOCOL_VERSION: raise ValueError("unsupported protocol version")
    received_ns = time.monotonic_ns() if received_ns is None else received_ns
    return TelemetrySample(
        drone_id=str(message["drone_id"]), sequence=int(message["sequence"]),
        drone_timestamp_us=int(message["timestamp_us"]), host_receive_timestamp_ns=received_ns,
        accel_body_g=message["accel_body_g"], gyro_body=message["gyro_body"],
        attitude_rpy_rad=message["attitude_rpy_rad"], motors=message["motors"],
        battery_v=float(message["battery_v"]), validity=Validity(int(message["validity"])),
        packet_age_ms=float(message.get("packet_age_ms", 0.0)),
        protocol_version=PROTOCOL_VERSION,
        imu_timestamp_us=_optional_int(message, "imu_timestamp_us"),
        motors_timestamp_us=_optional_int(message, "motors_timestamp_us"),
        attitude_timestamp_us=_optional_int(message, "attitude_timestamp_us"),
        battery_timestamp_us=_optional_int(message, "battery_timestamp_us"),
        source_health={str(key): float(value) for key, value in message.get("health", {}).items()},
    )


def _optional_int(message: dict[str, Any], key: str) -> int | None:
    value = message.get(key)
    return None if value is None else int(value)

def decode_telemetry(payload: bytes, received_ns: int | None = None) -> TelemetrySample:
    if len(payload) > 4096: raise ValueError("telemetry packet exceeds 4096 bytes")
    try: message = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc: raise ValueError("malformed telemetry JSON") from exc
    if not isinstance(message, dict): raise ValueError("telemetry packet must be an object")
    return telemetry_from_mapping(message, received_ns)

def pose_from_mapping(message: dict[str, Any]) -> PoseSample:
    required = {"type", "drone_id", "sequence", "timestamp_ns", "position_world_m", "rotation_world_from_body", "tracking_confidence", "reprojection_error", "tag_id"}
    missing = required - message.keys()
    if missing: raise ValueError(f"missing pose fields: {sorted(missing)}")
    if message["type"] != "pose": raise ValueError("not a pose packet")
    return PoseSample(str(message["drone_id"]), int(message["sequence"]), int(message["timestamp_ns"]), message["position_world_m"], message["rotation_world_from_body"], float(message["tracking_confidence"]), float(message["reprojection_error"]), int(message["tag_id"]))
