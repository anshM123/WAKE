"""Configuration loading with explicit placeholder validation."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import hashlib, json
import yaml

AUTONOMY_REQUIRED = (
    "geofence", "caution_distance_m", "stop_distance_m", "emergency_distance_m",
    "minimum_battery_v", "return_battery_v",
)

VALID_MODES = {
    "BENCH", "RECORD_ONLY", "CALIBRATION_FREE_AIR", "CALIBRATION_WALL",
    "REPLAY", "MAPPING_MANUAL_FLIGHT", "SIMULATION", "AUTONOMOUS",
}

def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as handle:
        value = yaml.safe_load(handle)
    if not isinstance(value, dict): raise ValueError(f"{path} must contain a YAML mapping")
    return value

def config_hash(*configs: dict[str, Any]) -> str:
    payload = json.dumps(configs, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()

def validate_autonomy(wake_cfg: dict[str, Any], safety_cfg: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    if wake_cfg.get("mode") != "AUTONOMOUS": reasons.append("mode is not AUTONOMOUS")
    if not wake_cfg.get("control", {}).get("matrix_high_level_interface_verified", False): reasons.append("Matrix high-level interface is unverified")
    if not wake_cfg.get("control", {}).get("manual_mapping_validated", False): reasons.append("manual mapping is not validated")
    if wake_cfg.get("models", {}).get("free_air_path") is None: reasons.append("free-air model is UNCALIBRATED")
    if wake_cfg.get("models", {}).get("surface_path") is None: reasons.append("surface model is UNCALIBRATED")
    reasons.extend(f"safety.{key} is a calibration placeholder" for key in AUTONOMY_REQUIRED if safety_cfg.get(key) is None)
    if safety_cfg.get("maximum_deceleration_mps2") is None: reasons.append("maximum deceleration is unmeasured")
    if safety_cfg.get("drone_radius_m") is None: reasons.append("drone radius is unconfigured")
    return reasons

def autonomy_blockers(wake_cfg:dict[str,Any],safety_cfg:dict[str,Any],camera_cfg:dict[str,Any],calibration_cfg:dict[str,Any],health:Any|None=None,model_metrics:dict[str,Any]|None=None)->list[str]:
    reasons=validate_autonomy(wake_cfg,safety_cfg)
    camera_file=Path(camera_cfg.get("camera",{}).get("calibration_file",""))
    if not camera_file.exists():reasons.append("camera calibration is missing")
    if camera_cfg.get("transforms",{}).get("T_world_from_tag") is None:reasons.append("T_world_from_tag is uncalibrated")
    if camera_cfg.get("transforms",{}).get("T_body_from_camera") is None:reasons.append("T_body_from_camera is uncalibrated")
    if not camera_cfg.get("frame_check_confirmed",False):reasons.append("WORLD frame motion check is unconfirmed")
    if calibration_cfg.get("status")!="CALIBRATED":reasons.append("calibration status is not CALIBRATED")
    if model_metrics is None or model_metrics.get("held_out_obstacle_recall",0)<safety_cfg.get("detection_recall_min",1):reasons.append("held-out obstacle recall is insufficient")
    if health is not None:
        if not health.tag_visible:reasons.append("tag is not currently healthy")
        if health.clock_sync_confidence<=0:reasons.append("clock is not synchronized")
        if not health.model_in_operational_envelope:reasons.append("current dynamics are outside the model envelope")
    return sorted(set(reasons))
