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
    if safety_cfg.get("safe_corridor_padding_m") is None: reasons.append("safe-corridor padding is unconfigured")
    return reasons

def autonomy_blockers(wake_cfg:dict[str,Any],safety_cfg:dict[str,Any],camera_cfg:dict[str,Any],calibration_cfg:dict[str,Any],health:Any|None=None,model_metrics:dict[str,Any]|None=None)->list[str]:
    reasons=validate_autonomy(wake_cfg,safety_cfg)
    camera_file=Path(camera_cfg.get("camera",{}).get("calibration_file",""))
    if not camera_file.exists():reasons.append("camera calibration is missing")
    else:
        try:
            camera_calibration=load_yaml(camera_file)
            if any(camera_calibration.get(key) is None for key in ("image_width","image_height","camera_matrix","distortion_coefficients","mean_reprojection_error_px")):reasons.append("camera calibration is incomplete")
        except (OSError,ValueError):reasons.append("camera calibration is invalid")
    if camera_cfg.get("transforms",{}).get("T_world_from_tag") is None:reasons.append("T_world_from_tag is uncalibrated")
    if camera_cfg.get("transforms",{}).get("T_body_from_camera") is None:reasons.append("T_body_from_camera is uncalibrated")
    if not camera_cfg.get("frame_check_confirmed",False):reasons.append("WORLD frame motion check is unconfirmed")
    if camera_cfg.get("camera",{}).get("capture_latency_ms") is None:reasons.append("camera acquisition latency is uncalibrated")
    if calibration_cfg.get("status")!="CALIBRATED":reasons.append("calibration status is not CALIBRATED")
    caution=safety_cfg.get("caution_distance_m");distance_recall=_distance_gated_recall(model_metrics,caution)
    if caution is None:reasons.append("caution distance is unconfigured for the recall gate")
    elif distance_recall is None or distance_recall<safety_cfg.get("detection_recall_min",1):reasons.append("held-out obstacle recall inside the caution zone is insufficient")
    if health is not None:
        if not health.tag_visible:reasons.append("tag is not currently healthy")
        if health.clock_sync_confidence<safety_cfg.get("minimum_clock_confidence",.5):reasons.append("clock confidence is below threshold")
        if health.clock_residual_ms>safety_cfg.get("maximum_clock_residual_ms",float("inf")):reasons.append("clock residual is above threshold")
        if not health.model_in_operational_envelope:reasons.append("current dynamics are outside the model envelope")
    return sorted(set(reasons))

def _distance_gated_recall(metrics:dict[str,Any]|None,caution_distance_m:float|None)->float|None:
    if metrics is None or caution_distance_m is None:return None
    bins=metrics.get("recall_by_distance",[])
    candidates=[item for item in bins if float(item["max_distance_m"])>=caution_distance_m]
    if not candidates:return None
    selected=min(candidates,key=lambda item:float(item["max_distance_m"]));return float(selected["recall"])
