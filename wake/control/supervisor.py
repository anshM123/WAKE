"""Independent fail-closed authority over planner motion proposals."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
from wake.types import SurfaceEstimate,SystemHealth

class SafetyAction(str,Enum):
    ALLOW="ALLOW";CAUTION="CAUTION";HOLD="HOLD";BACK_OFF="BACK_OFF";RETURN_HOME="RETURN_HOME";LAND="LAND";EMERGENCY_STOP="EMERGENCY_STOP"

@dataclass(frozen=True)
class SafetyDecision:action:SafetyAction;reason:str;speed_limit_mps:float=0.0;required_stop_distance_m:float=0.0

def dynamic_stop_distance(*,speed_mps:float,reaction_time_s:float,control_latency_s:float,maximum_deceleration_mps2:float,drone_radius_m:float,distance_sigma_m:float,uncertainty_k:float)->float:
    if maximum_deceleration_mps2<=0:raise ValueError("maximum deceleration must be positive")
    braking=speed_mps**2/(2*maximum_deceleration_mps2);latency=speed_mps*(reaction_time_s+control_latency_s);return drone_radius_m+latency+braking+uncertainty_k*distance_sigma_m

class SafetySupervisor:
    def __init__(self,config:dict):self.config=config
    def evaluate(self,health:SystemHealth,surface:SurfaceEstimate|None=None,*,current_speed_mps:float=0.,moving_into_unknown:bool=False,model_validation:dict|None=None)->SafetyDecision:
        if health.pose_age_ms>self.config["max_pose_age_ms"]:return SafetyDecision(SafetyAction.HOLD,"pose stale")
        if health.telemetry_age_ms>self.config["max_telemetry_age_ms"]:return SafetyDecision(SafetyAction.HOLD,"telemetry stale")
        if self.config.get("require_tag") and not health.tag_visible:return SafetyDecision(SafetyAction.HOLD,"TAG_LOST")
        if self.config.get("require_clock_sync") and (health.clock_model_age_ms>self.config["max_clock_age_ms"] or health.clock_sync_confidence<=0):return SafetyDecision(SafetyAction.HOLD,"CLOCK_UNSYNCED")
        if not health.model_calibrated:return SafetyDecision(SafetyAction.HOLD,"surface model UNCALIBRATED")
        if self.config.get("require_operational_envelope") and not health.model_in_operational_envelope:return SafetyDecision(SafetyAction.HOLD,"MODEL_OUT_OF_ENVELOPE")
        if model_validation is not None and model_validation.get("held_out_obstacle_recall",0)<self.config.get("detection_recall_min",1):return SafetyDecision(SafetyAction.HOLD,"detection-envelope recall below requirement")
        minimum=self.config.get("minimum_battery_v");reserve=self.config.get("return_battery_v")
        if minimum is not None and health.battery_v<=minimum:return SafetyDecision(SafetyAction.LAND,"battery below landing threshold")
        if reserve is not None and health.battery_v<=reserve:return SafetyDecision(SafetyAction.RETURN_HOME,"battery reserve reached")
        speed_limit=self.config["max_unknown_speed_mps"] if moving_into_unknown else self.config["max_known_speed_mps"]
        if surface is not None and surface.nearby_probability>=self.config.get("confidence_min",.5):
            deceleration=self.config.get("maximum_deceleration_mps2");radius=self.config.get("drone_radius_m")
            if deceleration is None or radius is None:return SafetyDecision(SafetyAction.HOLD,"dynamic stop parameters uncalibrated")
            required=dynamic_stop_distance(speed_mps=current_speed_mps,reaction_time_s=self.config.get("reaction_time_s",.25),control_latency_s=self.config.get("command_timeout_ms",250)/1000,maximum_deceleration_mps2=deceleration,drone_radius_m=radius,distance_sigma_m=surface.distance_sigma_m,uncertainty_k=self.config.get("uncertainty_k",2.))
            configured=self.config.get("stop_distance_m")
            if configured is not None and configured<required:return SafetyDecision(SafetyAction.HOLD,"configured stop distance is below dynamic minimum",0,required)
            emergency=self.config.get("emergency_distance_m")
            if emergency is not None and surface.distance_m<=emergency:return SafetyDecision(SafetyAction.EMERGENCY_STOP,"obstacle inside emergency distance",0,required)
            if surface.distance_m<=max(required,configured or 0):return SafetyDecision(SafetyAction.HOLD,"obstacle inside dynamic stop distance",0,required)
            caution=self.config.get("caution_distance_m")
            if caution is not None and surface.distance_m<=caution:return SafetyDecision(SafetyAction.CAUTION,"obstacle inside caution distance",self.config["max_unknown_speed_mps"],required)
        return SafetyDecision(SafetyAction.ALLOW,"health gates satisfied",speed_limit)
