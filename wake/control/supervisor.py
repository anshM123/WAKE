from dataclasses import dataclass
from enum import Enum
from wake.estimation.uncertainty import effective_distance
from wake.types import SurfaceEstimate,SystemHealth

class SafetyAction(str,Enum): ALLOW="ALLOW";CAUTION="CAUTION";HOLD="HOLD";RETURN_HOME="RETURN_HOME";LAND="LAND";EMERGENCY_STOP="EMERGENCY_STOP"
@dataclass(frozen=True)
class SafetyDecision: action:SafetyAction;reason:str;speed_limit_mps:float=0.0

class SafetySupervisor:
    def __init__(self,config:dict):self.config=config
    def evaluate(self,health:SystemHealth,surface:SurfaceEstimate|None=None)->SafetyDecision:
        if health.pose_age_ms>self.config["max_pose_age_ms"]:return SafetyDecision(SafetyAction.HOLD,"pose stale")
        if health.telemetry_age_ms>self.config["max_telemetry_age_ms"]:return SafetyDecision(SafetyAction.HOLD,"telemetry stale")
        if not health.model_calibrated:return SafetyDecision(SafetyAction.HOLD,"surface model UNCALIBRATED")
        minimum=self.config.get("minimum_battery_v");reserve=self.config.get("return_battery_v")
        if minimum is not None and health.battery_v<=minimum:return SafetyDecision(SafetyAction.LAND,"battery below landing threshold")
        if reserve is not None and health.battery_v<=reserve:return SafetyDecision(SafetyAction.RETURN_HOME,"battery reserve reached")
        if surface is not None and surface.nearby_probability>=self.config.get("confidence_min",.5):
            d=effective_distance(surface.distance_m,surface.distance_sigma_m,self.config.get("uncertainty_k",2.0)); emergency=self.config.get("emergency_distance_m");stop=self.config.get("stop_distance_m");caution=self.config.get("caution_distance_m")
            if emergency is not None and d<=emergency:return SafetyDecision(SafetyAction.EMERGENCY_STOP,"obstacle inside uncertainty-adjusted emergency distance")
            if stop is not None and d<=stop:return SafetyDecision(SafetyAction.HOLD,"obstacle inside uncertainty-adjusted stop distance")
            if caution is not None and d<=caution:return SafetyDecision(SafetyAction.CAUTION,"obstacle inside uncertainty-adjusted caution distance",self.config["max_unknown_speed_mps"])
        return SafetyDecision(SafetyAction.ALLOW,"health gates satisfied",self.config["max_known_speed_mps"])
