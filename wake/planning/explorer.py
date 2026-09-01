"""Conservative exploration state machine over known-safe corridors."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import numpy as np
from wake.planning.frontier import Frontier,score
from wake.types import PoseSample,SurfaceEstimate,SystemHealth

class ExplorationState(str,Enum):
    INIT="INIT";WAIT_FOR_POSE="WAIT_FOR_POSE";WAIT_FOR_CLOCK_SYNC="WAIT_FOR_CLOCK_SYNC";WAIT_FOR_CALIBRATION="WAIT_FOR_CALIBRATION";TAKEOFF_READY="TAKEOFF_READY";LOCAL_SCAN="LOCAL_SCAN";EXPLORE="EXPLORE";APPROACH_FRONTIER="APPROACH_FRONTIER";SURFACE_DETECTED="SURFACE_DETECTED";BOUNDARY_FOLLOW="BOUNDARY_FOLLOW";BACK_OFF="BACK_OFF";RELOCALIZE="RELOCALIZE";HOLD="HOLD";RETURN_HOME="RETURN_HOME";LAND="LAND";EMERGENCY_STOP="EMERGENCY_STOP"

@dataclass(frozen=True)
class PlannerObservation:
    pose:PoseSample|None;health:SystemHealth;surface:SurfaceEstimate|None;battery_return:bool=False;mission_complete:bool=False

class Explorer:
    def __init__(self)->None:self.state=ExplorationState.INIT;self.home_pose:PoseSample|None=None;self.home_timestamp:int|None=None;self.known_safe_trajectory:list[PoseSample]=[];self.target:Frontier|None=None
    def initialize(self,*,pose_healthy:bool,calibrated:bool,clock_healthy:bool=True)->ExplorationState:
        self.state=ExplorationState.WAIT_FOR_POSE if not pose_healthy else ExplorationState.WAIT_FOR_CLOCK_SYNC if not clock_healthy else ExplorationState.WAIT_FOR_CALIBRATION if not calibrated else ExplorationState.TAKEOFF_READY;return self.state
    def update(self,observation:PlannerObservation)->ExplorationState:
        health=observation.health
        if "EMERGENCY" in health.failure_modes:self.state=ExplorationState.EMERGENCY_STOP;return self.state
        if observation.pose is None or not health.tag_visible:self.state=ExplorationState.RELOCALIZE;return self.state
        if health.clock_sync_confidence<.5 or health.clock_model_age_ms==float("inf"):self.state=ExplorationState.WAIT_FOR_CLOCK_SYNC;return self.state
        if not health.model_calibrated or not health.model_in_operational_envelope:self.state=ExplorationState.WAIT_FOR_CALIBRATION;return self.state
        if observation.battery_return or observation.mission_complete:self.state=ExplorationState.RETURN_HOME;return self.state
        if self.home_pose is None:self.home_pose=observation.pose;self.home_timestamp=observation.pose.timestamp_ns;self.state=ExplorationState.LOCAL_SCAN
        self.known_safe_trajectory.append(observation.pose)
        if observation.surface and observation.surface.nearby_probability>=.8:
            self.state=ExplorationState.SURFACE_DETECTED if observation.surface.confidence<.7 else ExplorationState.BOUNDARY_FOLLOW
        elif self.state in {ExplorationState.LOCAL_SCAN,ExplorationState.TAKEOFF_READY}:self.state=ExplorationState.EXPLORE
        else:self.state=ExplorationState.APPROACH_FRONTIER if self.target else ExplorationState.EXPLORE
        return self.state
    def choose_frontier(self,frontiers:list[Frontier])->Frontier|None:
        reachable=[frontier for frontier in frontiers if frontier.reachable_from_safe]
        self.target=max(reachable,key=score) if reachable else None
        return self.target
    def boundary_velocity(self,surface:SurfaceEstimate,speed:float)->tuple[float,float,float]:
        normal=np.asarray(surface.normal_body);up=np.asarray([0.,0.,1.]);tangent=np.cross(up,normal);length=np.linalg.norm(tangent)
        return (0.,0.,0.) if length<1e-9 else tuple((tangent/length*speed).tolist())
    def safe_return(self)->list[PoseSample]:return list(reversed(self.known_safe_trajectory))
