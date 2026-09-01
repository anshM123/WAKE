from enum import Enum
class ExplorationState(str,Enum):
    INIT="INIT";WAIT_FOR_POSE="WAIT_FOR_POSE";WAIT_FOR_CALIBRATION="WAIT_FOR_CALIBRATION";TAKEOFF_READY="TAKEOFF_READY";EXPLORE="EXPLORE";APPROACH_FRONTIER="APPROACH_FRONTIER";SURFACE_DETECTED="SURFACE_DETECTED";BACK_OFF="BACK_OFF";RELOCALIZE="RELOCALIZE";HOLD="HOLD";RETURN_HOME="RETURN_HOME";LAND="LAND";EMERGENCY_STOP="EMERGENCY_STOP"
class Explorer:
    def __init__(self)->None:self.state=ExplorationState.INIT;self.home_pose=None;self.known_safe_trajectory=[]
    def initialize(self,*,pose_healthy:bool,calibrated:bool)->ExplorationState:
        self.state=ExplorationState.WAIT_FOR_POSE if not pose_healthy else ExplorationState.WAIT_FOR_CALIBRATION if not calibrated else ExplorationState.TAKEOFF_READY;return self.state
