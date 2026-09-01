from dataclasses import dataclass
import numpy as np
from wake.pose.transforms import quaternion_to_matrix
from wake.types import PoseSample,Vec3

@dataclass(frozen=True)
class WorldPlane:
    normal_world:Vec3
    offset_m:float # normal dot x + offset = 0
    def __post_init__(self):
        if abs(np.linalg.norm(self.normal_world)-1)>.001:raise ValueError("plane normal must be unit length")

def wall_label(pose:PoseSample,plane:WorldPlane)->tuple[float,Vec3]:
    normal=np.asarray(plane.normal_world);signed=float(normal@np.asarray(pose.position_world_m)+plane.offset_m);normal_toward=normal*(-1 if signed>0 else 1);normal_body=quaternion_to_matrix(pose.rotation_world_from_body).T@normal_toward
    return abs(signed),tuple(normal_body.tolist())
