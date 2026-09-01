from dataclasses import dataclass
import numpy as np
from wake.pose.transforms import quaternion_to_matrix
from pathlib import Path
import yaml
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

def plane_from_points(first:Vec3,second:Vec3,third:Vec3)->WorldPlane:
    a,b,c=np.asarray(first,float),np.asarray(second,float),np.asarray(third,float);normal=np.cross(b-a,c-a);length=float(np.linalg.norm(normal))
    if length<1e-9:raise ValueError("three points must not be collinear")
    normal/=length;return WorldPlane(tuple(normal.tolist()),-float(normal@a))

def closest_surface_point(position:Vec3,plane:WorldPlane)->Vec3:
    point=np.asarray(position,float);normal=np.asarray(plane.normal_world);signed=float(normal@point+plane.offset_m);return tuple((point-signed*normal).tolist())

def save_reference_plane(plane:WorldPlane,path:str|Path,name:str)->Path:
    target=Path(path);target.write_text(yaml.safe_dump({"reference_plane":{"name":name,"normal_world":list(plane.normal_world),"offset_m":plane.offset_m}},sort_keys=False),encoding="utf-8");return target
