from __future__ import annotations
from pathlib import Path
import json
from wake.mapping.voxel_map import SparseVoxelMap
from wake.mapping.planes import Plane
from dataclasses import asdict

def export_json(voxel_map:SparseVoxelMap,path:str|Path)->Path:
    path=Path(path);payload={"voxel_size_m":voxel_map.voxel_size_m,"voxels":[{"ijk":list(k),"occupancy_probability":v.occupancy_probability,"observation_count":v.observation_count,"confidence":v.confidence,"last_update_time_ns":v.last_update_time_ns} for k,v in sorted(voxel_map.voxels.items())]};path.write_text(json.dumps(payload,indent=2),encoding="utf-8");return path
def export_ply(voxel_map:SparseVoxelMap,path:str|Path,threshold:float=.7)->Path:
    path=Path(path);points=[]
    for k,v in sorted(voxel_map.voxels.items()):
        if v.occupancy_probability>=threshold:points.append(tuple((i+.5)*voxel_map.voxel_size_m for i in k))
    header="ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n"%len(points);path.write_text(header+"".join(f"{x} {y} {z}\n" for x,y,z in points),encoding="utf-8");return path
def export_planes(planes:list[Plane],path:str|Path)->Path:
    target=Path(path);target.write_text(json.dumps([asdict(plane) for plane in planes],indent=2),encoding="utf-8");return target
