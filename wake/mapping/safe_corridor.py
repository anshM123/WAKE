"""Strong free-space evidence for the measured swept collision volume."""
from __future__ import annotations
import math
import numpy as np
from wake.mapping.voxel_map import SparseVoxelMap
from wake.types import Vec3

def mark_swept_corridor(voxel_map:SparseVoxelMap,start_world_m:Vec3,end_world_m:Vec3,*,drone_radius_m:float,safety_padding_m:float,timestamp_ns:int)->int:
    if drone_radius_m<=0:raise ValueError("drone_radius_m must be positive")
    if safety_padding_m<0:raise ValueError("safety_padding_m must be nonnegative")
    start,end=np.asarray(start_world_m,float),np.asarray(end_world_m,float);segment=end-start;length=float(np.linalg.norm(segment));step=voxel_map.voxel_size_m/2;sample_count=max(1,math.ceil(length/step));radius=drone_radius_m+safety_padding_m;reach=math.ceil(radius/voxel_map.voxel_size_m);updated=set()
    for fraction in np.linspace(0,1,sample_count+1):
        point=start+fraction*segment;center=voxel_map.index(tuple(point.tolist()))
        for x in range(center[0]-reach,center[0]+reach+1):
            for y in range(center[1]-reach,center[1]+reach+1):
                for z in range(center[2]-reach,center[2]+reach+1):
                    index=(x,y,z);voxel_center=np.asarray([(axis+.5)*voxel_map.voxel_size_m for axis in index])
                    if np.linalg.norm(voxel_center-point)<=radius:updated.add(index)
    for index in sorted(updated):voxel_map.update(index,-.8,1.,timestamp_ns)
    return len(updated)
