from __future__ import annotations
import math,numpy as np
from wake.mapping.evidence import MapEvidence
from wake.mapping.voxel_map import SparseVoxelMap

def fuse_surface_evidence(voxel_map:SparseVoxelMap,evidence:MapEvidence,allow_free_space:bool=False,timestamp_ns:int|None=None)->None:
    origin=np.asarray(evidence.origin_world_m);direction=np.asarray(evidence.direction_world,float);direction/=np.linalg.norm(direction);sigma=max(evidence.distance_sigma_m,voxel_map.voxel_size_m/2);step=voxel_map.voxel_size_m/2
    helper=np.asarray([0.,0.,1.]) if abs(direction[2])<.9 else np.asarray([1.,0.,0.]);first=np.cross(direction,helper);first/=np.linalg.norm(first);second=np.cross(direction,first);directions=[(direction,1.)]
    for scale,angular_weight in ((.5,.8),(1.,.5),(2.,.15)):
        for angle in np.linspace(0,2*math.pi,8,endpoint=False):
            tangent=math.cos(angle)*first+math.sin(angle)*second;directions.append((direction*math.cos(scale*evidence.angular_sigma_rad)+tangent*math.sin(scale*evidence.angular_sigma_rad),angular_weight))
    start=max(step,evidence.distance_m-3*sigma);end=evidence.distance_m+3*sigma;updates={}
    for ray,angular_weight in directions:
        for d in np.arange(start,end+step,step):
            weight=math.exp(-.5*((d-evidence.distance_m)/sigma)**2)*evidence.confidence*angular_weight;index=voxel_map.index(tuple((origin+ray*d).tolist()));updates[index]=max(updates.get(index,0),weight)
    for index,weight in updates.items():voxel_map.update(index,.7*weight,weight,timestamp_ns)
    if allow_free_space and evidence.confidence>=.8:
        for d in np.arange(0,max(0,evidence.distance_m-3*sigma),voxel_map.voxel_size_m):voxel_map.update(voxel_map.index(tuple((origin+direction*d).tolist())),-.15*evidence.confidence,evidence.confidence,timestamp_ns)
