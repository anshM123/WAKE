from __future__ import annotations
import math,numpy as np
from wake.mapping.evidence import MapEvidence
from wake.mapping.voxel_map import SparseVoxelMap

def fuse_surface_evidence(voxel_map:SparseVoxelMap,evidence:MapEvidence,allow_free_space:bool=False)->None:
    origin=np.asarray(evidence.origin_world_m);direction=np.asarray(evidence.direction_world,float);direction/=np.linalg.norm(direction);sigma=max(evidence.distance_sigma_m,voxel_map.voxel_size_m/2);step=voxel_map.voxel_size_m/2
    start=max(step,evidence.distance_m-3*sigma);end=evidence.distance_m+3*sigma;seen=set()
    for d in np.arange(start,end+step,step):
        weight=math.exp(-.5*((d-evidence.distance_m)/sigma)**2)*evidence.confidence;index=voxel_map.index(tuple((origin+direction*d).tolist()))
        if index not in seen:voxel_map.update(index,.7*weight,weight);seen.add(index)
    if allow_free_space and evidence.confidence>=.8:
        for d in np.arange(0,max(0,evidence.distance_m-3*sigma),voxel_map.voxel_size_m):voxel_map.update(voxel_map.index(tuple((origin+direction*d).tolist())),-.15*evidence.confidence,evidence.confidence)
