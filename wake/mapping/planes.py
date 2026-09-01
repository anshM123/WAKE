"""Deterministic RANSAC plane extraction and evidence-bounded patches."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from wake.types import Vec3

@dataclass(frozen=True)
class Plane:
    normal_world:Vec3;offset_m:float;confidence:float;support_count:int;centroid_world:Vec3=(0.,0.,0.);corners_world:tuple[Vec3,Vec3,Vec3,Vec3]|None=None;classification:str="OTHER"
    def distance(self,points:np.ndarray)->np.ndarray:return np.abs(np.asarray(points)@np.asarray(self.normal_world)+self.offset_m)

def _fit_plane(points:np.ndarray)->tuple[np.ndarray,float]:
    centroid=points.mean(axis=0);_,_,vectors=np.linalg.svd(points-centroid,full_matrices=False);normal=vectors[-1]
    if normal[2]<0:normal=-normal
    return normal,-float(normal@centroid)

def _classify(normal:np.ndarray)->str:
    alignment=abs(float(normal[2]));return "FLOOR_OR_CEILING" if alignment>.85 else "WALL" if alignment<.25 else "OTHER"

def _patch(points:np.ndarray)->tuple[Vec3,Vec3,Vec3,Vec3]:
    centroid=points.mean(axis=0);_,_,vectors=np.linalg.svd(points-centroid,full_matrices=False);first,second=vectors[0],vectors[1];a=(points-centroid)@first;b=(points-centroid)@second;alo,ahi=np.percentile(a,[2,98]);blo,bhi=np.percentile(b,[2,98]);corners=(centroid+alo*first+blo*second,centroid+ahi*first+blo*second,centroid+ahi*first+bhi*second,centroid+alo*first+bhi*second);return tuple(tuple(c.tolist()) for c in corners)

def extract_planes(points:np.ndarray,*,confidences:np.ndarray|None=None,distance_threshold_m:float=.05,minimum_support:int=30,maximum_planes:int=8,iterations:int=500,seed:int=0)->list[Plane]:
    remaining=np.asarray(points,float)
    if remaining.ndim!=2 or remaining.shape[1]!=3:raise ValueError("points must be shaped (N, 3)")
    remaining_confidence=np.ones(len(remaining)) if confidences is None else np.asarray(confidences,float);rng=np.random.default_rng(seed);planes=[]
    for _ in range(maximum_planes):
        if len(remaining)<minimum_support:break
        best_mask=None;best_score=-1.
        for _ in range(iterations):
            sample=remaining[rng.choice(len(remaining),3,replace=False)];normal=np.cross(sample[1]-sample[0],sample[2]-sample[0]);length=np.linalg.norm(normal)
            if length<1e-8:continue
            normal/=length;offset=-normal@sample[0];mask=np.abs(remaining@normal+offset)<=distance_threshold_m;score=float(np.sum(remaining_confidence[mask]))
            if np.sum(mask)>=minimum_support and score>best_score:best_score,best_mask=score,mask
        if best_mask is None:break
        normal,offset=_fit_plane(remaining[best_mask]);refined=np.abs(remaining@normal+offset)<=distance_threshold_m;support=remaining[refined];support_confidence=remaining_confidence[refined];centroid=support.mean(axis=0);confidence=float(np.mean(support_confidence)*min(1.,len(support)/(minimum_support*3)));planes.append(Plane(tuple(normal.tolist()),offset,confidence,len(support),tuple(centroid.tolist()),_patch(support),_classify(normal)));remaining,remaining_confidence=remaining[~refined],remaining_confidence[~refined]
    return planes
