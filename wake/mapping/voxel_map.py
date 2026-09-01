from __future__ import annotations
from dataclasses import dataclass
import math,time
from wake.types import Vec3

@dataclass
class Voxel:
    occupancy_log_odds:float=0.0
    observation_count:int=0
    confidence:float=0.0
    last_update_time_ns:int=0
    @property
    def occupancy_probability(self)->float:return 1/(1+math.exp(-self.occupancy_log_odds))

class SparseVoxelMap:
    def __init__(self,voxel_size_m:float=.075,log_odds_min:float=-4,log_odds_max:float=4)->None:
        if voxel_size_m<=0:raise ValueError("voxel_size_m must be positive")
        self.voxel_size_m=voxel_size_m;self.log_odds_min=log_odds_min;self.log_odds_max=log_odds_max;self.voxels:dict[tuple[int,int,int],Voxel]={}
    def index(self,point:Vec3)->tuple[int,int,int]:return tuple(math.floor(v/self.voxel_size_m) for v in point)
    def get(self,index:tuple[int,int,int])->Voxel|None:return self.voxels.get(index)
    def update(self,index:tuple[int,int,int],log_odds_delta:float,confidence:float,timestamp_ns:int|None=None)->Voxel:
        voxel=self.voxels.setdefault(index,Voxel()); old=voxel.occupancy_log_odds; voxel.occupancy_log_odds=max(self.log_odds_min,min(self.log_odds_max,old+log_odds_delta)); voxel.observation_count+=1
        agreement=1.0 if old==0 or old*log_odds_delta>=0 else 0.0; voxel.confidence=max(0,min(1,(voxel.confidence*(voxel.observation_count-1)+confidence*agreement)/voxel.observation_count));voxel.last_update_time_ns=timestamp_ns or time.time_ns();return voxel
