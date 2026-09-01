"""Synthetic box-room simulator for software tests only, not wake physics."""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from wake.types import PoseSample,SurfaceEstimate

@dataclass(frozen=True)
class BoxRoom:
    width_m:float=4.;depth_m:float=5.;height_m:float=2.5
    def nearest_surface(self,position:tuple[float,float,float],noise_sigma_m:float=0.,rng=None)->SurfaceEstimate:
        x,y,z=position;candidates=[(x,(1,0,0)),(self.width_m-x,(-1,0,0)),(y,(0,1,0)),(self.depth_m-y,(0,-1,0)),(z,(0,0,1)),(self.height_m-z,(0,0,-1))];distance,normal=min(candidates,key=lambda item:item[0]);rng=rng or np.random.default_rng(0);measured=max(.01,float(distance+rng.normal(0,noise_sigma_m)));return SurfaceEstimate(.95,measured,normal,max(.01,noise_sigma_m),.05,.9,True)
    def trajectory(self,count:int=20)->list[PoseSample]:
        return [PoseSample("wake-01",index,index*10_000_000,(0.5+(self.width_m-1)*index/(count-1),self.depth_m/2,self.height_m/2),(1,0,0,0),1,0,0) for index in range(count)]
