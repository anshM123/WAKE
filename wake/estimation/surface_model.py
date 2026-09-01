from __future__ import annotations
from abc import ABC,abstractmethod
import math,numpy as np
from wake.estimation.residual import AerodynamicResidual
from wake.types import SurfaceEstimate

class SurfaceModel(ABC):
    calibrated:bool
    @abstractmethod
    def estimate(self,residual:AerodynamicResidual)->SurfaceEstimate|None: ...

class BaselineSurfaceModel(SurfaceModel):
    """Experimental debug heuristic. It must never enable autonomous flight."""
    calibrated=False
    def estimate(self,residual:AerodynamicResidual)->SurfaceEstimate|None:
        if not residual.persistent or residual.magnitude<1e-9:return None
        direction=residual.acceleration; norm=float(np.linalg.norm(direction))
        if norm<1e-9:return None
        probability=min(.6,.15+.2*residual.magnitude); distance=max(.15,min(1.,.8-.2*residual.magnitude))
        return SurfaceEstimate(probability,distance,tuple((direction/norm).tolist()),max(.15,.4*distance),math.radians(30),min(.4,probability),False)

class CalibratedSurfaceModel(SurfaceModel):
    calibrated=True
    def __init__(self,predictor):self.predictor=predictor
    def estimate(self,residual:AerodynamicResidual)->SurfaceEstimate|None:return self.predictor(residual)
