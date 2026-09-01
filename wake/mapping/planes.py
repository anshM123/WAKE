"""Robust plane fitting is optional until scipy/sklearn science extras are installed."""
from dataclasses import dataclass
from wake.types import Vec3
@dataclass(frozen=True)
class Plane: normal_world:Vec3;offset_m:float;confidence:float;support_count:int
