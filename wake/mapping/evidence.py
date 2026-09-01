from dataclasses import dataclass
from wake.types import Vec3
@dataclass(frozen=True)
class MapEvidence:
    origin_world_m:Vec3
    direction_world:Vec3
    distance_m:float
    distance_sigma_m:float
    angular_sigma_rad:float
    confidence:float
