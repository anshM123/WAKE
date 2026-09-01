from dataclasses import dataclass
from wake.types import Vec3
@dataclass(frozen=True)
class Frontier: position_world_m:Vec3;information_gain:float;travel_cost:float;collision_risk:float;localization_risk:float
def score(frontier:Frontier)->float:return frontier.information_gain-frontier.travel_cost-frontier.collision_risk-frontier.localization_risk
