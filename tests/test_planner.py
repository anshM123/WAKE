from wake.planning.explorer import Explorer,ExplorationState,PlannerObservation
from wake.planning.frontier import Frontier
from wake.planning.trajectory import known_safe_grid_path
from wake.simulation import BoxRoom
from wake.types import SystemHealth
def healthy():return SystemHealth(tag_visible=True,clock_sync_confidence=1,clock_model_age_ms=1,model_calibrated=True,model_in_operational_envelope=True)
def test_frontier_scoring_rejects_unreachable():
    explorer=Explorer();unsafe=Frontier((1,0,0),100,0,0,0,0,False);safe=Frontier((0,1,0),5,1,0,0,0,True);assert explorer.choose_frontier([unsafe,safe])==safe
def test_no_path_through_unknown():
    safe={(0,0,0),(1,0,0),(2,0,0)};assert known_safe_grid_path((0,0,0),(2,0,0),safe)==[(0,0,0),(1,0,0),(2,0,0)];assert known_safe_grid_path((0,0,0),(0,2,0),safe) is None
def test_lost_tag_relocalizes_and_return_is_reversed():
    explorer=Explorer();pose=BoxRoom().trajectory(3)[0];explorer.update(PlannerObservation(pose,healthy(),None));bad=healthy();bad.tag_visible=False;assert explorer.update(PlannerObservation(None,bad,None))==ExplorationState.RELOCALIZE;assert explorer.safe_return()==[pose]
def test_synthetic_room_surface_is_calibrated():assert BoxRoom().nearest_surface((.4,2,1)).distance_m==.4
