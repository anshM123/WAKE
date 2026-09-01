import pytest
from wake.calibration.labels import WorldPlane,closest_surface_point,plane_from_points,wall_geometry_label
from wake.types import PoseSample
from wake.recording.dataset import DatasetSplit,split_sessions,validate_session_split

def test_plane_from_three_points():
    plane=plane_from_points((2,0,0),(2,1,0),(2,0,1));assert abs(abs(plane.normal_world[0])-1)<1e-9;assert abs(abs(plane.offset_m)-2)<1e-9
    point=closest_surface_point((1,3,4),plane);assert point[0]==pytest.approx(2)

def test_session_split_is_disjoint():
    split=split_sessions([f"data/session-{index}" for index in range(5)]);validate_session_split(split);assert set(split.train_session_ids).isdisjoint(split.validation_session_ids+split.test_session_ids)

def test_session_leakage_rejected():
    with pytest.raises(ValueError):validate_session_split(DatasetSplit(["a"],["a"],["b"]))

def test_surface_direction_is_distinct_from_plane_normal():
    pose=PoseSample("wake-01",1,1,(3,0,0),(1,0,0,0),1,0,0);distance,direction,normal=wall_geometry_label(pose,WorldPlane((1,0,0),-2));assert distance==1;assert direction==(-1.,0.,0.);assert normal==(1.,0.,0.)
