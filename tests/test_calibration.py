import pytest
from wake.calibration.labels import closest_surface_point,plane_from_points
from wake.recording.dataset import DatasetSplit,split_sessions,validate_session_split

def test_plane_from_three_points():
    plane=plane_from_points((2,0,0),(2,1,0),(2,0,1));assert abs(abs(plane.normal_world[0])-1)<1e-9;assert abs(abs(plane.offset_m)-2)<1e-9
    point=closest_surface_point((1,3,4),plane);assert point[0]==pytest.approx(2)

def test_session_split_is_disjoint():
    split=split_sessions([f"data/session-{index}" for index in range(5)]);validate_session_split(split);assert set(split.train_session_ids).isdisjoint(split.validation_session_ids+split.test_session_ids)

def test_session_leakage_rejected():
    with pytest.raises(ValueError):validate_session_split(DatasetSplit(["a"],["a"],["b"]))
