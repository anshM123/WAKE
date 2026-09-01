import json
import yaml
from wake.cli import main
from wake.config import autonomy_blockers

def test_simulation_command_is_operational(capsys):
    assert main(["simulate"])==0;payload=json.loads(capsys.readouterr().out);assert payload["poses"]>0

def test_define_plane_command(tmp_path):
    output=tmp_path/"plane.yaml";assert main(["define-plane","--points","2","0","0","2","1","0","2","0","1","--output",str(output)])==0;assert yaml.safe_load(output.read_text())["reference_plane"]["normal_world"] is not None

def test_autonomy_blockers_are_specific(tmp_path):
    camera={"camera":{"calibration_file":str(tmp_path/"missing.yaml")},"transforms":{"T_world_from_tag":None,"T_body_from_camera":None},"frame_check_confirmed":False};wake={"mode":"AUTONOMOUS","control":{"matrix_high_level_interface_verified":False,"manual_mapping_validated":False},"models":{"free_air_path":None,"surface_path":None}};safety={"geofence":None,"caution_distance_m":None,"stop_distance_m":None,"emergency_distance_m":None,"minimum_battery_v":None,"return_battery_v":None,"maximum_deceleration_mps2":None,"drone_radius_m":None,"detection_recall_min":.95};blockers=autonomy_blockers(wake,safety,camera,{"status":"UNCALIBRATED"});assert "Matrix high-level interface is unverified" in blockers;assert "camera calibration is missing" in blockers;assert "held-out obstacle recall is insufficient" in blockers
