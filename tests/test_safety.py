from wake.control.supervisor import SafetyAction,SafetySupervisor
from wake.types import SurfaceEstimate,SystemHealth
CFG={"max_pose_age_ms":100,"max_telemetry_age_ms":100,"minimum_battery_v":3.2,"return_battery_v":3.5,"stop_distance_m":.3,"caution_distance_m":.5,"emergency_distance_m":.15,"uncertainty_k":2,"max_unknown_speed_mps":.1,"max_known_speed_mps":.3,"confidence_min":.5}
def healthy():return SystemHealth(pose_age_ms=1,telemetry_age_ms=1,battery_v=4,model_calibrated=True)
def test_obstacle_causes_hold():assert SafetySupervisor(CFG).evaluate(healthy(),SurfaceEstimate(.9,.4,(1,0,0),.1,.1,.9,True)).action==SafetyAction.HOLD
def test_stale_pose():h=healthy();h.pose_age_ms=101;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.HOLD
def test_stale_telemetry():h=healthy();h.telemetry_age_ms=101;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.HOLD
def test_uncalibrated_blocks():h=healthy();h.model_calibrated=False;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.HOLD
def test_low_battery_policy():h=healthy();h.battery_v=3.4;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.RETURN_HOME
