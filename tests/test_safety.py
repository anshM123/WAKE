from wake.control.supervisor import SafetyAction,SafetySupervisor
from wake.types import SurfaceEstimate,SystemHealth
CFG={"max_pose_age_ms":100,"max_telemetry_age_ms":100,"minimum_battery_v":3.2,"return_battery_v":3.5,"stop_distance_m":.3,"caution_distance_m":.5,"emergency_distance_m":.15,"uncertainty_k":2,"max_unknown_speed_mps":.1,"max_known_speed_mps":.3,"confidence_min":.5}
def healthy():return SystemHealth(pose_age_ms=1,telemetry_age_ms=1,battery_v=4,model_calibrated=True)
def test_obstacle_causes_hold():assert SafetySupervisor(CFG).evaluate(healthy(),SurfaceEstimate(.9,.4,(1,0,0),.1,.1,.9,True)).action==SafetyAction.HOLD
def test_stale_pose():h=healthy();h.pose_age_ms=101;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.HOLD
def test_stale_telemetry():h=healthy();h.telemetry_age_ms=101;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.HOLD
def test_uncalibrated_blocks():h=healthy();h.model_calibrated=False;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.HOLD
def test_low_battery_policy():h=healthy();h.battery_v=3.4;assert SafetySupervisor(CFG).evaluate(h).action==SafetyAction.RETURN_HOME

def test_dynamic_stop_distance_and_unsafe_configuration():
    config={**CFG,"maximum_deceleration_mps2":1.,"drone_radius_m":.1,"reaction_time_s":.2,"command_timeout_ms":100};surface=SurfaceEstimate(.9,.5,(1,0,0),.1,.1,.9,True);decision=SafetySupervisor(config).evaluate(healthy(),surface,current_speed_mps=.5);assert decision.action==SafetyAction.HOLD;assert decision.required_stop_distance_m>.3

def test_unknown_space_speed_limit():
    config={**CFG,"maximum_deceleration_mps2":1.,"drone_radius_m":.1};decision=SafetySupervisor(config).evaluate(healthy(),moving_into_unknown=True);assert decision.speed_limit_mps==.1

def test_bad_clock_and_out_of_envelope_fail_closed():
    config={**CFG,"require_clock_sync":True,"max_clock_age_ms":3000,"require_operational_envelope":True};health=healthy();health.clock_model_age_ms=4000;health.clock_sync_confidence=0;health.model_in_operational_envelope=False;assert SafetySupervisor(config).evaluate(health).action==SafetyAction.HOLD
