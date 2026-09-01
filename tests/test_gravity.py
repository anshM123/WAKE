import math,numpy as np
from wake.estimation.gravity import gravity_body_mps2,translational_acceleration_world,STANDARD_GRAVITY_MPS2
from wake.pose.transforms import quaternion_from_rpy

def test_level_specific_force_is_stationary():assert np.allclose(translational_acceleration_world((0,0,1),(1,0,0,0)),0)
def test_roll_gravity_body():assert np.allclose(gravity_body_mps2(quaternion_from_rpy(math.pi/2,0,0)),[0,-STANDARD_GRAVITY_MPS2,0],atol=1e-7)
def test_pitch_gravity_body():assert np.allclose(gravity_body_mps2(quaternion_from_rpy(0,math.pi/2,0)),[STANDARD_GRAVITY_MPS2,0,0],atol=1e-7)
