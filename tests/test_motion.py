import numpy as np
import pytest
from wake.estimation.motion import PoseMotionEstimator
from wake.pose.transforms import quaternion_from_rpy
from wake.types import PoseSample
def test_polynomial_motion_derivatives():
    estimator=PoseMotionEstimator(7);result=None
    for index in range(7):
        t=index*.1;pose=PoseSample("wake-01",index,round(t*1e9),(t+0.5*t*t,0,0),quaternion_from_rpy(0,0,.2*t),1,0,0);result=estimator.update(pose)
    assert result.world_velocity_mps[0]==pytest.approx(1.6,abs=.02);assert result.world_acceleration_mps2[0]==pytest.approx(1,abs=.02);assert result.yaw_rate_rps==pytest.approx(.2,abs=.01)
def test_body_velocity_rotates():
    estimator=PoseMotionEstimator(3)
    for index in range(3):result=estimator.update(PoseSample("wake-01",index,index*100_000_000,(index*.1,0,0),quaternion_from_rpy(0,0,np.pi/2),1,0,0))
    assert result.body_velocity_mps[1]==pytest.approx(-1,abs=.01)
