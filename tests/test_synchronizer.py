import pytest
from wake.telemetry.synchronizer import SampleSynchronizer,SynchronizationError
from wake.types import PoseSample,TelemetrySample

def pose(t,x,seq=0):return PoseSample("wake-01",seq,t,(x,0,0),(1,0,0,0),1,0,0)
def telemetry(t):return TelemetrySample("wake-01",1,t//1000,t+1_000_000,(0,0,1),(0,0,0),(0,0,0),(1000,)*4,4)
def test_interpolation():
    s=SampleSynchronizer(20,20);s.add_pose(pose(0,0));s.add_pose(pose(10_000_000,1));out=s.synchronize(telemetry(5_000_000),5_000_000);assert out.pose.position_world_m[0]==pytest.approx(.5)
def test_stale_rejection():
    s=SampleSynchronizer(5,5);s.add_pose(pose(0,0));s.add_pose(pose(10_000_000,1))
    with pytest.raises(SynchronizationError):s.synchronize(telemetry(5_000_000),5_000_000)
def test_out_of_order_pose_is_sorted():
    s=SampleSynchronizer();s.add_pose(pose(10_000_000,1));s.add_pose(pose(0,0));assert s.synchronize(telemetry(5_000_000),5_000_000).pose.position_world_m[0]==pytest.approx(.5)
