import math,numpy as np
from wake.pose.transforms import compose_body_pose,quaternion_from_rpy,quaternion_to_matrix,transform_point

def test_identity():assert np.allclose(quaternion_to_matrix((1,0,0,0)),np.eye(3))
def test_axis_rotations():
    assert np.allclose(quaternion_to_matrix(quaternion_from_rpy(math.pi/2,0,0))@[0,1,0],[0,0,1],atol=1e-7)
    assert np.allclose(quaternion_to_matrix(quaternion_from_rpy(0,math.pi/2,0))@[1,0,0],[0,0,-1],atol=1e-7)
    assert np.allclose(quaternion_to_matrix(quaternion_from_rpy(0,0,math.pi/2))@[1,0,0],[0,1,0],atol=1e-7)
def test_camera_to_world():
    T=np.eye(4);T[:3,3]=[1,2,3];assert transform_point(T,(0,0,0))==(1.,2.,3.)
def test_tag_to_drone_offset():
    camera=np.eye(4);tag=np.eye(4);body=np.eye(4);body[0,3]=.1;position,_=compose_body_pose(camera,tag,body);assert position==(.1,0.,0.)
