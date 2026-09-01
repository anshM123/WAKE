"""Filtered derivatives from timestamped global pose history."""
from __future__ import annotations
from dataclasses import dataclass
from collections import deque
import math
import numpy as np
from wake.pose.transforms import quaternion_to_matrix
from wake.types import PoseSample,Vec3

@dataclass(frozen=True)
class MotionFeatures:
    world_velocity_mps:Vec3=(0.,0.,0.)
    body_velocity_mps:Vec3=(0.,0.,0.)
    world_acceleration_mps2:Vec3=(0.,0.,0.)
    yaw_rate_rps:float=0.

class PoseMotionEstimator:
    """Local polynomial derivative estimator; avoids raw two-frame differences."""
    def __init__(self,window_size:int=7)->None:
        if window_size<3:raise ValueError("window_size must be at least 3")
        self.history:deque[PoseSample]=deque(maxlen=window_size)
    def update(self,pose:PoseSample)->MotionFeatures:
        self.history.append(pose)
        if len(self.history)<3:return MotionFeatures()
        times=np.asarray([(item.timestamp_ns-pose.timestamp_ns)/1e9 for item in self.history]);positions=np.asarray([item.position_world_m for item in self.history]);design=np.column_stack([np.ones(len(times)),times,times**2]);coefficients,*_=np.linalg.lstsq(design,positions,rcond=None);velocity=coefficients[1];acceleration=2*coefficients[2]
        yaws=np.unwrap([math.atan2(quaternion_to_matrix(item.rotation_world_from_body)[1,0],quaternion_to_matrix(item.rotation_world_from_body)[0,0]) for item in self.history]);yaw_coefficients,*_=np.linalg.lstsq(design,np.asarray(yaws),rcond=None);body_velocity=quaternion_to_matrix(pose.rotation_world_from_body).T@velocity
        return MotionFeatures(tuple(velocity.tolist()),tuple(body_velocity.tolist()),tuple(acceleration.tolist()),float(yaw_coefficients[1]))
