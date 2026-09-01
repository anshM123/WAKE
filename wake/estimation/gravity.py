"""Accelerometer specific-force conversion in arbitrary orientation."""
import numpy as np
from wake.pose.transforms import quaternion_to_matrix
from wake.types import Quaternion, Vec3

STANDARD_GRAVITY_MPS2 = 9.80665

def gravity_body_mps2(rotation_world_from_body: Quaternion) -> np.ndarray:
    """Return physical gravity [0,0,-g] expressed in DRONE_BODY."""
    return quaternion_to_matrix(rotation_world_from_body).T @ np.array([0.0, 0.0, -STANDARD_GRAVITY_MPS2])

def translational_acceleration_world(accel_specific_force_body_g: Vec3, rotation_world_from_body: Quaternion) -> np.ndarray:
    """Convert accelerometer specific force to inertial world acceleration.

    Accelerometers measure proper/specific force, not translational acceleration:
    a_world = R_world_from_body * f_body + gravity_world.
    """
    specific=np.asarray(accel_specific_force_body_g,float)*STANDARD_GRAVITY_MPS2
    return quaternion_to_matrix(rotation_world_from_body) @ specific + np.array([0.0,0.0,-STANDARD_GRAVITY_MPS2])
