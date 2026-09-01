import numpy as np
from wake.types import SynchronizedSample
from wake.estimation.motion import MotionFeatures

def instantaneous_features(sample:SynchronizedSample,motion:MotionFeatures|None=None) -> np.ndarray:
    t=sample.telemetry
    motors=np.asarray(t.motors);motion=motion or MotionFeatures();return np.concatenate([motors,[motors.mean(),motors.std(),t.battery_v],t.accel_body_g,t.gyro_body,t.attitude_rpy_rad,motion.world_velocity_mps,motion.body_velocity_mps,motion.world_acceleration_mps2,[motion.yaw_rate_rps]])
