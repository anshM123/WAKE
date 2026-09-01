import numpy as np
from wake.types import SynchronizedSample

def instantaneous_features(sample:SynchronizedSample) -> np.ndarray:
    t=sample.telemetry
    motors=np.asarray(t.motors); return np.concatenate([motors,[motors.mean(),motors.std(),t.battery_v],t.accel_body_g,t.gyro_body,t.attitude_rpy_rad])
