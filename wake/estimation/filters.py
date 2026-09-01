from dataclasses import dataclass
import numpy as np

@dataclass
class FilteredVector:
    raw: np.ndarray
    low_pass: np.ndarray
    high_pass: np.ndarray
    derivative: np.ndarray

class VectorFilter:
    def __init__(self, alpha: float=.25) -> None:
        if not 0 < alpha <= 1: raise ValueError("alpha must be in (0,1]")
        self.alpha=alpha; self.previous: np.ndarray|None=None; self.previous_time: float|None=None
    def update(self, raw: np.ndarray, timestamp_s: float) -> FilteredVector:
        raw=np.asarray(raw,float)
        if self.previous is None: low=raw.copy(); derivative=np.zeros_like(raw)
        else:
            low=self.alpha*raw+(1-self.alpha)*self.previous; dt=max(1e-9,timestamp_s-(self.previous_time or timestamp_s)); derivative=(raw-self.previous)/dt
        result=FilteredVector(raw.copy(),low,raw-low,derivative); self.previous=low; self.previous_time=timestamp_s; return result
