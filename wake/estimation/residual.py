from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class AerodynamicResidual:
    acceleration: np.ndarray
    gyro: np.ndarray
    magnitude: float
    persistent: bool

class ResidualEstimator:
    def __init__(self,persistence_samples:int=5,threshold:float=.15) -> None: self.persistence_samples=persistence_samples; self.threshold=threshold; self._history:list[float]=[]
    def calculate(self,actual:np.ndarray,expected:np.ndarray) -> AerodynamicResidual:
        residual=np.asarray(actual)-np.asarray(expected); magnitude=float(np.linalg.norm(residual)); self._history=(self._history+[magnitude])[-self.persistence_samples:]
        persistent=len(self._history)==self.persistence_samples and all(v>=self.threshold for v in self._history)
        return AerodynamicResidual(residual[:3],residual[3:6],magnitude,persistent)
