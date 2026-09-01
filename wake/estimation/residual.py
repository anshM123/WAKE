from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class AerodynamicResidual:
    acceleration: np.ndarray
    gyro: np.ndarray
    magnitude: float
    persistent: bool
    rolling_rms: float = 0.0
    rolling_variance: float = 0.0
    persistence_score: float = 0.0
    low_band_energy: float = 0.0
    high_band_energy: float = 0.0
    motor_imbalance: float = 0.0

class ResidualEstimator:
    def __init__(self,persistence_samples:int=5,threshold:float=.15) -> None: self.persistence_samples=persistence_samples; self.threshold=threshold; self._history:list[float]=[]
    def calculate(self,actual:np.ndarray,expected:np.ndarray,motors:np.ndarray|None=None) -> AerodynamicResidual:
        residual=np.asarray(actual)-np.asarray(expected); magnitude=float(np.linalg.norm(residual)); self._history=(self._history+[magnitude])[-max(self.persistence_samples,16):]
        recent=np.asarray(self._history[-self.persistence_samples:]); persistence_score=float(np.mean(recent>=self.threshold)) if len(recent) else 0.0; persistent=len(recent)==self.persistence_samples and persistence_score==1.0
        spectrum=np.abs(np.fft.rfft(np.asarray(self._history)-np.mean(self._history)))**2 if len(self._history)>=8 else np.zeros(5);split=max(1,len(spectrum)//2)
        return AerodynamicResidual(residual[:3],residual[3:6],magnitude,persistent,float(np.sqrt(np.mean(recent**2))),float(np.var(recent)),persistence_score,float(np.sum(spectrum[1:split])),float(np.sum(spectrum[split:])),0.0 if motors is None else float(np.std(motors)))

def residual_feature_vector(residual:AerodynamicResidual)->np.ndarray:
    return np.asarray([*residual.acceleration,*residual.gyro,residual.magnitude,residual.rolling_rms,residual.rolling_variance,residual.persistence_score,residual.low_band_energy,residual.high_band_energy,residual.motor_imbalance],dtype=float)
