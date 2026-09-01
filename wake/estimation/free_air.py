from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from pathlib import Path
from typing import Any
import json, numpy as np

class UncalibratedModelError(RuntimeError): pass

class FreeAirModel(ABC):
    calibrated: bool
    @abstractmethod
    def predict(self, features: np.ndarray) -> np.ndarray: ...

class BaselineFreeAirModel(FreeAirModel):
    """Debug-only ridge-like linear model; zero residual expectation by default."""
    calibrated=False
    def __init__(self, coefficients: np.ndarray|None=None, intercept: np.ndarray|None=None) -> None: self.coefficients=coefficients; self.intercept=np.zeros(6) if intercept is None else np.asarray(intercept)
    def predict(self, features: np.ndarray) -> np.ndarray:
        if self.coefficients is None: return self.intercept.copy()
        return np.asarray(features)@self.coefficients+self.intercept

@dataclass
class ModelArtifact:
    model_version: str
    training_timestamp: str
    features: list[str]
    normalization_mean: list[float]
    normalization_scale: list[float]
    dataset_ids: list[str]
    validation_metrics: dict[str,float]
    configuration_hash: str
    coefficients: list[list[float]]
    intercept: list[float]

class LearnedFreeAirModel(FreeAirModel):
    calibrated=True
    def __init__(self, artifact: ModelArtifact) -> None: self.artifact=artifact
    @classmethod
    def load(cls,path: str|Path) -> "LearnedFreeAirModel":
        raw=json.loads(Path(path).read_text(encoding="utf-8")); required={f.name for f in ModelArtifact.__dataclass_fields__.values()}
        missing=required-raw.keys()
        if missing: raise UncalibratedModelError(f"invalid free-air artifact; missing {sorted(missing)}")
        return cls(ModelArtifact(**{k:raw[k] for k in required}))
    def predict(self,features: np.ndarray) -> np.ndarray:
        x=(np.asarray(features)-np.asarray(self.artifact.normalization_mean))/np.asarray(self.artifact.normalization_scale)
        return x@np.asarray(self.artifact.coefficients)+np.asarray(self.artifact.intercept)
