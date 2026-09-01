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
    operational_min: list[float] | None = None
    operational_max: list[float] | None = None
    split_session_ids: dict[str, list[str]] | None = None

class LearnedFreeAirModel(FreeAirModel):
    calibrated=True
    def __init__(self, artifact: ModelArtifact) -> None: self.artifact=artifact
    @classmethod
    def load(cls,path: str|Path) -> "LearnedFreeAirModel":
        raw=json.loads(Path(path).read_text(encoding="utf-8")); required={"model_version","training_timestamp","features","normalization_mean","normalization_scale","dataset_ids","validation_metrics","configuration_hash","coefficients","intercept"}
        missing=required-raw.keys()
        if missing: raise UncalibratedModelError(f"invalid free-air artifact; missing {sorted(missing)}")
        return cls(ModelArtifact(**{k:raw.get(k) for k in ModelArtifact.__dataclass_fields__}))
    def predict(self,features: np.ndarray) -> np.ndarray:
        x=(np.asarray(features)-np.asarray(self.artifact.normalization_mean))/np.asarray(self.artifact.normalization_scale)
        return x@np.asarray(self.artifact.coefficients)+np.asarray(self.artifact.intercept)
    def in_operational_envelope(self,features:np.ndarray)->bool:
        if self.artifact.operational_min is None or self.artifact.operational_max is None:return False
        value=np.asarray(features);return bool(np.all(value>=np.asarray(self.artifact.operational_min)) and np.all(value<=np.asarray(self.artifact.operational_max)))
