"""Surface detection/regression with uncertainty and operational envelope."""
from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
import json
import math
import numpy as np

from wake.estimation.residual import AerodynamicResidual, residual_feature_vector
from wake.types import SurfaceEstimate


class SurfaceModel(ABC):
    calibrated: bool
    @abstractmethod
    def estimate(self, residual: AerodynamicResidual) -> SurfaceEstimate | None: ...


class BaselineSurfaceModel(SurfaceModel):
    """Experimental visualization heuristic; never enables autonomy or mapping."""
    calibrated = False
    def estimate(self, residual: AerodynamicResidual) -> SurfaceEstimate | None:
        if not residual.persistent or residual.magnitude < 1e-9:
            return None
        direction = residual.acceleration
        norm = float(np.linalg.norm(direction))
        if norm < 1e-9:
            return None
        probability = min(.6, .15 + .2 * residual.magnitude)
        distance = max(.15, min(1., .8 - .2 * residual.magnitude))
        return SurfaceEstimate(probability, distance, tuple((direction / norm).tolist()), max(.15, .4 * distance), math.radians(30), min(.4, probability), False)


@dataclass
class SurfaceArtifact:
    model_version: str
    training_timestamp: str
    feature_names: list[str]
    normalization_mean: list[float]
    normalization_scale: list[float]
    classifier_weights: list[float]
    distance_weights: list[float]
    normal_weights: list[list[float]]
    distance_sigma_m: float
    angular_sigma_rad: float
    dataset_ids: list[str]
    split_session_ids: dict[str, list[str]]
    validation_metrics: dict[str, float]
    configuration_hash: str
    operational_min: list[float]
    operational_max: list[float]


def _sigmoid(value: np.ndarray | float) -> np.ndarray | float:
    return 1.0 / (1.0 + np.exp(-np.clip(value, -40, 40)))


class CalibratedSurfaceModel(SurfaceModel):
    calibrated = True
    def __init__(self, artifact: SurfaceArtifact) -> None:
        self.artifact = artifact
        self.last_in_envelope = False

    @classmethod
    def load(cls, path: str | Path) -> "CalibratedSurfaceModel":
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
        return cls(SurfaceArtifact(**raw))

    def estimate(self, residual: AerodynamicResidual) -> SurfaceEstimate | None:
        raw = residual_feature_vector(residual)
        minimum, maximum = np.asarray(self.artifact.operational_min), np.asarray(self.artifact.operational_max)
        self.last_in_envelope = bool(np.all(raw >= minimum) and np.all(raw <= maximum))
        mean, scale = np.asarray(self.artifact.normalization_mean), np.asarray(self.artifact.normalization_scale)
        x = np.concatenate([(raw - mean) / scale, [1.0]])
        probability = float(_sigmoid(x @ np.asarray(self.artifact.classifier_weights)))
        if probability < .05:
            return None
        distance = max(.01, float(x @ np.asarray(self.artifact.distance_weights)))
        normal = x @ np.asarray(self.artifact.normal_weights)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            return None
        confidence = probability * (1.0 if self.last_in_envelope else .1)
        return SurfaceEstimate(probability, distance, tuple((normal / norm).tolist()), self.artifact.distance_sigma_m, self.artifact.angular_sigma_rad, confidence, True)


def train_surface_model(features: np.ndarray, nearby: np.ndarray, distances: np.ndarray, normals: np.ndarray, *, validation: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray], dataset_ids: list[str], split_session_ids: dict[str, list[str]], configuration_hash: str) -> SurfaceArtifact:
    x = np.asarray(features, float); y = np.asarray(nearby, float)
    mean, scale = x.mean(axis=0), x.std(axis=0); scale[scale < 1e-9] = 1
    design = np.column_stack([(x - mean) / scale, np.ones(len(x))])
    classifier = np.zeros(design.shape[1])
    for iteration in range(1500):
        probability = _sigmoid(design @ classifier)
        gradient = design.T @ (probability - y) / len(y)
        gradient[:-1] += 1e-4 * classifier[:-1]
        classifier -= .2 / math.sqrt(1 + iteration / 100) * gradient
    positive = y > .5
    distance_weights, *_ = np.linalg.lstsq(design[positive], np.asarray(distances)[positive], rcond=None)
    normal_weights, *_ = np.linalg.lstsq(design[positive], np.asarray(normals)[positive], rcond=None)
    vx, vy, vd, vn = validation
    vdesign = np.column_stack([(np.asarray(vx)-mean)/scale, np.ones(len(vx))]); probability = _sigmoid(vdesign@classifier); predicted = probability >= .5
    true = np.asarray(vy)>.5; tp=np.sum(predicted&true);fp=np.sum(predicted&~true);fn=np.sum(~predicted&true);tn=np.sum(~predicted&~true)
    positive_validation = true
    distance_prediction = vdesign[positive_validation]@distance_weights; distance_error=distance_prediction-np.asarray(vd)[positive_validation]
    normal_prediction = vdesign[positive_validation]@normal_weights; normal_prediction/=np.linalg.norm(normal_prediction,axis=1,keepdims=True);true_normals=np.asarray(vn)[positive_validation];angles=np.arccos(np.clip(np.sum(normal_prediction*true_normals,axis=1),-1,1))
    metrics={"precision":float(tp/max(1,tp+fp)),"recall":float(tp/max(1,tp+fn)),"false_positive_rate":float(fp/max(1,fp+tn)),"distance_mae_m":float(np.mean(np.abs(distance_error))),"distance_p90_m":float(np.percentile(np.abs(distance_error),90)),"normal_mae_deg":float(np.degrees(np.mean(angles))),"held_out_obstacle_recall":float(tp/max(1,tp+fn))}
    names=["accel_x","accel_y","accel_z","gyro_x","gyro_y","gyro_z","magnitude","rolling_rms","rolling_variance","persistence","low_energy","high_energy","motor_imbalance"]
    return SurfaceArtifact("surface-linear-v1",datetime.now(timezone.utc).isoformat(),names,mean.tolist(),scale.tolist(),classifier.tolist(),distance_weights.tolist(),normal_weights.tolist(),float(max(.01,np.std(distance_error))),float(max(math.radians(1),np.std(angles))),dataset_ids,split_session_ids,metrics,configuration_hash,np.percentile(x,.5,axis=0).tolist(),np.percentile(x,99.5,axis=0).tolist())


def save_surface_artifact(artifact: SurfaceArtifact, path: str | Path) -> Path:
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(artifact), indent=2), encoding="utf-8")
    return path
