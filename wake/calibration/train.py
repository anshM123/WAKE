from __future__ import annotations
from datetime import datetime,timezone
from pathlib import Path
import json,numpy as np
from wake.estimation.free_air import ModelArtifact

def train_linear_free_air(features:np.ndarray,targets:np.ndarray,*,feature_names:list[str],dataset_ids:list[str],configuration_hash:str,model_version:str="linear-v1")->ModelArtifact:
    x=np.asarray(features,float);y=np.asarray(targets,float)
    if x.ndim!=2 or y.ndim!=2 or len(x)!=len(y):raise ValueError("features and targets must be aligned 2-D arrays")
    mean=x.mean(axis=0);scale=x.std(axis=0);scale[scale<1e-9]=1;normalized=(x-mean)/scale;design=np.column_stack([normalized,np.ones(len(x))]);weights,*_=np.linalg.lstsq(design,y,rcond=None);prediction=design@weights;mae=float(np.mean(np.abs(prediction-y)))
    return ModelArtifact(model_version,datetime.now(timezone.utc).isoformat(),feature_names,mean.tolist(),scale.tolist(),dataset_ids,{"validation_mae_training_only":mae},configuration_hash,weights[:-1].tolist(),weights[-1].tolist())

def save_artifact(artifact:ModelArtifact,path:str|Path)->Path:
    from dataclasses import asdict
    path=Path(path);path.write_text(json.dumps(asdict(artifact),indent=2),encoding="utf-8");return path
