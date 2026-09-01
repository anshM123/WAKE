"""Session-separated trainers and versioned artifact persistence."""
from __future__ import annotations
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
import json
import numpy as np
from wake.estimation.free_air import ModelArtifact

def _axis_metrics(truth:np.ndarray,predicted:np.ndarray)->dict[str,object]:
    error=predicted-truth
    return {"mae_per_axis":np.mean(np.abs(error),axis=0).tolist(),"rmse_per_axis":np.sqrt(np.mean(error**2,axis=0)).tolist(),"p90_absolute_error_per_axis":np.percentile(np.abs(error),90,axis=0).tolist()}

def train_linear_free_air(features:np.ndarray,targets:np.ndarray,*,feature_names:list[str],dataset_ids:list[str],configuration_hash:str,model_version:str="linear-v2",validation_features:np.ndarray|None=None,validation_targets:np.ndarray|None=None,split_session_ids:dict[str,list[str]]|None=None)->ModelArtifact:
    x,y=np.asarray(features,float),np.asarray(targets,float)
    if x.ndim!=2 or y.ndim!=2 or len(x)!=len(y):raise ValueError("features and targets must be aligned 2-D arrays")
    mean,scale=x.mean(axis=0),x.std(axis=0);scale[scale<1e-9]=1;design=np.column_stack([(x-mean)/scale,np.ones(len(x))]);weights,*_=np.linalg.lstsq(design,y,rcond=None)
    vx=x if validation_features is None else np.asarray(validation_features,float);vy=y if validation_targets is None else np.asarray(validation_targets,float);prediction=np.column_stack([(vx-mean)/scale,np.ones(len(vx))])@weights;metrics=_axis_metrics(vy,prediction);metrics["validation_scope"]="held-out-sessions" if validation_features is not None else "training-only-NOT-FOR-AUTONOMY"
    return ModelArtifact(model_version,datetime.now(timezone.utc).isoformat(),feature_names,mean.tolist(),scale.tolist(),dataset_ids,metrics,configuration_hash,weights[:-1].tolist(),weights[-1].tolist(),np.percentile(x,.5,axis=0).tolist(),np.percentile(x,99.5,axis=0).tolist(),split_session_ids)

def save_artifact(artifact:ModelArtifact,path:str|Path)->Path:
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True);path.write_text(json.dumps(asdict(artifact),indent=2),encoding="utf-8");return path
