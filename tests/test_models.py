import numpy as np
from wake.calibration.train import train_linear_free_air
from wake.estimation.free_air import LearnedFreeAirModel
from wake.estimation.residual import AerodynamicResidual,residual_feature_vector
from wake.estimation.surface_model import CalibratedSurfaceModel,train_surface_model

def test_free_air_uses_held_out_validation(tmp_path):
    rng=np.random.default_rng(1);x=rng.normal(size=(100,4));weights=rng.normal(size=(4,3));y=x@weights
    artifact=train_linear_free_air(x[:80],y[:80],feature_names=["a","b","c","d"],dataset_ids=["s1","s2"],configuration_hash="x",validation_features=x[80:],validation_targets=y[80:],split_session_ids={"train":["s1"],"validation":["s2"],"test":["s3"]})
    assert artifact.validation_metrics["validation_scope"]=="held-out-sessions"
    assert max(artifact.validation_metrics["mae_per_axis"])<1e-10

def test_surface_model_trains_and_predicts():
    rng=np.random.default_rng(2);x=rng.normal(size=(300,13));nearby=(x[:,6]>.1).astype(float);distance=np.clip(.5-.1*x[:,6],.1,1);normals=np.tile([1.,0,0],(300,1))
    artifact=train_surface_model(x[:220],nearby[:220],distance[:220],normals[:220],normals[:220],validation=(x[220:],nearby[220:],distance[220:],normals[220:],normals[220:]),dataset_ids=["a","b","c"],split_session_ids={"train":["a"],"validation":["b"],"test":["c"]},configuration_hash="hash")
    model=CalibratedSurfaceModel(artifact);features=x[np.argmax(x[:,6])];residual=AerodynamicResidual(features[:3],features[3:6],features[6],True,*features[7:13]);estimate=model.estimate(residual)
    assert estimate is not None and estimate.calibrated
    assert estimate.distance_sigma_m>0 and estimate.angular_sigma_rad>0
    assert artifact.validation_metrics["recall_by_distance"]

def test_residual_features_include_persistence():
    residual=AerodynamicResidual(np.zeros(3),np.zeros(3),1,True,1,.1,1,2,3,4)
    assert residual_feature_vector(residual).shape==(13,)
