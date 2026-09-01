import numpy as np
from wake.estimation.residual import ResidualEstimator
def test_persistence_required():
    estimator=ResidualEstimator(3,.1);assert not estimator.calculate(np.ones(6),np.zeros(6)).persistent;estimator.calculate(np.ones(6),np.zeros(6));assert estimator.calculate(np.ones(6),np.zeros(6)).persistent
