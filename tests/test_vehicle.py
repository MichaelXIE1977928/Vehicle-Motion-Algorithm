import pytest
import numpy as np
from vma.core.vehicle.bicycle import BicycleModel

def test_bicycle_derivative():
    params = {
        'mass': 1500, 'yaw_inertia': 2500, 'cg_to_front': 1.2, 'cg_to_rear': 1.6,
        'front_cornering_stiffness': 80000, 'rear_cornering_stiffness': 90000
    }
    veh = BicycleModel(params)
    state = np.array([0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0])
    inputs = np.array([0.0, 0.0])
    deriv = veh.state_derivative(0, state, inputs)
    assert deriv.shape == (8,)
    assert np.all(np.isfinite(deriv))
    assert abs(deriv[0] - state[3]) < 1e-9
