"""
Kalman Filter test
"""

import numpy as np

import kalman_filter as kf

def test_predict_state():

    # Input
    time_step = 1
    state = np.array([0, 1, 0, 1])
    transition_matrix = np.array([
        [1, 0, time_step, 0],
        [0, 1, 0, time_step],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])

    # Output
    predicted_state = kf.predict_state(
        state=state,
        transition_matrix=transition_matrix,
    )

    # Test
    actual_state = np.array([0, 2, 0, 1])
    np.assert_equal(predicted_state, actual_state)

test_predict_state()