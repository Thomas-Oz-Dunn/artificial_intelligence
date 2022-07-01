"""
Useful functions for Kalman Filter
"""
import numpy as np

THRESHOLD = 1e-6

class KalmanFilter:
    """
    A class for modelling Kalman Filters
    """
    def __init__(
        self, 
        state, 
        covariance, 
        transition_matrix
    ):
        self.state = state
        self.covariance = covariance
        self.transition_matrix = transition_matrix
        self.prcx_noise_cov = np.eye(state.shape() [0])
        self.input_effect = np.eye(state.shape() [0])
        self.control_input = np.zeros((state.shape() [0], 1))

    def run(
        self, 
        new_measure,
        measure_matrix,
        measure_cov,
        n_iter: int = 150
    ):
        for _ in np.arange(0, n_iter):
            self.predict()
            self.update(
                new_measure, 
                measure_matrix, 
                measure_cov)
        
        return (self.state, self.covariance)

    def predict(self) -> None:
        self.state = predict_state(
            state=self.state,
            transition_matrix=self.transition_matrix,
            control_effect=self.control_effect,
            control_input=self.control_input)

        self.covariance = predict_covariance(
            mean_covariance=self.covariance,
            transition_matrix=self.transition_matrix,
            prcx_noise_covariance=self.prcx_noise_cov)

    def update(
        self,
        new_measure, 
        measure_matrix, 
        measure_cov
    ) -> None:

        pred_mean = np.dot(measure_matrix, self.state)
        predict_cov = measure_cov + np.dot(measure_matrix, np.dot(self.covariance, measure_matrix.T))
        
        self.kalman_gain = np.dot(self.covariance, np.dot(measure_matrix.T, np.linalg.inv(predict_cov)))
        diff = (new_measure - pred_mean)
        self.state = self.state + np.dot(self.kalman_gain, diff)
        self.covariance = self.covariance - np.dot(self.kalman_gain, np.dot(predict_cov, self.kalman_gain.T))

def predict_covariance(
    mean_covariance,
    transition_matrix,
    prcx_noise_covariance
):
    transition_cov = np.dot(mean_covariance, transition_matrix.T)
    return np.dot(transition_matrix, transition_cov) + prcx_noise_covariance


def predict_state(
    state,
    transition_matrix,
    control_effect,
    control_input
):
    transition = np.dot(transition_matrix, state)
    controls = np.dot(control_effect, control_input)
    return transition + controls

