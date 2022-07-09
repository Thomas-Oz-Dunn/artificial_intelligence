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
        measurement,
        measure_matrix,
        measure_cov,
        n_iter: int = 150
    ):
        for _ in np.arange(0, n_iter):
            self.predict()
            self.update(
                measurement, 
                measure_matrix, 
                measure_cov)
        
        return (self.state, self.covariance)

    def predict(self) -> None:
        self._predict_state()
        self._predict_covariance()

    def update(
        self,
        measurement, 
        measure_matrix, 
        measure_cov
    ) -> None:

        pred_mean = np.dot(measure_matrix, self.state)
        predict_cov = measure_cov + np.dot(measure_matrix, np.dot(self.covariance, measure_matrix.T))
        
        self.kalman_gain = np.dot(self.covariance, np.dot(measure_matrix.T, np.linalg.inv(predict_cov)))
        diff = (measurement - pred_mean)
        self.state = self.state + np.dot(self.kalman_gain, diff)
        self.covariance = self.covariance - np.dot(self.kalman_gain, np.dot(predict_cov, self.kalman_gain.T))


    def _predict_covariance(self):
        transition_cov = np.dot(self.covariance, self.transition_matrix.T)
        self.covariance = np.dot(self.transition_matrix, transition_cov) + self.prcx_noise_covariance


    def _predict_state(self):
        transition = np.dot(self.transition_matrix, self.state)
        controls = np.dot(self.control_effect, self.control_input)
        self.state = transition + controls

