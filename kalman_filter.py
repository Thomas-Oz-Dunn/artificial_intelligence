"""
Useful functions for Kalman Filter

Usage
-----
import kalman_filter as kf
"""

import numpy as np
class KalmanFilter:
    """
    A class for modelling Kalman Filters
    """
    def __init__(
        self, 
        state_vec: np.array, 
        covariance: np.array, 
        transition_matrix: np.array
    ):
        """
        Initialize Kalman Filter

        Parameters
        ----------
        state_vec
            Vector describing initial state

        covariance
            Covariance of state vector

        transition_matrix
            Transformation of state during timestep
        """
        self.state = state_vec
        self.covariance = covariance
        self.transition_matrix = transition_matrix
        self.prcx_noise_cov = np.eye(state_vec.shape() [0])
        self.input_effect = np.eye(state_vec.shape() [0])
        self.control_input = np.zeros((state_vec.shape() [0], 1))
        self.measurement_matrix = np.zeros()

    def run(
        self, 
        measure_state: np.array,
        measure_cov: np.array,
        n_iter: int = 150
    ):
        """
        Run Model

        Parameters
        ----------
        measure_state
            measured state vector

        measure_cov
            measured covariancee

        n_iter
            number of iterations to run
        """
        for _ in np.arange(0, n_iter):
            self.predict()
            self.update(
                measure_state, 
                measure_cov)
        
        return (self.state, self.covariance)

    def predict(self) -> None:
        """
        Predict measurement based on internal model
        """
        self._predict_state()
        self._predict_covariance()

    def update(
        self,
        measure_state: np.array, 
        measure_cov: np.array
    ) -> None:
        """
        Update model with new measurement

        Parameters
        ----------
        measure_state
            Measured state vector

        measure_cov
            Measured covariance
        """
        pred_mean = np.dot(self.measure_matrix, self.state)
        predict_cov = measure_cov + np.dot(self.measure_matrix, np.dot(self.covariance, self.measure_matrix.T))
        
        self.kalman_gain = np.dot(self.covariance, np.dot(self.measure_matrix.T, np.linalg.inv(predict_cov)))
        diff = (measure_state - pred_mean)
        self.state = self.state + np.dot(self.kalman_gain, diff)
        self.covariance = self.covariance - np.dot(self.kalman_gain, np.dot(predict_cov, self.kalman_gain.T))


    def _predict_covariance(self):
        """
        Predict covariance based on last time step
        """
        transition_cov = np.dot(self.covariance, self.transition_matrix.T)
        self.covariance = np.dot(self.transition_matrix, transition_cov) + self.prcx_noise_covariance


    def _predict_state(self):
        """
        Predict state based on last time step
        """
        transition = np.dot(self.transition_matrix, self.state)
        controls = np.dot(self.control_effect, self.control_input)
        self.state = transition + controls

