
"""
Interacting Multi Model Markov Decsion Process

Usage
-----
import multi_model as imm
"""

import numpy as np
import typing as typing

import kalman_filter as kf

class InteractiveMultiModel:
    """
    A class for Interactive MultiModel (IMM)
    """
    def __init__(
        self,
        filters: typing.Iterable[kf.KalmanFilter],
        confidences: typing.Iterable[float],
        markov_transition: np.array
    ) -> None:
        """
        Initialize Interactive Multiple Models

        Parameters
        ----------
        filters : kalman_filter.KalmanFilter
            Iterable of Kalman Filter objects

        confidences : float
            Confidence in each filter

        markov_transision: np.ndarray
            Transition probability matrix between filters
        """
        self.filters = filters
        self.MU = confidences
        self.markov_transition = markov_transition
        self.state = np.zeros(filters[0].state.shape)
        self.covariance = np.zeros(filters[0].covariance.shape)
        self.num_filters = len(filters)
        self.omega = np.zeros([self.num_filters, self.num_filters])

    def run(
        self,
        measure_state: np.array,
        measure_cov: np.array,
        n_iter: int = 150
    ) -> None:
        """
        Run Model

        Parameters
        ----------
        measure_state
            New measurement of target

        n_iter
            Number of iterations to run
        """
        for _ in range(n_iter):
            self.predict()
            self.update(
                measure_state=measure_state,
                measure_cov=measure_cov)

    def predict(self) -> None:
        """
        Predict state in Kalman Filters
        """
        for _, filter in enumerate(self.filters):
            filter.predict()

    def update(
        self, 
        measure_state: np.array,
        measure_cov: np.array
    ) -> None:
        """
        Update state in Kalman Filters

        Parameters
        ----------
        measurement: np.ndarray
            New measurement of target
        """
        for i_filter, filter in enumerate(self.filters):
            filter.update(
                measure_state=measure_state,
                measure_cov=measure_cov)
            self.likelihood[i_filter] = filter.likelihood
        
        self._compute_mixing_probabilites()
        self._compute_state_estimate()

    def _compute_state_estimate(self) -> None:
        """
        Compute IMM's mixed state estimate
        """
        self.state.fill(0)
        for f, mu in zip(self.filters, self.MU):
            self.state += f.state * mu

        self.covariance.fill(0)
        for f, mu in zip(self.filters, self.MU):
            y = f.state - self.state
            self.covariance += mu * (np.outer(y, y) + f.covariance)


    def _compute_mixing_probabilities(self) -> None:
        """
        Compute Mixing Probability
        """
        cbar = np.dot(self.confidences, self.transition)
        for i in range(self.num_filters):
            for j in range(self.num_filters):
                self.omega[i, j] = self.markov_transition[i, j] * self.MU[i] / cbar[j]
    
