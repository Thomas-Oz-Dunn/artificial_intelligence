
"""
Interacting Multi Model
"""

import numpy as np
import kalman_filter as kf

class InteractiveMultiModel:
    """
    A class for Interactive MultiModel (IMM)
    """
    def __init__(
        self,
        filters: kf.KalmanFilter,
        confidences,
        markov_transition
    ) -> None:
        self.filter = filters
        self.MU = confidences
        self.markov_transition = markov_transition

    def compute_state_estimate(self) -> None:
        for f, mu in zip(self.filters, self.MU):
            self.state += f.state * mu

        for f, mu in zip(self.filters, self.MU):
            y = f.state - self.state
            self.p += mu * (np.outer(y, y) + f.covariance)

    def compute_mixing_probabilities(self) -> None:
        N = len(self.confidences)
        omega = np.zeros([N, N])
        cbar = np.dot(self.confidences, self.transition)
        for i in N:
            for j in N:
                omega[i, j] = self.markov_transition[i, j] * self.MU[i] / cbar[j]

        return omega