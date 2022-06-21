"""
Useful functions for Kalman Filter
"""
import numpy as np

THRESHOLD = 1e-6

def interact_multi_model(
    mean_state, 
    mean_cov,
    time_step,
    omega
):
    cons_vel_mat = np.array([
        [1, 0, time_step, 0],
        [0, 1, 0, time_step],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    const_vel_KF = KalmanFilter(
        mean_state=mean_state,
        mean_cov=mean_cov,
        transition_matrix=cons_vel_mat)

    cons_acc_mat = np.array([
        [1, 0, time_step, 0, 0.5 * time_step**2, 0],
        [0, 1, 0, time_step, 0, 0.5 * time_step**2],
        [0, 0, 1, 0, time_step, 0],
        [0, 0, 0, 1, 0, time_step],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]])
    const_acc_KF = KalmanFilter(
        mean_state=mean_state,
        mean_cov=mean_cov,
        transition_matrix=cons_acc_mat)
    
    cons_turn_mat = np.array([
        [1, 0, np.sin(omega*time_step)/omega, -(1-np.cos(omega*time_step))/omega, 0],
        [0, 1, np.cos(omega*time_step), -np.sin(omega*time_step), 0],
        [0, 0, (1-np.cos(omega*time_step))/omega, np.sin(omega*time_step)/omega, 0],
        [0, 0, np.sin(omega*time_step), -np.cos(omega*time_step), 0],
        [0, 0, 0, 0, 1]])
    const_turn_KF = KalmanFilter(
        mean_state=mean_state,
        mean_cov=mean_cov,
        transition_matrix=cons_turn_mat)
    
    return (mean_state, mean_cov)

class KalmanFilter:
    """
    Kalman Filter class
    """
    def __init__(
        self, 
        mean_state, 
        mean_cov, 
        transition_matrix
    ):
        self.mean_state = mean_state
        self.mean_cov = mean_cov
        self.transition_matrix = transition_matrix
        self.prcx_noise_cov = np.eye(mean_state.shape() [0])
        self.input_effect = np.eye(mean_state.shape() [0])
        self.control_input = np.zeros((mean_state.shape() [0], 1))

    def run(
        self, 
        new_measure,
        measure_matrix,
        measure_cov,
        n_iter: int = 150
    ):
        for i_iter in np.arange(0, n_iter):
            self.predict()
            self.update(new_measure, measure_matrix, measure_cov)

    def predict(self) -> None:
        self.mean_state = predict_state(
            mean_state=self.mean_state,
            transition_matrix=self.transition_matrix,
            control_effect=self.control_effect,
            control_input=self.control_input)

        self.mean_cov = predict_covariance(
            mean_covariance=self.mean_cov,
            transition_matrix=self.transition_matrix,
            prcx_noise_covariance=self.prcx_noise_cov)

    def update(
        self,
        new_measure, 
        measure_matrix, 
        measure_cov
    ) -> None:
        pred_mean = np.dot(measure_matrix, self.mean_state)
        predict_cov = measure_cov + np.dot(measure_matrix, np.dot(self.mean_cov, measure_matrix.T))
        
        self.kalman_gain = np.dot(self.mean_cov, np.dot(measure_matrix.T, np.linalg.inv(predict_cov)))
        diff = (new_measure - pred_mean)
        self.mean_state = self.mean_state + np.dot(self.kalman_gain, diff)
        self.mean_cov = self.mean_cov - np.dot(self.kalman_gain, np.dot(predict_cov, self.kalman_gain.T))

def predict_covariance(
    mean_covariance,
    transition_matrix,
    prcx_noise_covariance
):
    transition_cov = np.dot(mean_covariance, transition_matrix.T)
    return np.dot(transition_matrix, transition_cov) + prcx_noise_covariance


def predict_state(
    mean_state,
    transition_matrix,
    control_effect,
    control_input
):
    transition = np.dot(transition_matrix, mean_state)
    controls = np.dot(control_effect, control_input)
    return transition + controls

