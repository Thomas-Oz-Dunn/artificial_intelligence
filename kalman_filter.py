"""
Useful functions for Kalman Filter
"""
import numpy as np

THRESHOLD = 1e-6

def interact_multi_model(
    mean_state, 
    mean_cov
):
    return (mean_state, mean_cov)

class KalmanFilter:
    """
    Kalman Filter class
    """
    def __init__(self, mean_state, mean_cov):
        self.mean_state = mean_state
        self.mean_cov = mean_cov
        self.transition_matrix = 
        self.prcx_noise_cov = np.eye(mean_state.shape() [0])
        self.input_effect = np.eye(mean_state.shape() [0])
        self.control_input = np.zeros((mean_state.shape() [0], 1))
        self.measure_matrix = 
        self.measure_cov =        

    def run(
        self, 
        new_measure,
        n_iter: int = 150):
        
        for i_iter in np.arange(0, n_iter):
            
            (mean_state, mean_cov) = predict(
                self.mean_state, 
                self.mean_cov, 
                self.transition_matrix, 
                self.prcx_noise_cov, 
                self.input_effect, 
                self.control_input)
            (mean_state, mean_cov) = update(
                self.mean_state, 
                self.mean_cov, 
                new_measure, 
                self.measure_matrix, 
                self.measure_cov)
        
        return (mean_state, mean_cov)

def predict(
    mean_state, 
    mean_cov, 
    transition_matrix, 
    prcx_noise_cov, 
    control_effect, 
    control_input
):
    predicted_state = predict_state(
        mean_state=mean_state,
        transition_matrix=transition_matrix,
        control_effect=control_effect,
        control_input=control_input)

    predicted_covariance = predict_covariance(
        mean_covariance=mean_cov,
        transition_matrix=transition_matrix,
        prcx_noise_covariance=prcx_noise_cov)

    return (predicted_state, predicted_covariance)


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


def update(
    mean_state, 
    mean_cov, 
    new_measure, 
    measure_matrix, 
    measure_cov
):
    pred_mean = np.dot(measure_matrix, mean_state)
    predict_cov = measure_cov + np.dot(measure_matrix, np.dot(mean_cov, measure_matrix.T))
    kalman_gain = np.dot(mean_cov, np.dot(measure_matrix.T, np.linalg.inv(predict_cov)))
    diff = (new_measure - pred_mean)

    mean_state = mean_state + np.dot(kalman_gain, diff)
    mean_cov = mean_cov - np.dot(kalman_gain, np.dot(predict_cov, kalman_gain.T))

    return (mean_state, mean_cov)
