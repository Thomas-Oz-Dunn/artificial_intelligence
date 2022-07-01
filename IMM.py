
"""
Interacting Multi Model
"""

import numpy as np
import kalman_filter.KalmanFilter as KalmanFilter

def interact_multi_model(
    state, 
    covariance,
    time_step,
    omega,
    measurement
):
    # [x_pos, y_pos, x_vel, y_vel]
    cons_vel_mat = np.array([
        [1, 0, time_step, 0],
        [0, 1, 0, time_step],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    const_vel_KF = KalmanFilter(
        state=state,
        covariance=covariance,
        transition_matrix=cons_vel_mat)

    # [x_pos, y_pos, x_vel, y_vel, x_acc, y_acc]
    cons_acc_mat = np.array([
        [1, 0, time_step, 0, 0.5 * time_step**2, 0],
        [0, 1, 0, time_step, 0, 0.5 * time_step**2],
        [0, 0, 1, 0, time_step, 0],
        [0, 0, 0, 1, 0, time_step],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1]])
    const_acc_KF = KalmanFilter(
        state=state,
        covariance=covariance,
        transition_matrix=cons_acc_mat)
    
    # [x_pos, y_pos, x_vel, y_vel, omega]
    cons_turn_mat = np.array([
        [1, 0, np.sin(omega*time_step)/omega, -(1-np.cos(omega*time_step))/omega, 0],
        [0, 1, np.cos(omega*time_step), -np.sin(omega*time_step), 0],
        [0, 0, (1-np.cos(omega*time_step))/omega, np.sin(omega*time_step)/omega, 0],
        [0, 0, np.sin(omega*time_step), -np.cos(omega*time_step), 0],
        [0, 0, 0, 0, 1]])
    const_turn_KF = KalmanFilter(
        state=state,
        covariance=covariance,
        transition_matrix=cons_turn_mat)

    state_cv, cov_cv = const_vel_KF.run(
        new_measure=measurement,
        measure_cov=,
        measure_matrix=)
    state_ca, cov_ca = const_acc_KF.run(
        new_measure=measurement,
        measure_cov=,
        measure_matrix=)
    state_ct, cov_ct = const_turn_KF.run(
        new_measure=measurement,
        measure_cov=,
        measure_matrix=)
    
    # Markov Decision Process
    mixed_trans = 
    mixed_state =
    mixed_cov = 

    mixed_KF = KalmanFilter(
        state=mixed_state,
        covariance=mixed_cov,
        transition_matrix=mixed_trans)

    state, covariance = mixed_KF.run(
        new_measure=measurement,
        measure_matrix=,
        measure_cov=)

    # Model comparison based an accuracy to ground truth
    # Apply weight to prediction for decoder

    return (state, covariance)
