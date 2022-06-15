"""
Useful functions for Kalman Filter
"""
import numpy as np

def interact_multi_model(
    input_state,
    input_covariance
):
    # Predict and update loop
    while err > THRESHOLD
        predict()
        update()
    return (output_state, output_covariance)

def predict(
    prev_state, 
    prev_cov, 
    transition, 
    prcx_noise_cov, 
    input_effect, 
    control_input
):
  new_state = np.dot(transition, prev_state) + np.dot(input_effect, control_input)
  new_cov = np.dot(transition, np.dot(prev_cov, transition.T)) + prcx_noise_cov
  return (new_state, new_cov)

def update(
    prev_state, 
    prev_cov, 
    new_meas, 
    meas_matr, 
    meas_cov
):
  IM = np.dot(meas_matr, prev_state)
  IS = meas_cov + np.dot(meas_matr, np.dot(prev_cov, meas_matr.T))
  K = np.dot(prev_cov, np.dot(meas_matr.T, np.linalg.inv(IS)))
  X = prev_state + np.dot(K, (new_meas - IM))
  P = prev_cov - np.dot(K, np.dot(IS, K.T))
  LH = gauss_pdf(new_meas, IM, IS)
  return (X, P, K, IM, IS, LH)

def gauss_pdf(
    X, 
    mean, 
    covariance
):
  if mean.shape()[1] == 1:
    DX = X - np.tile(mean, X.shape()[1])
    E = 0.5 * sum(DX * (np.dot(np.linalg.inv(covariance), DX)), axis=0)
    E = E + 0.5 * mean.shape()[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(S))
    P = np.exp(-E)
  elif X.shape()[1] == 1:
    DX = np.tile(X, mean.shape()[1]) - mean
    E = 0.5 * sum(DX * (np.dot(np.linalg.inv(covariance), DX)), axis=0)
    E = E + 0.5 * mean.shape()[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(covariance))
    P = np.exp(-E)
  else:
    DX = X - mean
    E = 0.5 * np.dot(DX.T, np.dot(np.linalg.inv(covariance), DX))
    E = E + 0.5 * mean.shape()[0] * np.log(2 * np.pi) + 0.5 * np.log(np.linalg.det(covariance))
    P = np.exp(-E)
  return (P[0], E[0])
