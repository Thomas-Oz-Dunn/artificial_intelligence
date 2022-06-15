"""
Autoencoder Kalman Filter Hybrid
"""
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
from tensorflow.keras.models import Model

def kf_predict(
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

def kf_update(
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

  latent_dim = 64

def imm(
    input_state,
    input_cov
):
    # Constant Velocity
    dim = 6
    # Constant Acceleration
    dim = 9
    # Constant Turn
    dim = 9

    # Compare Models based on fit
    transition = np.zeros((dim, dim))
    predict_state, predict_cov = kf_predict(
        prev_state=input_state,
        prev_cov=input_cov,
        transition=transition,
        prcx_noise_cov=,
        input_effect=0,
        control_input=0
    )
    update = kf_update(
        prev_state=predict_state,
        prev_cov=predict_cov,

    )
    (X, P, K, IM, IS, LH) = update


class Denoise(Model):
  def __init__(self):
    super(Denoise, self).__init__()
    self.encoder = tf.keras.Sequential([
      layers.Input(shape=(28, 28, 1)),
      layers.Conv2D(16, (3, 3), activation='relu', padding='same', strides=2),
      layers.Conv2D(8, (3, 3), activation='relu', padding='same', strides=2)])

    self.decoder = tf.keras.Sequential([
      layers.Conv2DTranspose(8, kernel_size=3, strides=2, activation='relu', padding='same'),
      layers.Conv2DTranspose(16, kernel_size=3, strides=2, activation='relu', padding='same'),
      layers.Conv2D(1, kernel_size=(3, 3), activation='sigmoid', padding='same')])

  def call(self, x):
    encoded = self.encoder(x)
    # Insert IMM here
    decoded = self.decoder(encoded)
    return decoded

autoencoder = Denoise()