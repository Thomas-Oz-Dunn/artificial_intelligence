"""
Autoencoder Kalman Filter Hybrid

By: Thomas Dunn
"""
import numpy as np
import matplotlib.pyplot as plt
import keras

import kalman_filter as kf
import multi_model as imm

# %% Autoencoder
class Denoise(keras.Model):
  def __init__(self):
    super(Denoise, self).__init__()
    self.encoder = keras.Sequential([
      keras.layers.Input(shape=(28, 28, 1)),
      keras.layers.Conv2D(
        256, 
        (3, 3), 
        activation='relu', 
        padding='same', 
        strides=2),
      keras.layers.Conv2D(
        64, 
        (3, 3), s
        activation='relu',
         padding='same', 
         strides=2)
         # Final layer condense into state and covariance
      ])

    self.decoder = keras.Sequential([
      # First layer operate on state and covariance
      keras.layers.Conv2DTranspose(
        64, 
        kernel_size=3, 
        strides=2, 
        activation='relu', 
        padding='same'),
      keras.layers.Conv2DTranspose(
        256, 
        kernel_size=3, 
        strides=2, 
        activation='relu', 
        padding='same'),
      keras.layers.Conv2D(
        1, 
        kernel_size=(3, 3), 
        activation='sigmoid', 
        padding='same')])

  def call(self, x):
    encoded = self.encoder(x)
    const_vel = kf.KalmanFilter()
    const_acc = kf.KalmanFilter()
    const_turn = kf.KalmanFilter()
    filters = [const_vel, const_acc, const_turn]

    multi_model = imm.InteractiveMultiModel(
      filters=filters,
      confidences=,
      markov_transition=)

    multi_model.run()

    decoded = self.decoder(multi_model.state, multi_model.covariance)
    return decoded

autoencoder = Denoise()