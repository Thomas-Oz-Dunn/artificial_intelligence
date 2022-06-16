"""
Autoencoder Kalman Filter Hybrid

By: Thomas Dunn
"""
import numpy as np
import matplotlib.pyplot as plt
import keras

import kalman_filter as kf

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
        (3, 3), 
        activation='relu',
         padding='same', 
         strides=2)])

    self.decoder = keras.Sequential([
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
    # Insert IMM here
    # Input R, process covariance, and x, the measurement
    kf.interact_multi_model(encoded)
    decoded = self.decoder(encoded)
    return decoded

autoencoder = Denoise()