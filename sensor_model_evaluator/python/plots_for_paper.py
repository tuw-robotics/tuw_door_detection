import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource
import matplotlib.mlab as mlab


def decay(x_range, z_star):
    y_exp = np.exp(((-(x_range + 330) / (z_star)) * 2))
    y_exp[np.int(z_star):] = 0
    return y_exp


def lambda_decay(x_range, lambda_short, z_star, z_maxval=50):
    #return (1.0 / (1.0 - np.exp(-lambda_short * z_star))) * lambda_short * np.exp(-lambda_short * x_range)
    return (1.0 / (1.0 - np.exp(-lambda_short * z_star))) * lambda_short * np.exp(-lambda_short * x_range)

def uniform_noise(x_range, z_max_val, offset):
    y_uni = np.zeros(x_range.shape)
    y_uni[:] = (1.0 / z_max_val)
    y_uni[:offset] = 0
    y_uni[-offset:] = 0
    return y_uni


def z_max_fun(x_range, z_max_val, offset):
    y_uni = np.zeros(x_range.shape)
    y_uni[(z_max_val - offset):(z_max_val + offset)] = 0.025
    return y_uni


max_val = 50
z_max_val = 45
offset = 24
z_star = 200

x_range = np.arange(0, max_val, 1)
y_normpdf = mlab.normpdf(x_range, z_star, 0.5)

plt.subplot(2, 2, 1)
plt.plot(x_range, y_normpdf)
plt.xlabel("range")
plt.ylabel("p_hit")
plt.title("Gaussian")

plt.subplot(2, 2, 2)
plt.plot(x_range, lambda_decay(x_range, 0.05, z_star))
plt.xlabel("range")
plt.ylabel("p_short")
plt.title("Occlusions")

plt.subplot(2, 2, 3)
plt.plot(x_range, z_max_fun(x_range, z_max_val, offset))
plt.xlabel("range")
plt.ylabel("p_max")
plt.title("Max readings")

x_range_append = np.concatenate([np.arange(-20, 0, 1), x_range, np.arange(len(x_range) - 1, len(x_range) + 20, 1)],
                                axis=0)
plt.subplot(2, 2, 4)
plt.plot(x_range_append, uniform_noise(x_range_append, z_max_val, 20))
plt.xlabel("range")
plt.ylabel("p_rand")
plt.title("Uniform noise")

plt.show(block=True)
