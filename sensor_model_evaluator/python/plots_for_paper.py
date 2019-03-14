import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource
import matplotlib.mlab as mlab


def decay(x_range, z_star):
    y_exp = np.exp(-x_range / z_star)
    y_exp[np.int(z_star):] = 0
    return y_exp

def uniform_noise(x_range, z_max_val):
    y_uni = np.zeros(x_range.shape)
    y_uni[:] = (1.0 / z_max_val)
    return y_uni

def z_max_fun(x_range, z_max_val, offset):
    y_uni = np.zeros(x_range.shape)
    y_uni[(z_max_val - offset):(z_max_val + offset)] = 0.2
    return y_uni

max_val = 300
z_max_val = 275
offset = 24
z_star = 200

x_range = np.arange(0, max_val, 0.25)
y_normpdf = mlab.normpdf(x_range, z_star, 2.5)

plt.subplot(2, 2, 1)
plt.plot(x_range, y_normpdf)
plt.xlabel("range")
plt.ylabel("p_hit")
plt.title("Gaussian")

plt.subplot(2, 2, 2)
plt.plot(x_range, decay(x_range, z_star))
plt.xlabel("range")
plt.ylabel("p_hit")
plt.title("Occlusions")

plt.subplot(2, 2, 3)
plt.plot(x_range, z_max_fun(x_range, z_max_val, offset))
plt.xlabel("range")
plt.ylabel("p_hit")
plt.title("Zmax")

plt.subplot(2, 2, 4)
plt.plot(x_range, uniform_noise(x_range, z_max_val))
plt.xlabel("range")
plt.ylabel("p_hit")
plt.title("Uniform noise")

plt.show(block=True)
