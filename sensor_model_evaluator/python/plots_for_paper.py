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


def pshort(x_range, lambda_short, z_star, stepsize):
    result = (1.0 / (1.0 - np.exp(-lambda_short * z_star))) * lambda_short * np.exp(-lambda_short * x_range)
    result[int((1.0 / stepsize) * z_star):] = 0
    return result


def prand(x_range, z_max_val, offset):
    y_uni = np.zeros(x_range.shape)
    y_uni[:] = (1.0 / z_max_val)
    y_uni[:offset] = 0
    y_uni[-offset:] = 0
    return y_uni


def pmax(x_range, stepsize, z_max, offset):
    y_uni = np.zeros(x_range.shape)
    px_max = int((1.0 / stepsize) * z_max)
    y_uni[(px_max - offset):(px_max + offset)] = 0.025
    return y_uni


max_val = 50
offset = 7
z_star = 25
lambda_short = 0.9

stepsize = 0.1
x_range = np.arange(0, max_val, stepsize)
y_normpdf = mlab.normpdf(x_range, z_star, 0.5)

fig, (axes0, axes1) = plt.subplots(1, 2, gridspec_kw = {'width_ratios':[1, 1], 'height_ratios':[0.20]})
axes0.plot(x_range, y_normpdf)
axes0.set_xlabel("range")
axes0.set_ylabel("p_hit")
axes0.set_title("Gaussian")

axes1.plot(x_range, pshort(x_range, 0.1, z_star, stepsize))
axes1.set_xlabel("range")
axes1.set_ylabel("p_short")
axes1.set_title("Occlusions")

plt.show(block=True)
plt.figure()

plt.subplot(1, 2, 1)
plt.plot(x_range, pmax(x_range, stepsize, max_val - 5, offset))
plt.axes[0, 0]
plt.xlabel("range")
plt.ylabel("p_max")
plt.title("Max readings")

x_range_append = np.concatenate([np.arange(-20, 0, 1), x_range, np.arange(len(x_range) - 1, len(x_range) + 20, 1)],
                                axis=0)
plt.subplot(1, 2, 2)
plt.plot(x_range_append, prand(x_range_append, stepsize, 20))
plt.xlabel("range")
plt.ylabel("p_rand")
plt.title("Uniform noise")

plt.show(block=True)
