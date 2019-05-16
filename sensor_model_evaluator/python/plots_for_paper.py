import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource
import matplotlib.mlab as mlab
from matplotlib.ticker import FormatStrFormatter


def decay(x_range, z_star):
    y_exp = np.exp(((-(x_range + 330) / (z_star)) * 2))
    y_exp[np.int(z_star):] = 0
    return y_exp


def pshort(x_range, lambda_short, z_star, stepsize):
    result = (1.0 / (1.0 - np.exp(-lambda_short * z_star))) * lambda_short * np.exp(-lambda_short * x_range)
    result[int((1.0 / stepsize) * z_star):] = 0
    return result


def pmax(x_range, stepsize, z_max, offset):
    y_uni = np.zeros(x_range.shape)
    px_max = int((1.0 / stepsize) * z_max)
    y_uni[(px_max - offset):(px_max + offset)] = 0.75
    return y_uni


def prand(x_range, stepsize, z_max, offset):
    y_uni = np.zeros(x_range.shape)
    y_uni[:] = (1.0 / z_max)
    offset_len = int((1.0 / stepsize) * offset)
    y_uni[:offset_len] = 0
    y_uni[-offset_len:] = 0
    return y_uni


max_val = 50
rand_prob = (1.0 / max_val)
offset = 7
z_star = 25
lambda_short = 0.9
axes_xticks = [
               [0, 25, 45],
               [0, 25, 45],
               [0, 25, 45],
               [0, 25, 45],
              ]

axes_yticks = [
               [0, 0.5, 1.0],
               [0, 0.06, 0.12],
               [0, 0.37, 0.75],
               [0, 1.5*rand_prob, 3.0 * rand_prob]
              ]

label_size=16
stepsize = 0.1
x_range = np.arange(0, max_val, stepsize)
y_normpdf = mlab.normpdf(x_range, z_star, 0.75)

fig, axes = plt.subplots(2, 2)
axes0 = axes[0, 0]
axes1 = axes[0, 1]
axes2 = axes[1, 0]
axes3 = axes[1, 1]

axes0.plot(x_range, y_normpdf)
#axes0.set_xlabel("range")
#axes0.set_ylabel("p_hit")
axes0.set_xticks(axes_xticks[0])
axes0.set_yticks(axes_yticks[0])
axes0.set_title("$p_{hit}$", fontsize=label_size+4)

axes1.plot(x_range, pshort(x_range, 0.1, z_star, stepsize))
#axes1.set_xlabel("range")
#axes1.set_ylabel("p_short")
axes1.set_xticks(axes_xticks[1])
axes1.set_yticks(axes_yticks[1])
axes1.set_title("$p_{short}$", fontsize=label_size+4)

axes2.plot(x_range, pmax(x_range, stepsize, max_val - 5, offset))
#axes2.set_xlabel("range")
#axes2.set_ylabel("p_max")
axes2.set_xticks(axes_xticks[2])
axes2.set_yticks(axes_yticks[2])
axes2.set_title("$p_{max}$", fontsize=label_size+4)

x_range_append = np.concatenate([np.arange(-7, 0, stepsize), x_range, np.arange(max_val, max_val + 7, stepsize)],
                                axis=0)
print(len(x_range_append))
print(len(x_range))
axes3.plot(x_range_append, prand(x_range_append, stepsize, max_val - 5, 5))
#axes3.set_xlabel("range")
#axes3.set_ylabel("p_rand")
axes3.set_xticks(axes_xticks[3])
axes3.set_yticks(axes_yticks[3])
axes3.set_title("$p_{rand}$", fontsize=label_size+4)

for ax_row in axes:
    for ax in ax_row:
        ax.tick_params(axis='both', which='major', labelsize=label_size)
        ax.xaxis.set_major_formatter(FormatStrFormatter('%dm'))
        ax.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

plt.show(block=True)
plt.figure()
