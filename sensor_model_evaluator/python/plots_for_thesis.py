import matplotlib.pyplot as plt
from scipy import special
from scipy.stats import norm
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource
import matplotlib.mlab as mlab
from matplotlib.ticker import FormatStrFormatter


def gaussian_pdf(x, mu, sigma):
    val = 1.0 / (np.sqrt(2 * np.pi) * sigma) * np.exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma))
    print('{},{},{}'.format(mu, sigma, val))
    return val


sigma_list = [0.5, 1.0, 2.0]
mu = 0
x = np.arange(-5, 5, 0.02)
label_font_size = 24
tick_size = label_font_size - 4
plt.rc('xtick', labelsize=tick_size)
plt.rc('ytick', labelsize=tick_size)
area_color = (0.5, 0.5, 0.5, 0.1)

# Plots in total
fig, ax = plt.subplots(1, 2)
ax0 = ax[0]
ax1 = ax[1]

for sigma in sigma_list:
    y_gauss = norm.pdf(x, mu, sigma)
    ax0.plot(x, y_gauss)
    ax0.fill_between(x, y_gauss, color=area_color)

ax0.set_xlabel("$x$", fontsize=label_font_size)
ax0.set_ylabel("$\mathcal{N}(x|\;\mu,\sigma)$", fontsize=label_font_size)
ax0.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])

for sigma in sigma_list:
    y_cdf = 0.5 * special.erfc((mu - x) / (np.sqrt(2) * sigma))
    ax1.plot(x, y_cdf)
    ax1.fill_between(x, y_cdf, color=area_color)

ax1.set_xlabel("$x$", fontsize=label_font_size)
ax1.set_ylabel("$\Phi(x|\;\mu,\sigma)$", fontsize=label_font_size)
ax1.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])

for ax_elem in ax:
    ax_elem.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))

legend_desc = ['$\mu={}, \sigma={}$'.format(mu, sigma) for sigma in sigma_list]
ax0.legend(legend_desc, fontsize=tick_size, loc='upper left')
ax1.legend(legend_desc, fontsize=tick_size)

ax0.grid(linestyle='dotted')
ax1.grid(linestyle='dotted')
ax0.axhline(y=0, color='k')
ax0.axvline(x=0, color='k')
ax1.axhline(y=0, color='k')
ax1.axvline(x=0, color='k')

plt.show(block=True)
