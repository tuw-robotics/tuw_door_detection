import matplotlib.pyplot as plt
from scipy import special
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource
import matplotlib.mlab as mlab
from matplotlib.ticker import FormatStrFormatter


def gaussian_pdf(x, mu, sigma):
    return 1.0 / (np.sqrt(2 * np.pi) * sigma) * np.exp(-((x - mu) ** 2) / (2 * sigma * sigma))


mu = 0
sigma = 1.0
x = np.arange(-3.5, 3.5, 0.05)
label_font_size = 16
tick_size = label_font_size - 4
plt.rc('xtick', labelsize=tick_size)
plt.rc('ytick', labelsize=tick_size)

fig, ax = plt.subplots(1, 2)
ax0 = ax[0]
ax1 = ax[1]
ax0.plot(x, gaussian_pdf(x, mu, sigma))
ax0.set_xlabel("$x$", fontsize=label_font_size)
ax0.set_ylabel("$\mathcal{N}(x,0,1)$", fontsize=label_font_size)
ax0.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
ax0.fill_between(x, gaussian_pdf(x, mu, sigma), color=(0.5, 0.5, 0.5, 0.1))
ax1.plot(x, 0.5 * special.erfc(-x / np.sqrt(2)))
ax1.set_xlabel("$x$", fontsize=label_font_size)
ax1.set_ylabel("$\mathrm{erf}(x,0,1)$", fontsize=label_font_size)
ax1.set_yticks([0, 0.2, 0.4, 0.6, 0.8, 1.0])
for ax_elem in ax:
    ax_elem.yaxis.set_major_formatter(FormatStrFormatter('%.2f'))
ax0.grid(linestyle='dotted')
ax1.grid(linestyle='dotted')
ax0.axhline(y=0, color='k')
ax0.axvline(x=0, color='k')
ax1.axhline(y=0, color='k')
ax1.axvline(x=0, color='k')
plt.show(block=True)
