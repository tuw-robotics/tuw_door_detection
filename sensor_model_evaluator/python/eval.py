import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

ID = 0
R_EXP = 1
R_OBS = 2
P_EXP = 3
P_OBS = 5

table_range = {}


def read(file):
    lines = []
    for l in file.readlines():
        lelems = l.split(", ")
        assert len(lelems) == 7, "inconsistent file, 7 entries per line expected"
        # print('%s, %s, %s, %s, %s' % (lelems[0], lelems[R_EXP], lelems[R_OBS], lelems[P_EXP], lelems[P_OBS]))
        lines.append(lelems)
    return lines


def eval(file_contents):
    for line in file_contents:
        key = np.int(line[ID])
        table_range[np.int(line[ID])] = (np.double(line[R_EXP]), np.double(line[R_OBS]))
        # print("%d %lf %lf" % (key, table_range[key][0], table_range[key][1]))
    plot2d(table_range)


def plot3d(table_range):
    xrange = np.arange(1, len(table_range), 1)
    yrange = np.arange(1, len(table_range), 1)
    for key, val in table_range.items():
        pass


def plot2d(table_range):
    plt.xlabel('id')
    plt.ylabel('ranges')

    expected = []
    observed = []
    idxs = np.arange(0, len(table_range), 1)
    for key, val in table_range.items():
        expected.append(val[0])
        observed.append(val[1])

    assert len(expected) == len(observed) == len(idxs), "HEAST!"

    plt.plot(idxs, expected, 'or', label='expected', markersize=2)
    plt.plot(idxs, observed, 'ob', label='observed', markersize=2)
    plt.title("Measurements")
    plt.legend()
    plt.draw()
    plt.pause(0.1)


plt.ion()
jj = 0
while (True):
    file = open(os.path.abspath("../files/result.csv"), "r")
    lines = read(file)
    file.close()
    eval(lines)
    plt.clf()
    table_range.clear()
    time.sleep(0.5)
