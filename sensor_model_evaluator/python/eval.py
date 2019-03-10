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
        key = np.double(line[ID])
        table_range[np.double(line[ID])] = (np.double(line[R_EXP]), np.double(line[R_OBS]))
        # print("%d %lf %lf" % (key, table_range[key][0], table_range[key][1]))
    plot2d(table_range)


def lookup_table(table, x, y):
    v = table.get((x, y))
    if v is None:
        return 0
    return 1.0


def plot3d(file_contents):
    x_expected = np.array([np.double(x[P_EXP]) for x in file_contents])
    y_expected = np.array([np.double(y[P_EXP + 1]) for y in file_contents])
    expected_vals = np.array([np.double(v[R_EXP]) for v in file_contents])

    table_expected = {(x, y): val for x, y, val in zip(x_expected, y_expected, expected_vals)}

    x_observed = np.array([np.double(x[P_OBS]) for x in file_contents])
    y_observed = np.array([np.double(y[P_OBS + 1]) for y in file_contents])
    observed_vals = np.array([np.double(v[R_OBS]) for v in file_contents])

    table_observed = {(x, y): val for x, y, val in zip(x_observed, y_observed, observed_vals)}

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    X, Y = np.meshgrid(x_expected, y_expected)
    zs = np.array([lookup_table(table_expected, x, y) for x, y in zip(np.ravel(X), np.ravel(Y))])
    Z = zs.reshape(X.shape)

    ax.plot_surface(X, Y, Z)

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show(block=True)


def plot2d(table_range):
    plt.xlabel('angles')
    plt.ylabel('ranges')

    expected = [None for x in range(0, len(table_range))]
    observed = [None for x in range(0, len(table_range))]
    idxs = [None for x in range(0, len(table_range))]
    ridx = 0
    for key, val in table_range.items():
        expected[ridx] = val[0]
        observed[ridx] = val[1]
        idxs[ridx] = key
        ridx += 1

    assert len(expected) == len(observed) == len(idxs), "HEAST!"

    plt.plot(idxs, expected, 'or', label='expected', markersize=2)
    plt.plot(idxs, observed, 'ob', label='observed', markersize=2)
    plt.title("Measurements")
    plt.legend()
    plt.draw()
    plt.pause(0.1)


plt.ion()
while (True):
    file = open(os.path.abspath("../files/result.csv"), "r")
    lines = read(file)
    file.close()
    eval(lines)
    # plot3d(lines)
    plt.clf()
    table_range.clear()
    time.sleep(0.5)
