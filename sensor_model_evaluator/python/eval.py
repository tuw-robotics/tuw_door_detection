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


class AngularRepresentation:
    angle_id_ = 0
    position_expected_ = np.array([], dtype=np.double)
    position_observed_ = np.array([], dtype=np.double)
    range_expected_ = np.array([], dtype=np.double)
    range_observed_ = np.array([], dtype=np.double)

    def __init__(self, angle_id, range_expected, range_observed, position_expected, position_observed):
        self.angle_id_ = angle_id
        self.range_expected_ = np.concatenate([self.range_expected_, np.array([range_expected], dtype=np.double)])
        self.range_observed_ = np.concatenate([self.range_observed_, np.array([range_observed], dtype=np.double)])
        self.position_expected_ = np.array([[position_expected[0]], [position_expected[1]]], dtype=np.double)
        self.position_observed_ = np.array([[position_observed[0]], [position_observed[1]]], dtype=np.double)

    def add(self, range_expected, range_observed, position_expected, position_observed):
        self.range_expected_ = np.concatenate([self.range_expected_, np.array([range_expected], dtype=np.double)])
        self.range_observed_ = np.concatenate([self.range_observed_, np.array([range_observed], dtype=np.double)])
        self.position_expected_ = np.concatenate([self.position_expected_, position_expected.reshape(2, 1)], axis=0)
        self.position_observed_ = np.concatenate([self.position_observed_, position_observed.reshape(2, 1)], axis=0)

    def normalize_observations(self):
        '''
            Calculates the distance between observations and expected measurements
        :return:
        '''
        self.normalized_observations_ = self.range_expected_ - self.range_observed_


class StatisticsEvaluator:

    def __init__(self):
        pass


def parse(file, max_lines=100000000):
    lines = []
    cntr = 0
    for l in file.readlines():
        if cntr >= max_lines:
            return lines
        lelems = l.split(", ")
        if len(lelems) != 7:
            print("WARNING: line {} inconsistent, 7 entries per line expected, therefore aborting parse operation " % (
                    len(lines) + 1))
            return lines
        # print('%s, %s, %s, %s, %s' % (lelems[0], lelems[R_EXP], lelems[R_OBS], lelems[P_EXP], lelems[P_OBS]))
        lines.append(lelems)
        cntr += 1
    return lines


def agglomerate(lines):
    '''
        agglomerates for each angle the respective measurements to perform a statistic on how many measurements
        lie within the boundaries of the expected measurements etc...
    :param lines: the individual lines read from result.csv
    :return: datastructure containing the agglomerated result for further processing
    '''
    angle2measurements = {}
    for line in lines:
        angle_id = np.double(line[ID])
        range_exp = np.double(line[R_EXP])
        range_obs = np.double(line[R_OBS])
        position_exp = np.array([np.double(line[P_EXP]), np.double(line[P_EXP + 1])], dtype=np.double)
        position_obs = np.array([np.double(line[P_OBS]), np.double(line[P_OBS + 1])], dtype=np.double)

        val = angle2measurements.get(angle_id)
        if val is None:
            angle2measurements[angle_id] = AngularRepresentation(angle_id,
                                                                 range_exp,
                                                                 range_obs,
                                                                 position_exp,
                                                                 position_obs)
        else:
            val.add(range_exp,
                    range_obs,
                    position_exp,
                    position_obs)
    return angle2measurements


def eval(angle2meas):
    total_meas = len(angle2meas.items())
    print("eval: ", total_meas)

    cntr = 0
    for k, v in angle2meas.items():
        v.normalize_observations()
        print("", cntr, total_meas)
        cntr += 1

        # print("%d %lf %lf" % (key, table_range[key][0], table_range[key][1]))
    plot2d(angle2meas, -1, normalized=True)


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


def plot2d(angle2meas, idx=-1, normalized=False):
    plt.xlabel('angles')
    plt.ylabel('ranges')

    if not normalized:
        expected = np.array([], dtype=np.double)
        observed = np.array([], dtype=np.double)
        idxs = np.array([], dtype=np.double)
        if idx >= 0:
            for key, val in angle2meas.items():
                if (len(val.range_expected_) > idx):
                    expected.append(val.range_expected_[idx])
                    observed.append(val.range_observed_[idx])
                    idxs.append(val.angle_id_)
        elif idx == -1:
            for key, val in angle2meas.items():
                expected.append(val.range_expected_)
                observed.append(val.range_observed_)
                idxs.append(val.angle_id_)

        plt.plot(idxs, expected, 'or', label='expected', markersize=2)
        plt.plot(idxs, observed, 'ob', label='observed', markersize=2)
        plt.title("Measurements")
        plt.legend()
        plt.draw()
        plt.pause(0.1)
    else:
        normalized_vals = np.array([], dtype=np.double)
        idxs = np.array([], dtype=np.double)

        for key, val in angle2meas.items():
            idxs = np.concatenate(
                [idxs, np.array([key for _ in range(len(val.normalized_observations_))], dtype=np.double)])
            normalized_vals = np.concatenate([normalized_vals, val.normalized_observations_])
        print("plotting vals")
        plt.plot(idxs, normalized_vals, 'or', label='normalized obs', markersize=2)
        plt.title("Measurements")
        plt.legend()
        plt.draw()
        plt.pause(50)

plt.ion()
tstart = time.time()
print("tic > parse")
file = open(os.path.abspath("../files/result.csv"), "r")
lines = parse(file, 200000)
file.close()
print("parsed > agglomerate")
angle2meas = agglomerate(lines)
print("agglomerate > eval")
eval(angle2meas)
print("toc ")
print(time.time() - tstart)
## plot3d(lines)
# plt.clf()
# table_range.clear()
# time.sleep(0.5)
