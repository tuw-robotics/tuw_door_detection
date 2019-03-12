import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource

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
        # if hasattr(self, 'range_exp_cutoff'):
        #    if range_expected > self.range_exp_cutoff:
        #        return
        # if hasattr(self, 'range_obs_cutoff'):
        #    if range_observed > self.range_obs_cutoff:
        #        return
        self.range_expected_ = np.concatenate([self.range_expected_, np.array([range_expected], dtype=np.double)])
        self.position_expected_ = np.concatenate([self.position_expected_, position_expected.reshape(2, 1)], axis=0)
        self.range_observed_ = np.concatenate([self.range_observed_, np.array([range_observed], dtype=np.double)])
        self.position_observed_ = np.concatenate([self.position_observed_, position_observed.reshape(2, 1)], axis=0)

    def set_cutoff(self, range_exp_cutoff=np.finfo(np.double).max, range_obs_cutoff=np.finfo(np.double).max):
        self.range_exp_cutoff = min(range_exp_cutoff, range_obs_cutoff)
        self.range_obs_cutoff = min(range_exp_cutoff, range_obs_cutoff)

    def normalize_observations(self):
        '''
            Calculates the distance between observations and expected measurements
        :return:
        '''
        if not hasattr(self, 'normalized_observations'):
            self.normalized_observations_ = self.range_expected_ - self.range_observed_

    def getIdxArray(self):
        if not hasattr(self, 'idx_array_'):
            self.idx_array_ = np.array([self.angle_id_ for _ in range(self.range_observed_)], dtype=np.double)
        return self.idx_array_

    def getMaxExp(self):
        if not hasattr(self, 'maxval_exp'):
            self.maxval_exp = np.max(self.range_expected_)
        return self.maxval_exp

    def getMaxObs(self):
        if not hasattr(self, 'maxval_obs'):
            self.maxval_obs = np.max(self.range_observed_)
        return self.maxval_obs

    def getMinExp(self):
        if not hasattr(self, 'minval_exp'):
            self.minval_exp = np.min(self.range_expected_)
        return self.minval_exp

    def getMinObs(self):
        if not hasattr(self, 'minval_obs'):
            self.minval_obs = np.min(self.range_observed_)
        return self.minval_obs


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


def lookup_table(table, x, y):
    v = table.get((x, y))
    if v is None:
        return 0
    return 1.0


def hist3d_thrun_dbg(angle2meas, binsize_xy):
    fig = plt.figure()
    idxs = np.array([], dtype=np.double)
    max_exp_val = np.finfo(np.double).min
    min_exp_val = np.finfo(np.double).max

    for key, val in angle2meas.items():
        max_exp_val = max(val.getMaxExp(), max_exp_val)
        min_exp_val = min(val.getMinExp(), min_exp_val)

    binrange_y = max_exp_val - min_exp_val

    binshape_xy = (np.int(np.ceil(binrange_y / binsize_xy[0])), np.int(np.ceil(binrange_y / binsize_xy[1])))
    grid = np.zeros(shape=binshape_xy, dtype=np.double)
    X, Y = np.meshgrid(np.arange(0, binshape_xy[0], 1), np.arange(0, binshape_xy[1], 1))

    print(max_exp_val)
    print(min_exp_val)
    print(binshape_xy)

    for key, val in angle2meas.items():
        y_exp_idx = ((val.range_expected_ - min_exp_val) / binsize_xy[1]).astype(np.int)
        grid[y_exp_idx, y_exp_idx] += 1

    ax = fig.add_subplot(111, projection='3d')

    # Construct arrays for the anchor positions of the 16 bars.
    # Note: np.meshgrid gives arrays in (ny, nx) so we use 'F' to flatten xpos,
    # ypos in column-major order. For numpy >= 1.7, we could instead call meshgrid
    # with indexing='ij'.

    # Construct arrays with the dimensions for the 16 bars.
    # ls = LightSource(270, 45)
    # rgb = ls.shade(grid, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
    ax.plot_surface(X, Y, grid, rstride=1, cstride=1,  # facecolors=rgb,
                    linewidth=0, antialiased=False, shade=False)
    plt.show(block=True)


def scatter_plot(angle2meas):
    fig = plt.figure()
    scatter_xrange = np.array([], dtype=np.double)
    scatter_yrange = np.array([], dtype=np.double)
    for key, val in angle2meas.items():
        scatter_xrange = np.concatenate([scatter_xrange, val.range_observed_])
        scatter_yrange = np.concatenate([scatter_yrange, val.range_expected_])

    axsc = fig.add_subplot(111)
    axsc.scatter(scatter_xrange, scatter_yrange, marker='+')


def hist3d_thrun(angle2meas, binsize_xy, normalize_total=False, normalize_rows=True, medianize=True):
    fig = plt.figure()
    idxs = np.array([], dtype=np.double)
    max_obs_val = np.finfo(np.double).min
    min_obs_val = np.finfo(np.double).max
    max_exp_val = np.finfo(np.double).min
    min_exp_val = np.finfo(np.double).max

    for key, val in angle2meas.items():
        max_obs_val = max(val.getMaxObs(), max_obs_val)
        min_obs_val = min(val.getMinObs(), min_obs_val)
        max_exp_val = max(val.getMaxExp(), max_exp_val)
        min_exp_val = min(val.getMinExp(), min_exp_val)

    binrange_x = max_obs_val - min_obs_val
    binrange_y = max_exp_val - min_exp_val

    binshape_xy = (np.int(np.ceil(binrange_x / binsize_xy[0])), np.int(np.ceil(binrange_y / binsize_xy[1])))
    binshape_xy = (max(binshape_xy[0], binshape_xy[1]), max(binshape_xy[0], binshape_xy[1]))
    grid = np.zeros(shape=binshape_xy, dtype=np.double)
    X, Y = np.meshgrid(np.arange(0, binshape_xy[0], 1), np.arange(0, binshape_xy[1], 1))
    X = X * binsize_xy[0]
    Y = Y * binsize_xy[1]

    print(binsize_xy)
    print(grid.shape)

    print(max_obs_val)
    print(min_obs_val)
    print(max_exp_val)
    print(min_exp_val)
    print(binshape_xy)

    total_measurements = 0
    for key, val in angle2meas.items():
        x_obs_idx = ((val.range_observed_ - min_obs_val) / binsize_xy[0]).astype(np.int)
        y_exp_idx = ((val.range_expected_ - min_exp_val) / binsize_xy[1]).astype(np.int)
        grid[x_obs_idx, y_exp_idx] += 1
        total_measurements += len(x_obs_idx)

    if normalize_total:
        grid = grid / total_measurements
    elif normalize_rows:
        rowsum = np.sum(grid, axis=0)
        for c in range(grid.shape[1]):
            grid[:, c] = grid[:, c] / rowsum[c]

    if medianize:
        for c in range(grid.shape[1]):
            tmp_row = np.ndarray(shape=(grid.shape[0] - 2,), dtype=np.double)
            for r in range(1, grid.shape[0] - 1):
                tmp_row[r - 1] = np.median(grid[(r - 1):(r + 1), c])
            grid[0, c] = np.median([grid[0, c], grid[1, c]])
            grid[grid.shape[0] - 1, c] = np.median([grid[grid.shape[0] - 2, c], grid[grid.shape[0] - 1, c]])
            grid[1:(grid.shape[0] - 1), c] = tmp_row

    ax = fig.add_subplot(111, projection='3d')
    ## Construct arrays for the anchor positions of the 16 bars.
    ## Note: np.meshgrid gives arrays in (ny, nx) so we use 'F' to flatten xpos,
    ## ypos in column-major order. For numpy >= 1.7, we could instead call meshgrid
    ## with indexing='ij'.

    ## Construct arrays with the dimensions for the 16 bars.
    # ls = LightSource(270, 45)
    # rgb = ls.shade(grid, cmap=cm.gist_earth, vert_exag=0.1, blend_mode='soft')
    # ax.plot_surface(X, Y, grid, rstride=1, cstride=1, facecolors=rgb,
    #                linewidth=0, antialiased=False, shade=True)
    ax.plot_surface(X, Y, grid, cmap=cm.coolwarm, antialiased=False)
    ax.set_xlabel('Observed')
    ax.set_ylabel('Expected')
    ax.set_zlabel('Probabilities')
    plt.show(block=True)


def hist3d_plot(angle2meas):
    fig = plt.figure()
    idxs = np.array([], dtype=np.double)
    normalized_vals = np.array([], dtype=np.double)

    for key, val in angle2meas.items():
        idxs = np.concatenate(
            [idxs, np.array([key for _ in range(len(val.normalized_observations_))], dtype=np.double)])
        normalized_vals = np.concatenate([normalized_vals, val.normalized_observations_])

    ax = fig.add_subplot(111, projection='3d')
    hist, xedges, yedges = np.histogram2d(idxs, normalized_vals, bins=[180, 10])

    # Construct arrays for the anchor positions of the 16 bars.
    # Note: np.meshgrid gives arrays in (ny, nx) so we use 'F' to flatten xpos,
    # ypos in column-major order. For numpy >= 1.7, we could instead call meshgrid
    # with indexing='ij'.
    xpos, ypos = np.meshgrid(xedges[:-1], yedges[:-1])
    xpos = xpos.flatten('F')
    ypos = ypos.flatten('F')
    zpos = np.zeros_like(xpos)

    # Construct arrays with the dimensions for the 16 bars.
    dx = 0.5 * np.ones_like(zpos)
    dy = dx.copy()
    dz = hist.flatten()

    ax.bar3d(xpos, ypos, zpos, dx, dy, dz, color='b', zsort='average')
    plt.show(block=True)


def plot3d_deprecated(file_contents):
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
file = open(os.path.abspath("../files/result_open_doors.csv"), "r")
lines = parse(file)
file.close()
print("parsed > agglomerate")
angle2meas = agglomerate(lines)
print("agglomerate > eval")
eval(angle2meas)
hist3d_thrun(angle2meas, (0.5, 0.5), normalize_total=False, normalize_rows=True, medianize=True)
# hist3d_thrun_dbg(angle2meas, (0.50, 0.50))
# plot2d(angle2meas, -1, normalized=True)
# hist3d_plot(angle2meas)
print("toc ")
print(time.time() - tstart)
## plot3d(lines)
# plt.clf()
# table_range.clear()
# time.sleep(0.5)
