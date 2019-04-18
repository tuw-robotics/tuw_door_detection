import time
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from matplotlib.colors import LightSource

LINE_ITEMS_NR = 2


# class Measurement:
#
#    def __init__(self, position_ws: np.array, angle_ws: np.double):
#        self.position =
#
#
# class ObsExpContainer:
#
#    def __init__(self):
#        self.obs = []
#        self.exp = []
#
#    def add(self, observation, expectation):
#        self.obs.append(observation)
#        self.exp.append(expectation)

class Result:
    def __init__(self, r, alpha):
        self.r = r
        self.alpha = alpha


class Results:
    def __init__(self):
        self.rs = np.array([], np.double)
        self.alphas = np.array([], np.double)

    def append(self, res: Result):
        np.append(self.rs, res.r)
        np.append(self.alphas, res.alpha)


def parse(file):
    lines = []
    cntr = 0
    for l in file.readlines():
        # if cntr >= max_lines:
        #    return lines
        lelems = l.split(", ")
        if len(lelems) != LINE_ITEMS_NR:
            print("WARNING: line {} inconsistent, 7 entries per line expected, therefore aborting parse operation " % (
                    len(lines) + 1))
            return lines
        # print('%s, %s, %s, %s, %s' % (lelems[0], lelems[R_EXP], lelems[R_OBS], lelems[P_EXP], lelems[P_OBS]))
        lines.append(lelems)
        cntr += 1
    return lines


def agglomerate(lines):
    '''
        Builds the internal datastructures from the parsed lines in the csv
    :param lines: the individual lines read from result.csv
    :return: datastructure containing the agglomerated result for further processing
    '''

    results = Results()
    for line in lines:
        res = Result(line[0], line[1])
        results.append(res)
    return results


def statsPlot(results: Results):
    pass
