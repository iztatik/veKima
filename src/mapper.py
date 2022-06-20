import numpy as np
from numba import jit


# TC-joint
@jit(nopython=True)
def tc_joint(osc):
    tc_r = np.add(np.multiply(100, osc[0:3]), 90)
    tc_l = np.add(np.multiply(100, osc[3:6]), 90)
    return tc_r[0], tc_l[0], tc_r[1], tc_l[1], tc_r[2], tc_l[2]


# CTr-joint
@jit(nopython=True)
def ctr_joint(osc, offset):
    factor = 90-offset
    ctr_r = np.add(np.multiply(factor, osc[0:3]), 90-offset)
    ctr_l = np.add(np.multiply(factor, osc[3:6]), 90+offset)

    for i in range(len(osc)):
        if ctr_r[i] > 90-offset: ctr_r[i] = 90-offset
        if ctr_l[i] < 90+offset: ctr_l[i] = 90+offset

    return ctr_r[0], ctr_l[0], ctr_r[1], ctr_l[1], ctr_r[2], ctr_l[2]
