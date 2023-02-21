import numpy as np
import math


def quat_to_euler(q: np.array):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]
    heading = math.atan2(2 * (w * z + x * y), 1 - 2 * (pow(y, 2) + pow(z, 2)))
    elevation = math.asin(2 * (w * y - z * x))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (pow(x, 2) + pow(y, 2)))
    return (heading, elevation, roll)


def tum_to_xyzw(tum: np.array):
    xyzw = np.zeros([len(tum), 5])
    for i in range(len(tum)):
        her = quat_to_euler(tum[i, 4:8])
        heading = her[0]
        if (heading > math.pi):
            heading = heading - math.pi
        xyzw[i] = np.hstack((tum[i, 0:4], heading))
    return xyzw
