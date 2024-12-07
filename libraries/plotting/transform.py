# the resulting plot is landscape mode, not portrait
# so we need to invert xy and add a minus sign
# also, convert to numpy array to facilitate operations (addition, scalar multiplication, etc.)

import numpy as np


def transform(pose):
    result = np.array([pose.x, pose.y, pose.rz])
    result[0] = pose.y
    result[1] = -pose.x
    return result
