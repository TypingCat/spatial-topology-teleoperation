#!/usr/bin/env python3

import math
import numpy as np

def quaternion_to_euler(x, y, z, w):
    t0 = 2.*(w*x + y*z)
    t1 = 1. - 2.*(x*x + y*y)
    roll = math.atan2(t0, t1)

    t2 = 2.*(w*y - z*x)
    t2 = 1. if t2 > 1. else t2
    t2 = -1. if t2 < -1. else t2
    pitch = math.asin(t2)

    t3 = 2.*(w*z + x*y)
    t4 = 1. - 2.*(y*y + z*z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return x, y, z, w

def get_ellipse_contour(center=[0, 0], axis=[[1, 0], [0, 1]], size=[1, 1], contour_num=100):
    # Draw ellipse on coordinate [[1, 0], [0, 1]]
    t = np.linspace(0, 2*np.pi, contour_num)
    xs = size[0] * np.cos(t)
    ys = size[1] * np.sin(t)

    # Rotate ellipse to fit in coordinate [axis[0], axis[1]]
    R = np.array(axis).T
    xp, yp = np.dot(R, [xs, ys])

    return xp + center[0], yp + center[1]