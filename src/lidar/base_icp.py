"""
Iterative Closest Point (ICP) SLAM example
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı, Shamil Gemuev
"""

import math

from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
import numpy as np

#  ICP parameters
EPS = 0.0001
MAX_ITER = 40

show_animation = False


def icp_base(previous_points, current_points):
    """
    Iterative Closest Point matching
    - input
    previous_points: 2D or 3D points in the previous frame
    current_points: 2D or 3D points in the current frame
    - output
    R: Rotation matrix
    T: Translation vector
    """
    H = None  # homogeneous transformation matrix

    dError = np.inf
    preError = np.inf
    count = 0

    if show_animation:
        fig = plt.figure()
        if previous_points.shape[0] == 3:
           fig.add_subplot(111, projection='3d')

    while dError >= EPS:
        count += 1

        if show_animation:  # pragma: no cover
            plot_points(previous_points, current_points, fig)
            plt.pause(0.1)

        indexes, error = nearest_neighbor_association(previous_points, current_points)
        Rt, Tt = svd_motion_estimation(previous_points[:, indexes], current_points)
        # update current points
        current_points = (Rt @ current_points) + Tt[:, np.newaxis]

        dError = abs(preError - error)
        print("Residual:", error)

        if dError < 0:  # prevent matrix H changing, exit loop
            print("Not Converge...", preError, dError, count)
            return None, None, None

        preError = error
        H = update_homogeneous_matrix(H, Rt, Tt)

        if dError <= EPS:
            print("Converge", error, dError, count)
            break
        elif MAX_ITER <= count:
            print("Not Converge...", error, dError, count)
            return None, None, None

    R = np.array(H[0:-1, 0:-1])
    T = np.array(H[0:-1, -1])

    return R, T, error


def update_homogeneous_matrix(Hin, R, T):

    r_size = R.shape[0]
    H = np.zeros((r_size + 1, r_size + 1))

    H[0:r_size, 0:r_size] = R
    H[0:r_size, r_size] = T
    H[r_size, r_size] = 1.0

    if Hin is None:
        return H
    else:
        return Hin @ H


def nearest_neighbor_association(previous_points, current_points):

    # calc the sum of residual errors
    delta_points = previous_points - current_points
    d = np.linalg.norm(delta_points, axis=0)
    error = sum(d)

    # calc index with nearest neighbor assosiation
    d = np.linalg.norm(np.repeat(current_points, previous_points.shape[1], axis=1)
                       - np.tile(previous_points, (1, current_points.shape[1])), axis=0)
    indexes = np.argmin(d.reshape(current_points.shape[1], previous_points.shape[1]), axis=1)

    return indexes, error


def svd_motion_estimation(previous_points, current_points):
    pm = np.mean(previous_points, axis=1)
    cm = np.mean(current_points, axis=1)

    p_shift = previous_points - pm[:, np.newaxis]
    c_shift = current_points - cm[:, np.newaxis]

    W = c_shift @ p_shift.T
    u, s, vh = np.linalg.svd(W)

    R = (u @ vh).T
    t = pm - (R @ cm)

    return R, t


def plot_points(previous_points, current_points, figure):
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])
    if previous_points.shape[0] == 3:
        plt.clf()
        axes = figure.add_subplot(111, projection='3d')
        axes.scatter(previous_points[0, :], previous_points[1, :],
                    previous_points[2, :], c="r", marker=".")
        axes.scatter(current_points[0, :], current_points[1, :],
                    current_points[2, :], c="b", marker=".")
        axes.scatter(0.0, 0.0, 0.0, c="r", marker="x")
        figure.canvas.draw()
    else:
        plt.cla()
        plt.plot(previous_points[0, :], previous_points[1, :], ".r")
        plt.plot(current_points[0, :], current_points[1, :], ".b")
        plt.plot(0.0, 0.0, "xr")
        plt.axis("equal")


