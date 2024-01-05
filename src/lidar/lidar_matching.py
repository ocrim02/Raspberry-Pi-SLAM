"""
Iterative Closest Point (ICP) SLAM example
original author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı, Shamil Gemuev
modified by: Mirco Richter
"""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from settings import *

def icp_matching(previous_points, current_points, ignore_outliers, show_animation=False, percent_outs = ICP_OUTLIERS):
    """
    Iterative Closest Point matching
    - input
    previous_points: 2D or 3D points in the previous frame
    current_points: 2D or 3D points in the current frame
    - output
    R: Rotation matrix
    T: Translation vector
    """
    global MAX_ITER
    global EPS
    H = None  # homogeneous transformation matrix

    dError = np.inf
    preError = np.inf
    count = 0
    error = np.inf
    prev_indexes = []

    if show_animation:
        fig = plt.figure()
        if previous_points.shape[0] == 3:
            fig.add_subplot(111, projection='3d')

    while (dError >= EPS):
        count += 1

        if show_animation:  # pragma: no cover
            plot_points(previous_points, current_points, fig)
            for i in range(len(prev_indexes)):
                plt.plot([previous_points[0, prev_indexes[i]], current_points[0,current_indexes[i]]], [previous_points[1, prev_indexes[i]], current_points[1,current_indexes[i]]], c='g')
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.pause(0.1)
            #plt.show()

        prev_indexes, current_indexes, error = nearest_neighbor_association(previous_points, current_points, ignore_outliers, percent_outs)

        #for i in range(len(prev_indexes)):
        #    plt.plot([previous_points[0, prev_indexes[i]], current_points[0,current_indexes[i]]], [previous_points[1, prev_indexes[i]], current_points[1,current_indexes[i]]], c='g')
        #plt.show()
        
        Rt, Tt = svd_motion_estimation(previous_points[:, prev_indexes], current_points[:, current_indexes])
        # update current points
        current_points = (Rt @ current_points) + Tt[:, np.newaxis]

        dError = abs(preError - error)
        #print("Residual:", dError)

        if dError < 0:  # prevent matrix H changing, exit loop
            print("Not Converge...", preError, dError, count)
            return None, None, None

        preError = error
        H = update_homogeneous_matrix(H, Rt, Tt)
        
        if(dError <= EPS):
            #print("Converge", error, dError, count)
            if show_animation:  # pragma: no cover
                plot_points(previous_points, current_points, fig)
                for i in range(len(prev_indexes)):
                    plt.plot([previous_points[0, prev_indexes[i]], current_points[0,current_indexes[i]]], [previous_points[1, prev_indexes[i]], current_points[1,current_indexes[i]]], c='g')
                plt.pause(0.5)
                #plt.show()
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


def nearest_neighbor_association(previous_points, current_points, ignore_outliers, percent_outs):
    diff = current_points[:, :, np.newaxis] - previous_points[:, np.newaxis, :]
    matrix = np.linalg.norm(diff, axis=0)

    indexes = np.argmin(matrix, axis=1)
    values = np.min(matrix, axis=1)
        
    # set according to task
    if(ignore_outliers):
        percentage = percent_outs
    else:
        percentage = 100
    
    threshold = np.percentile(values, percentage)
    current_indexes = np.where(values <= threshold)[0]
    prev_indexes = indexes[current_indexes]

    # calc the sum of residual errors
    delta_points = previous_points[:, prev_indexes] - current_points[:, current_indexes]
    d = np.linalg.norm(delta_points, axis=0)
    error = sum(d)

    return prev_indexes, current_indexes, error


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

