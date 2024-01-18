from lidar.lidar_matching import *
from utils.functions import *
from multiprocessing import Queue

from graphslam.vertex import *
from graphslam.edge.edge_odometry import *
from graphslam.graph import *
from graphslam.pose.base_pose import *

from global_localisation.paint_map import global_matcher
from settings import *

from copy import deepcopy
import time


def loop_closure_process(icp_queue, map_queue, og_queue):
    # current rotation and position of the robot
    rotation = 0
    T0 = 0.0
    T1 = 0.0

    poses = [[0,0,0,[],[]]]
    edges = []
    vertices = [Vertex(0, PoseSE2([0, 0], 0))]

    g = Graph(edges, vertices)

    info_mat = np.ndarray(shape=(3,3), dtype=int)
    info_mat[0][0] = LC_MAT_XY
    info_mat[0][1] = 0
    info_mat[0][2] = 0
    info_mat[1][0] = 0
    info_mat[1][1] = LC_MAT_XY
    info_mat[1][2] = 0
    info_mat[2][0] = 0
    info_mat[2][1] = 0
    info_mat[2][2] = LC_MAT_R

    info_mat_add = np.ndarray(shape=(3,3), dtype=int)
    info_mat_add[0][0] = LC_MAT_ADD_XY
    info_mat_add[0][1] = 0
    info_mat_add[0][2] = 0
    info_mat_add[1][0] = 0
    info_mat_add[1][1] = LC_MAT_ADD_XY
    info_mat_add[1][2] = 0
    info_mat_add[2][0] = 0
    info_mat_add[2][1] = 0
    info_mat_add[2][2] = LC_MAT_ADD_R
    
    info_mat_glob = np.ndarray(shape=(3,3), dtype=int)
    info_mat_glob[0][0] = LC_MAT_GLOB_LOC_XY
    info_mat_glob[0][1] = 0
    info_mat_glob[0][2] = 0
    info_mat_glob[1][0] = 0
    info_mat_glob[1][1] = LC_MAT_GLOB_LOC_XY
    info_mat_glob[1][2] = 0
    info_mat_glob[2][0] = 0
    info_mat_glob[2][1] = 0
    info_mat_glob[2][2] = LC_MAT_GLOB_LOC_R

    last_closure = 0

    while True:
        icp_data = icp_queue.get()
        R_change = icp_data[1]
        T_change = icp_data[0]
        error = icp_data[2]
        scan = icp_data[3]
        raw = icp_data[4]

        start_time = time.time()

        t = rotate_vector(T_change, rotation)
        rotation = math.degrees(poses[-1][2]) + math.degrees(math.asin(R_change[1][0]))
        T0 = poses[-1][0] + t[0]
        T1 = poses[-1][1] + t[1]

        g._edges.append(EdgeOdometry([len(poses)-1, len(poses)], info_mat, PoseSE2([T_change[0], T_change[1]], math.asin(R_change[1][0]))))
        poses.append([T0, T1, math.radians(rotation), scan, raw])
        g._vertices.append(Vertex(len(vertices), PoseSE2([T0, T1], math.radians(rotation))))

        # pose based loop closure
        loop_closes = 0
        if(len(vertices) - last_closure > LC_SEARCH_FREQ):
            for i in range(1, len(poses[LC_IGNORED_POSES:])):
                if(math.sqrt((poses[i][0] - poses[-1][0])**2 + (poses[i][1] - poses[-1][1])**2) < LC_XY_THRES):
                    print("additional edge added", i , " ", len(poses)-1)
                    pre = poses[i][3].copy()
                    cur = poses[-1][3].copy()
                    rotation_offset = poses[-1][2] - poses[i][2]
                    if(abs(rotation_offset) > np.pi/90.0):
                        pre = rotate_points(pre, poses[i][2] - poses[-1][2])
                    else:
                        rotation_offset = 0
                    r, t, error = icp_matching(pre, cur, ignore_outliers=True, show_animation=False)

                    if(type(r) != np.ndarray):
                        continue
                    loop_closes += 1
                    g._edges.append(EdgeOdometry([i, len(poses)-1], info_mat_add, PoseSE2([t[0], t[1]], math.asin(r[1][0]) + rotation_offset)))
                    last_closure = len(vertices)
                    if(loop_closes >= LC_MAX_CLOSURES):
                        break
        
        # global localiser loop closure
        if(USE_GLOBAL_LOC and len(vertices)%LC_GLOBAL_LOC_FREQ == 0 and loop_closes == 0 and og_queue.qsize() != 0 and len(vertices)>6):
            while(og_queue.qsize() != 0):
                og = og_queue.get()
            occupancyGridTotal = og[0]
            occupancyGridVisited = og[1]
            ogMap = occupancyGridVisited / occupancyGridTotal
            lidar_points = scan.copy()
            lidar_points[1] = lidar_points[1] * (-1)
            R = np.array([[math.cos(math.radians(rotation-90)), -math.sin(math.radians(rotation-90))],
                        [math.sin(math.radians(rotation-90)), math.cos(math.radians(rotation-90))]])
            T = np.array([T0, T1])
            t, r = global_matcher(R, T, LC_GLOBAL_LOC_ITER, ogMap, lidar_points, visualize=False)
            print("Global Result: ", t[0], t[1], -(math.degrees(math.asin(r[1][0]))+90))
            print("Local Result: ", T0, T1, rotation)
            if(abs(rotation-(-(math.degrees(math.asin(r[1][0]))+90))) < 10):
                print("GLOBAL ADD")
                g._edges.append(EdgeOdometry([0, len(poses)-1], info_mat_glob, PoseSE2([t[0], t[1]], -(math.asin(r[1][0]) + math.radians(90)))))
        
        g._link_edges()
        g.optimize()

        # update latest pose to match its vertice
        poses[-1][0] = g._vertices[-1].pose.position[0]
        poses[-1][1] = g._vertices[-1].pose.position[1]
        poses[-1][2] = g._vertices[-1].pose.orientation

        # add latest position
        v = [g._vertices[-1]]
        p = [poses[-1].copy()]
        update = [False]

        # check for changes and request update if necessary
        for i in range(len(poses)-1):
            if(abs(poses[i][0] - g._vertices[i].pose.position[0]) > LC_UPDATE_THRES_XY):
                p.append(poses[i].copy())
                v.append(g._vertices[i])
                update.append(True)
                poses[i][0] = g._vertices[i].pose.position[0]
                poses[i][1] = g._vertices[i].pose.position[1]
                poses[i][2] = g._vertices[i].pose.orientation

            elif(abs(poses[i][1] - g._vertices[i].pose.position[1]) > LC_UPDATE_THRES_XY):
                p.append(poses[i].copy())
                v.append(g._vertices[i])
                update.append(True)
                poses[i][0] = g._vertices[i].pose.position[0]
                poses[i][1] = g._vertices[i].pose.position[1]
                poses[i][2] = g._vertices[i].pose.orientation

            elif(abs(poses[i][2] - g._vertices[i].pose.orientation) > (np.pi/180.0)*LC_UPDATE_THRES_R):
                p.append(poses[i].copy())
                v.append(g._vertices[i])
                update.append(True)
                poses[i][0] = g._vertices[i].pose.position[0]
                poses[i][1] = g._vertices[i].pose.position[1]
                poses[i][2] = g._vertices[i].pose.orientation

        
        map_queue.put([deepcopy(v), p, update])
        print("Queue Map ", time.time() - start_time)




