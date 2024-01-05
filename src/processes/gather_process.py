import time
import numpy as np
from lidar.lidar_matching import *
from lidar.lidar_advanced_filter import advanced_filter, advanced_filter_set_high
from multiprocessing import Queue
import PyLidar3, math
from settings import *

from copy import deepcopy

def gather_process(icp_queue):
    print("gather process start")
    if(not SIMULATION):
        Obj = PyLidar3.YdLidarX4(LIDAR_PORT, LIDAR_BUFFER_SIZE)  #PyLidar3.your_version_of_lidar(port,chunk_size)
        if(Obj.Connect() or True):
            Obj.Reset()
            print(Obj.GetDeviceInfo())
            gen = Obj.StartScanning()
            time.sleep(1)

            print("lidar startup complete")
            next(gen)
            prev_raw = next(gen)
            prev_x, prev_y, c = advanced_filter(prev_raw)

            try:
                while True:
                    Obj._s.reset_input_buffer()
                    current_raw = next(gen)

                    current_x, current_y, c = advanced_filter(current_raw.copy())
                    prev = np.array([prev_x, prev_y])
                    current = np.array([current_x, current_y])
                    R_change, T_change, error = icp_matching(prev, current, ignore_outliers=True)

                    if(error == None):
                        continue
                    if(math.sqrt(T_change[0]**2 + T_change[1]**2) > GATHER_TRANSLATION_THRESHOLD or math.degrees(math.asin(R_change[1][0])) > GATHER_ROTATION_THRESHOLD):
                        arr = advanced_filter_set_high(current_raw.copy())
                        icp_queue.put([deepcopy(T_change), deepcopy(R_change), error, deepcopy(current), arr.copy()])
                        prev_x = current_x.copy()
                        prev_y = current_y.copy()

            except KeyboardInterrupt:
                print("end gather process")

    else:
        #data = np.load("src/data.npy", allow_pickle=True)
        #data = np.load("src/map1.npy", allow_pickle=True)
        #data = np.load("src/crazy.npy", allow_pickle=True)
        #data = np.load("src/big_run_small_rot.npy", allow_pickle=True)
        #data = np.load("src/big_run_big_rot.npy", allow_pickle=True)
        #data = np.load("src/full_appartment.npy", allow_pickle=True)
        data = np.load("src/test_recordings/full_appartment_2.npy", allow_pickle=True)
        data = data[40:]
        #data = data[:-140]
        prev_raw = data[0]
        prev_x, prev_y, c = advanced_filter(prev_raw)

        try:
            for i in range(len(data)-1):
                start_time = time.time()
                current_raw = data[i+1].copy()

                current_x, current_y, c = advanced_filter(current_raw.copy())
                prev = np.array([prev_x, prev_y])
                current = np.array([current_x, current_y])
                R_change, T_change, error = icp_matching(prev, current, ignore_outliers=True, show_animation=False)

                if(error == None):
                    print("continue due to error")
                    continue
                if(math.sqrt(T_change[0]**2 + T_change[1]**2) > GATHER_TRANSLATION_THRESHOLD or math.degrees(math.asin(R_change[1][0])) > GATHER_ROTATION_THRESHOLD):    #15 grad und 100 mm
                    arr = advanced_filter_set_high(current_raw.copy())
                    icp_queue.put([deepcopy(T_change), deepcopy(R_change), error, deepcopy(current), arr.copy()])
                    prev_x = current_x.copy()
                    prev_y = current_y.copy()
                    print("gathered ", time.time() - start_time)

        except KeyboardInterrupt:
            print("end gather process")
