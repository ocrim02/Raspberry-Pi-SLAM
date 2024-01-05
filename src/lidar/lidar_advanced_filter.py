# -*- coding: utf-8 -*-
"""
Created on Tue Apr 18 19:56:26 2023

@author: Mirco Richter
"""

import math
from settings import *

range_multiplier = FILTER_RANGE_MULT


def advanced_filter_set_high(raw_data):
    arr = []
    c = []
    for key in range(LIDAR_SAMPLES_PER_REV):
        color = 'r'
        if(abs(raw_data[((key-1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier and abs(raw_data[((key+1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier):
            color = 'b'
        elif(abs(raw_data[((key-1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier and abs(raw_data[((key-2) % LIDAR_SAMPLES_PER_REV)] - raw_data[((key-1) % LIDAR_SAMPLES_PER_REV)]) < raw_data[((key-2) % LIDAR_SAMPLES_PER_REV)]*range_multiplier):
            color = 'b'
        elif(abs(raw_data[((key+1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier and abs(raw_data[((key+2) % LIDAR_SAMPLES_PER_REV)] - raw_data[((key+1) % LIDAR_SAMPLES_PER_REV)]) < raw_data[((key+2) % LIDAR_SAMPLES_PER_REV)]*range_multiplier):
            color = 'b'
        c.append(color)

    for i in range(LIDAR_SAMPLES_PER_REV):
        if(c[(-i-LIDAR_ANGLE_CORRECTION)%LIDAR_SAMPLES_PER_REV] == 'b'):
            if(raw_data[(-i-LIDAR_ANGLE_CORRECTION)%LIDAR_SAMPLES_PER_REV]>LIDAR_MINIMUM_DISTANCE):
                arr.append(raw_data[(-i-LIDAR_ANGLE_CORRECTION)%LIDAR_SAMPLES_PER_REV]/1000.0)
            else:
                arr.append(10)
        else:
            arr.append(10)

    return arr

def advanced_filter(raw_data, angle_correction=LIDAR_ANGLE_CORRECTION, x_off=0, y_off=0):
    x = []
    y = []
    c = []
    for key in range(LIDAR_SAMPLES_PER_REV):
        color = 'r'
        if(abs(raw_data[((key-1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier and abs(raw_data[((key+1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier):
            color = 'b'
        elif(abs(raw_data[((key-1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier and abs(raw_data[((key-2) % LIDAR_SAMPLES_PER_REV)] - raw_data[((key-1) % LIDAR_SAMPLES_PER_REV)]) < raw_data[((key-2) % LIDAR_SAMPLES_PER_REV)]*range_multiplier):
            color = 'b'
        elif(abs(raw_data[((key+1) % LIDAR_SAMPLES_PER_REV)] - raw_data[key]) < raw_data[key]*range_multiplier and abs(raw_data[((key+2) % LIDAR_SAMPLES_PER_REV)] - raw_data[((key+1) % LIDAR_SAMPLES_PER_REV)]) < raw_data[((key+2) % LIDAR_SAMPLES_PER_REV)]*range_multiplier):
            color = 'b'

        if(color == 'b'):
            if(raw_data[key]>LIDAR_MINIMUM_DISTANCE):
                x.append((-1) * raw_data[key] * math.cos(math.radians(key + angle_correction)) + x_off)
                y.append(raw_data[key] * math.sin(math.radians(key + angle_correction)) + y_off)
                c.append(color)

    return x, y, c
