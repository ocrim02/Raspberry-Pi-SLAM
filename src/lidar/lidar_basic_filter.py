# -*- coding: utf-8 -*-
"""
Created on Tue Apr 18 19:56:26 2023

@author: Mirco
"""

import math
from settings import *


def basic_filter(data):
    x = []
    y = []

    for angle in range(0,360):
        if(data[angle]>LIDAR_MINIMUM_DISTANCE):
            x[angle] = (-1) * data[angle] * math.cos(math.radians(angle + LIDAR_ANGLE_CORRECTION))
            y[angle] = data[angle] * math.sin(math.radians(angle + LIDAR_ANGLE_CORRECTION))

    for i in range(360):
        if(x[i] != 0 and y[i] != 0 and math.sqrt(pow(x[i], 2) + pow(y[i], 2)) < 1600):
            count = 0
            for z in range(360):
                if(abs(x[i]-x[z]) + abs(y[i]-y[z]) < 100):
                    count += 1
            if(count < 5):
                x[i] = 0
                y[i] = 0

    return x, y

