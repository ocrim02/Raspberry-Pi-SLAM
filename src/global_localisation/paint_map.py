from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy, copy
import math


from lidar.lidar_matching import *


class GridMap:
    def __init__(self, pixel_size):
        self.pixel_size = pixel_size
        self.x_dim = 0
        self.y_dim = 0
        self.grid = []
        self.points = [[],[]]
        #fig, self.ax = plt.subplots(1, 1, sharex=True, sharey=True)


    def read_lidar_points(self, array):
        half_x = int((max(max(array[0]), abs(min(array[0]))))/self.pixel_size)
        half_y = int((max(max(array[1]), abs(min(array[1]))))/self.pixel_size)
        self.x_dim = int(max(max(array[0]), abs(min(array[0])))*2)
        self.y_dim = int(max(max(array[1]), abs(min(array[1])))*2)

        for y in range(int(self.y_dim/self.pixel_size)+1):
            self.grid.append([])
            for x in range(int(self.x_dim/self.pixel_size)+1):
                self.grid[y].append(0)

        for i in range(len(array[0])):
            self.grid[int(array[1][i]/self.pixel_size)+half_y][int(array[0][i]/self.pixel_size)+half_x] = 1


    def read_map_array(self, array, map_pixel_size=20):
        self.y_dim = (len(array) * map_pixel_size) + 1
        self.x_dim = (len(array[0]) * map_pixel_size) + 1

        for y in range(int(self.y_dim/self.pixel_size)+1):
            self.grid.append([])
            for x in range(int(self.x_dim/self.pixel_size)+1):
                self.grid[y].append(0)

        thres = np.amax(array)*0.85
        for y in range(len(array)):
            for x in range(len(array[0])):
                value = array[x][y]
                # Check if the point is occupied
                if value > thres:
                    self.grid[int((y*map_pixel_size)/self.pixel_size)][int((x*map_pixel_size)/self.pixel_size)] = 1


    def read_png(self, path):
        image = Image.open(path)
        # Convert the image to grayscale mode
        image = image.convert("L")
        self.x_dim, self.y_dim = image.size

        for y in range(int(self.y_dim/self.pixel_size)+1):
            self.grid.append([])
            for x in range(int(self.x_dim/self.pixel_size)+1):
                self.grid[y].append(0)

        for y in range(self.y_dim):
            for x in range(self.x_dim):
                # Get the pixel value at (x, y)
                pixel_value = image.getpixel((x, y))
                
                # Check if the pixel is black or white
                if pixel_value < 128:
                    # Black pixel
                    #print(f"Pixel at ({x}, {y}): Black")
                    self.grid[int(y/self.pixel_size)][int(x/self.pixel_size)] = 1

    def draw(self):
        y_size = int(self.y_dim/2)
        x_size = int(self.x_dim/2)
        for y in range(int(self.y_dim/self.pixel_size)):
            for x in range(int(self.x_dim/self.pixel_size)):
                if(self.grid[y][x] == 1):
                     self.points[1].append(x*self.pixel_size + self.pixel_size/2 - x_size)
                     self.points[0].append(y*self.pixel_size + self.pixel_size/2 - y_size)

                    

def rotate_vector(vector, angle):
    x = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
    y = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
    return [x, y]


def global_matcher(R, T, sizes, map_array, lidar_array, percent_outs=90, visualize=False):
    for i in range(len(sizes)):
        #get points
        a = GridMap(sizes[i])
        a.read_map_array(deepcopy(map_array), 20)
        a.draw()

        b = GridMap(sizes[i])
        b.read_lidar_points(deepcopy(lidar_array))
        b.draw()
        
        # rotate and translate
        p = np.array(b.points)
        p = np.matmul(R, p) + T[:, np.newaxis]

        R_new, T_new, error = icp_matching(np.array(a.points), p[0:2], show_animation=visualize, ignore_outliers=True, percent_outs=percent_outs)
        if(error != None):
            R = np.dot(R_new, R)
            T = np.dot(R_new, T) + T_new

    return T, R
