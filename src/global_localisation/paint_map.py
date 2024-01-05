from PIL import Image
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

from copy import deepcopy, copy


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

        #print(half_x, self.x_dim)
        #print(half_y, self.y_dim)

        for y in range(int(self.y_dim/self.pixel_size)+1):
            self.grid.append([])
            for x in range(int(self.x_dim/self.pixel_size)+1):
                self.grid[y].append(0)

        for i in range(len(array[0])):
            #print(array[1][i], array[0][i])
            #print(int(array[1][i]/self.pixel_size)+half_y, int(array[0][i]/self.pixel_size)+half_x)
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
                     #self.ax.scatter(x*self.pixel_size,y*self.pixel_size,c='b',s=8)
                     self.points[1].append(x*self.pixel_size + self.pixel_size/2 - x_size)
                     self.points[0].append(y*self.pixel_size + self.pixel_size/2 - y_size)
                     
        #self.ax.axis('equal')
        #plt.show()
                    

def rotate_vector(vector, angle):
    x = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
    y = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
    return [x, y]


def global_matcher(R, T, sizes, map_array, lidar_array, percent_outs=90, visualize=False):
    '''
    #get points
    a = GridMap(4)
    a.read_map_array(deepcopy(map_array), 20)
    a.draw()
    fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)
    ax.scatter(a.points[0],a.points[1],c='b',s=8)
    ax.axis('equal')
    plt.show()

    b = GridMap(4)
    b.read_lidar_points(deepcopy(lidar_array))
    b.draw()
    p = np.array(b.points)
    p = np.matmul(R, p) + T[:, np.newaxis]
    fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)
    ax.scatter(p[0],p[1],c='b',s=8)
    ax.axis('equal')
    plt.show()
    '''

    for i in range(len(sizes)):
        #TODO: only downsample the map not the scan -> same is done in hector slam


        #print("Detail: ", sizes[i])
        #get points
        a = GridMap(sizes[i])
        a.read_map_array(deepcopy(map_array), 20)
        a.draw()

        b = GridMap(sizes[i])
        b.read_lidar_points(deepcopy(lidar_array))
        b.draw()

        #print(a.points)
        #print(b.points)
        
        # rotate and translate
        p = np.array(b.points)      #-> a
        p = np.matmul(R, p) + T[:, np.newaxis]

        #R_new, T_new, error = icp_matching(np.array(a.points), p[0:2], show_animation=True, ignore_outliers=True, percent_outs=percent_outs)
        R_new, T_new, error = icp_matching(np.array(a.points), p[0:2], show_animation=visualize, ignore_outliers=True, percent_outs=percent_outs)    #-> b
        if(error != None):
            #print(R)
            #print(T)
            R = np.dot(R_new, R)
            T = np.dot(R_new, T) + T_new

        #input("next iteration?")
        
    '''
    #get points
    a = GridMap(4)
    a.read_map_array(deepcopy(map_array), 20)
    a.draw()

    b = GridMap(4)
    b.read_lidar_points(deepcopy(lidar_array))
    b.draw()
        
    # rotate and translate
    p = np.array(b.points)
    p = np.matmul(R, p) + T[:, np.newaxis]

    # plot
    fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)
    ax.scatter(a.points[0],a.points[1],c='b',s=8)
    ax.scatter(p[0],p[1],c='r',s=8)
    ax.axis('equal')
    plt.show()
    '''

    return T, R
    
    '''print()
    if(error < 600):
        print("------------- Matched! ----------")
        return True, T, R
    else:
        print("------------- NO match ----------")
        return False, T, R'''


def global_matcher_test(R, T, sizes, map_img, section_img):
    #sizes = [130, 116, 100, 82, 70, 58, 41, 29, 21, 15, 11, 8, 4]
    #sizes = [80, 70, 60, 40, 30, 20, 10, 4]

    old_points = [[],[]]

    for i in range(len(sizes)):
        #fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)
        print("Detail: ", sizes[i])

        #get points
        a = GridMap(sizes[i])
        a.read_png(map_img)
        a.draw()

        b = GridMap(sizes[i])
        b.read_png(section_img)
        b.draw()

        
        # rotate and translate
        p = np.array(b.points)
        p = np.matmul(R, p) + T[:, np.newaxis]

        # plot
        #ax.scatter(a.points[0],a.points[1],c='b',s=8)
        #ax.scatter(p[0],p[1],c='r',s=8)
        #ax.scatter(old_points[0],old_points[1],c='g',s=8)
        #ax.axis('equal')
        #plt.show()
        #old_points = deepcopy(p)

        R_new, T_new, error = icp_matching(np.array(a.points), p[0:2])
        print(R)
        print(T)
        R = np.dot(R_new, R)
        T = np.dot(R_new, T) + T_new

        #input("next iteration?")
        
    #get points
    a = GridMap(4)
    a.read_png(map_img)
    a.draw()

    b = GridMap(4)
    b.read_png(section_img)
    b.draw()
        
    # rotate and translate
    p = np.array(b.points)
    p = np.matmul(R, p) + T[:, np.newaxis]

    # plot
    '''
    fig, ax = plt.subplots(1, 1, sharex=True, sharey=True)
    ax.scatter(a.points[0],a.points[1],c='b',s=8)
    ax.scatter(p[0],p[1],c='r',s=8)
    ax.axis('equal')
    plt.pause(1)'''
    
    print()
    if(error < 600):
        print("------------- Matched! ----------")
        return True
    else:
        print("------------- NO match ----------")
        return False
    


# TODO: Benchmark tool erstellen

'''
# yes
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([500.0, -500.0])
global_matcher(R, T)

# no
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([-500.0, 500.0])
global_matcher(R, T)

# no
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([0.0, -500.0])
global_matcher(R, T)

# no
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([-500.0, 0.0])
global_matcher(R, T)    
'''
# no
#R = np.array([[1.0, 0.0],
#              [0.0, 1.0]])
#T = np.array([-500.0, -500.0])
#global_matcher(R, T)
'''
# no
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([0.0, 0.0])
global_matcher(R, T)

# yes
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([500.0, 0.0])
global_matcher(R, T)

# no
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([0.0, 500.0])
global_matcher(R, T)

# no
R = np.array([[1.0, 0.0],
              [0.0, 1.0]])
T = np.array([500.0, 500.0])
global_matcher(R, T)
'''        
