# -*- coding: utf-8 -*-
"""
Created on Tue Apr 18 19:56:26 2023

@author: Mirco
"""

import PyLidar3
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math    
import time

angle_correction = 270

def draw():
    fig = plt.figure(1)
    plt.cla()
    #plt.ylim(-9000,9000)
    #plt.xlim(-9000,9000)

    #filter
    start_time = time.time()
    for i in range(360):
        if(x[i] != 0 and y[i] != 0 and math.sqrt(pow(x[i], 2) + pow(y[i], 2)) < 1600):
            count = 0
            for z in range(360):
                if(abs(x[i]-x[z]) + abs(y[i]-y[z]) < 100):
                    count += 1
            if(count < 5):
                x[i] = 0
                y[i] = 0
    print(time.time() - start_time)
    
    ax = fig.add_subplot(111)
    ax.scatter(x,y,c='r',s=8)
    ax.add_patch(Rectangle(
        xy=(-120, -120) ,width=240, height=240,
        linewidth=1, color='blue', fill=False))
    ax.axis('equal')
    plt.pause(0.001)
    plt.show()
    #plt.close("all")
    
                
is_plot = True
x=[]
y=[]
for _ in range(360):
    x.append(0)
    y.append(0)

#port =  input("Enter port name which lidar is connected:") #windows
port = "/dev/ttyUSB0" #linux
Obj = PyLidar3.YdLidarX4(port)  #PyLidar3.your_version_of_lidar(port,chunk_size)
#threading.Thread(target=draw).start()
if(Obj.Connect() or True):
    print(Obj.GetDeviceInfo())
    gen = Obj.StartScanning()
    time.sleep(1)
    data = next(gen)
    #print(data)
    for angle in range(0,360):
        if(data[angle]>120):
            x[angle] = (-1) * data[angle] * math.cos(math.radians(angle + angle_correction))
            y[angle] = data[angle] * math.sin(math.radians(angle + angle_correction))
    Obj.StopScanning()
    Obj.Disconnect()
    draw()
else:
    print("Error connecting to device")
