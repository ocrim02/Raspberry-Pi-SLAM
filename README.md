# Raspberry-Pi-SLAM
This repository contains code for a 2D LiDAR SLAM algorithm designed to work run embedded on a Raspberry Pi 3b or better with sparse LiDAR data as its only input.

![Mapping Example!](/images/example.PNG "Mapping example done in real-time on Raspberry Pi")

## Getting started
1. Clone this repository onto your machine (Raspeberry Pi 3b or better).
2. `pip install -r requirements.txt`
3. Check settings.py to match your environment and hardware. If you are using a YDLIDAR X4 you only need to change the LIDAR_PORT and LIDAR_ANGLE_CORRECTION. Otherwise check processes/gather_process.py and change to Code to match your LiDAR.
4. Set SIMULATION in settings.py to False.
5. Run main.py.

## Use other LiDAR's
If you do not have a YDLIDAR X4 you need to make some changes in processes/gather_process.py.
You can find the current code that fetches data from the LiDAR in this file. 
Replace this code with your LiDAR's code.

NOTE: 2D LiDAR data is required in the form `{0: <distance in mm>, 1: <distance in mm>, ...}` where the key is the angle at which the value was measured.


