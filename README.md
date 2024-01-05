# Raspberry-Pi-SLAM
In recent years, Simultaneous Localisation and Mapping (SLAM) algorithms have become integral to the advancement of robotics, enabling machines to autonomously navigate and map their surroundings. While the applications of SLAM are diverse and promising, the cost associated with implementing this technology has been a significant barrier, making it difficult for enthusiasts, hobbyists and educational institutions with limited budgets to access this technology. Recognising the transformative potential of SLAM in fostering innovation and learning, this work seeks to explore and present a SLAM algorithm tailored to work seamlessly with the widely available and affordable Raspberry Pi microcomputer, coupled with a low-cost LiDAR sensor. This research provides an implementation of a real-time SLAM algorithm designed for the Raspberry Pi, using only sparse LiDAR data.

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


