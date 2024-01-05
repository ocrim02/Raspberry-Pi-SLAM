from graphslam.vertex import *
from mapping.occupancy_map import *
import time
from settings import *

def mapping_process(map_queue, og_queue):
    initMapXLength, initMapYLength, unitGridSize, lidarFOV, lidarMaxRange = MAP_X_LENGTH, MAP_Y_LENGTH, MAP_GRID_SIZE, 2*np.pi, LIDAR_MAX_RANGE # in Meters
    wallThickness = MAP_WALL_SIZE

    plt.rcParams.update({'font.size': MAP_FONT_SIZE})
    fig, ax = plt.subplots(figsize=MAP_WINDOW_SIZE)
    scatter_start = ax.scatter([], [], color='r', s=500)
    scatter_path = ax.scatter([], [], color='g', s=60)
    scatter_end = ax.scatter([], [], color='b', s=500)
    line, = ax.plot([], [])
    # Create a blank image with the specified extent
    xRange = [-initMapXLength//2, initMapXLength//2]
    yRange = [-initMapYLength//2, initMapYLength//2]
    blank_image = np.ones((initMapXLength, initMapYLength))  # Adjust the size as needed
    ogMap = ax.imshow(blank_image, cmap='gray', extent=[xRange[0], xRange[1], yRange[0], yRange[1]], origin='upper', vmin=0, vmax=1)

    numSamplesPerRev = LIDAR_SAMPLES_PER_REV
    initXY = {"x": 0.0, "y": 0.0}

    og = OccupancyGrid(initMapXLength, initMapYLength, initXY, unitGridSize, lidarFOV, numSamplesPerRev, lidarMaxRange, wallThickness)
    xTrajectory, yTrajectory = [0], [0]

    ax.set_aspect('equal', adjustable='box')
    ax.axis('equal')
    
    plt.ion()  # Turn on interactive mode

    while True:
        while(map_queue.qsize() == 0):
            plt.pause(0.05)
        data = map_queue.get()

        start_time = time.time()
        vertices = data[0]
        poses = data[1]
        update = data[2]

        for i in range(len(vertices)):
            v = vertices[i]
            if(update[i] == True):
                sensorData={"range": poses[i][4],
                            "theta": poses[i][2] + 0*(np.pi/2.0),
                            "x": poses[i][0]/1000.0,
                            "y": poses[i][1]/1000.0}
                og.updateOccupancyGrid(sensorData, replace=True)
                sensorData={"range": poses[i][4],
                            "theta": v.pose.orientation + 0*(np.pi/2.0),
                            "x": v.pose.position[0]/1000.0,
                            "y": v.pose.position[1]/1000.0}
                og.updateOccupancyGrid(sensorData)
                replaceElementInTrajectoryPlot(sensorData, vertices[i].id, xTrajectory, yTrajectory)
                
            else:
                sensorData={"range": poses[i][4],
                            "theta": v.pose.orientation + 0*(np.pi/2.0),
                            "x": v.pose.position[0]/1000.0,
                            "y": v.pose.position[1]/1000.0}
                og.updateOccupancyGrid(sensorData)
                updateTrajectoryPlot(sensorData, xTrajectory, yTrajectory)
            
        calc_time = time.time()

        scatter_path.set_offsets(list(zip(xTrajectory[1:], yTrajectory[1:])))
        scatter_start.set_offsets([xTrajectory[0], yTrajectory[0]])
        scatter_end.set_offsets([xTrajectory[-1], yTrajectory[-1]])
        line.set_data(xTrajectory, yTrajectory)
        og.addOccupancyGridToPlot(ogMap, xRange, yRange)

        print("update plot after ", time.time() - start_time, "s Total, ", calc_time - start_time, "s of calculation for ", len(vertices), " data points")

        plt.pause(0.1)

        og_queue.put([og.occupancyGridTotal, og.occupancyGridVisited])

    plt.ioff()  # Turn off interactive mode
    plt.show()  # Display the final plot

    