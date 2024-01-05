
SIMULATION = True               # set to True if you want to use a prerecored path. Set to False if you want to use live lidar data

LIDAR_PORT = "/dev/ttyUSB0"     # linux
LIDAR_BUFFER_SIZE = 3000
LIDAR_MAX_RANGE = 5             # in m
LIDAR_SAMPLES_PER_REV = 360
LIDAR_MINIMUM_DISTANCE = 120    # minimum distance for measurements to be valid (in mm)
LIDAR_ANGLE_CORRECTION = 270    # if your Lidar orientation is not lined up with the driving direction of your robot

FILTER_RANGE_MULT = 0.05        # factor to calssify measurements as valid or noise

#  ICP parameters

EPS = 0.0001        # minimum rate of change or ICP is aborted
MAX_ITER = 100      # 40
ICP_OUTLIERS = 90   # percentage of points that are expected to have a neighbour

# Gather Process

GATHER_TRANSLATION_THRESHOLD = 100  # minimum translation between lidar scans to be added to the processing queue (in mm)
GATHER_ROTATION_THRESHOLD = 2       # minimum rotation between lidar scans to be added to the processing queue (in degrees)

# Mapping Process

MAP_FONT_SIZE = 22
MAP_WINDOW_SIZE = (10.80, 10.80)

MAP_X_LENGTH = 16                   # in m
MAP_Y_LENGTH = 16                   # in m
MAP_GRID_SIZE = 0.02                # in m
MAP_WALL_SIZE = 4 * MAP_GRID_SIZE   # in m


# Loop Closure Process

USE_GLOBAL_LOC = True       # Set True if you want to use the global localiser in the loop closure process (uses more cpu but helps with longer explorations)
LC_GLOBAL_LOC_FREQ = 5      # how often the global localiser is called if no other loops where closed
LC_GLOBAL_LOC_ITER = [30]   # resolution steps for the iterative matching of the global localiser

LC_SEARCH_FREQ = 5          # after the given amount of scans a new loop closure search will be started
LC_IGNORED_POSES = 20       # the last n poses will be ignored for the loop closure search to avoid small loops
LC_XY_THRES = 200           # maximum distance between two poses to be considered an ICP matching pair (mm)
LC_MAX_CLOSURES = 2         # after x loop closures per new pose the search is aborted

LC_UPDATE_THRES_XY = 50     # minimum change in XY required for a pose to be updated (mm)
LC_UPDATE_THRES_R = 2       # minimum change in R required for a pose to be updated (deg)

# not recommended to change
LC_MAT_XY = 1000            # stiffness of edges of the original path
LC_MAT_R = 1000000000       # stiffness of joints of edges of the original path

LC_MAT_ADD_XY = 1           # stiffness of edges added as constraints
LC_MAT_ADD_R = 1000000000   # stiffness of joints of edges added as constraints

LC_MAT_GLOB_LOC_XY = 1      # stiffness of edges added as constraints by global localisation
LC_MAT_GLOB_LOC_R = 1000000 # stiffness of joints of edges added as constraints by global localisation