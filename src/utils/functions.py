import math
import numpy as np

def rotate_vector(vector, angle):
    x = vector[0] * math.cos(math.radians(angle)) - vector[1] * math.sin(math.radians(angle))
    y = vector[0] * math.sin(math.radians(angle)) + vector[1] * math.cos(math.radians(angle))
    return [x, y]

def rotate_points(point_array, angle_radians):
    # Calculate the rotation matrix
    rotation_matrix = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                                [np.sin(angle_radians), np.cos(angle_radians)]])
    # Apply the rotation to all points at once
    rotated_points = np.dot(rotation_matrix, point_array)

    return rotated_points

def rotate_point(x, y, theta_deg):
    # Convert degrees to radians
    theta_rad = math.radians(theta_deg)
    
    # Apply rotation transformation
    x_rotated = x * math.cos(theta_rad) - y * math.sin(theta_rad)
    y_rotated = x * math.sin(theta_rad) + y * math.cos(theta_rad)
    
    return x_rotated, y_rotated