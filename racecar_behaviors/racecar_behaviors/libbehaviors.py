#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
import tf2_ros
import numpy as np
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformListener
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_point
import tf_transformations as transformations

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = transformations.translation_matrix(trans1)
    rot1_mat   = transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = transformations.translation_matrix(trans2)
    rot2_mat    = transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = transformations.translation_from_matrix(mat3)
    rot3 = transformations.quaternion_from_matrix(mat3)
    
    return trans3, rot3

def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
    # 0 = Path, 100 = Unknown, -1 = obstacle
    mapOfWorld[occupancyGrid==100] = 1 # set all unknowns and obstacles to 1
    mapOfWorld[occupancyGrid==-1] = 1 
    
    # do brushfire algorithm here
    nRows = mapOfWorld.shape[0]
    nCols = mapOfWorld.shape[1]
    a = 1

    while np.any(mapOfWorld == 0):
        for iRow in range(nRows):
            for iCol in range(nCols):
                if mapOfWorld[iRow][iCol] == a:
                    if (iRow > 0):
                        if (mapOfWorld[iRow-1][iCol] == 0) : mapOfWorld[iRow-1][iCol] = a + 1
                    if (iRow < nRows-1):
                        if (mapOfWorld[iRow+1][iCol] == 0) : mapOfWorld[iRow+1][iCol] = a + 1
                    if (iCol > 0):
                        if (mapOfWorld[iRow][iCol-1] == 0) : mapOfWorld[iRow][iCol-1] = a + 1
                    if (iCol < nCols-1):
                        if (mapOfWorld[iRow][iCol+1] == 0) : mapOfWorld[iRow][iCol+1] = a + 1
                        
        a += 1
    
    # brushfire: 1 = obstacle or unknown, safer cells have higher value)
    
    return mapOfWorld 

def wavefront(occupancyGrid, goal):
   
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
 
    # obstacles = 1
    mapOfWorld[occupancyGrid == 100] = 1
    mapOfWorld[occupancyGrid == -1] = 1
 
    nRows, nCols = mapOfWorld.shape
 
    # --- INITIALIZATION ---
    goal_r, goal_c = goal
 
    if mapOfWorld[goal_r, goal_c] == 1:
        raise ValueError("Goal is an obstacle!")
 
    # start wavefront
    mapOfWorld[goal_r, goal_c] = 2  # start value (can be 1)
 
    a = 2
 
    # --- WAVEFRONT PROPAGATION ---
    while True:
        changed = False
 
        for r in range(nRows):
            for c in range(nCols):
                if mapOfWorld[r, c] == a:
                    # NEIGHBORS
                    if r > 0 and mapOfWorld[r-1, c] == 0:
                        mapOfWorld[r-1, c] = a + 1
                        changed = True
                    if r < nRows-1 and mapOfWorld[r+1, c] == 0:
                        mapOfWorld[r+1, c] = a + 1
                        changed = True
                    if c > 0 and mapOfWorld[r, c-1] == 0:
                        mapOfWorld[r, c-1] = a + 1
                        changed = True
                    if c < nCols-1 and mapOfWorld[r, c+1] == 0:
                        mapOfWorld[r, c+1] = a + 1
                        changed = True
 
        if not changed:
            break
 
        a += 1
    
    return mapOfWorld

def combine_maps(map_wavefront, map_brushfire, alpha):
    combine_maps = np.zeros(map_wavefront.shape, dtype=int)

    for i in range(map_wavefront.shape[0]):
        for j in range(map_wavefront.shape[1]):
            if map_wavefront[i][j] == 1 or map_brushfire[i][j] == 1:
                combine_maps[i][j] = 1  # obstacle
            else:
                combine_maps[i][j] = map_wavefront[i][j] + alpha * map_brushfire[i][j]
    return combine_maps

def extract_path_from_combined(combined_map, start, goal):
    path = []
    current = start
    path.append(current)
    
    rows, cols = combined_map.shape
    
    while current != goal:
        r, c = current
        
        # Find neighbor with minimum value
        min_val = float('inf')
        next_node = None
        
        # 4-connectivity
        neighbors = [
            (r-1, c), (r+1, c), 
            (r, c-1), (r, c+1)
        ]
        
        for nr, nc in neighbors:
            if 0 <= nr < rows and 0 <= nc < cols:
                val = combined_map[nr, nc]
                if val != 1 and val < min_val: # 1 is obstacle
                    min_val = val
                    next_node = (nr, nc)
        
        if next_node is None or next_node == current:
            # Stuck or reached local minimum
            break
            
        current = next_node
        path.append(current)
        
        if len(path) > rows * cols: # Safety break
            break
            
    return path






def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('your_node_name')  # Change 'your_node_name' to a suitable name

    # Your ROS 2 specific setup here

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
