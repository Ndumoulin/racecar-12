#!/usr/bin/env python3

# NOTE: "Band-Aid" for `ros2 run` issue. Not required when using `python3` to
# run the script.
#import sys
#if "/usr/local/lib/python3.12/dist-packages" in sys.path:
#    sys.path.remove("/usr/local/lib/python3.12/dist-packages")

import rclpy
import rclpy.logging
from rclpy.node import Node
import cv2
import numpy as np
from nav_msgs.srv import GetMap
from racecar_behaviors.libbehaviors import brushfire
from racecar_behaviors.libbehaviors import wavefront
from racecar_behaviors.libbehaviors import combine_maps
from racecar_behaviors.libbehaviors import extract_path_from_combined

class Path_Planning(Node):
    def __init__(self):
        super().__init__('brushfire')
        self.prefix = "rtabmap"
        self.get_map_client = self.create_client(GetMap, self.prefix + '/get_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def export_brushfire_map(self, brushfire_map):
        # Adjust color: 0 (black) = obstacle, 10-255 (white) = safest cells
        maximum = np.amax(brushfire_map)
        if maximum > 0:
            mask = brushfire_map == 1
            brushfire_map = brushfire_map.astype(float) / float(maximum) * 225.0 + 30.0
            brushfire_map[mask] = 0
            brushfire_map = brushfire_map.astype(np.uint8)  # Removes "type warning" from OpenCV
            # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
            cv2.imwrite('brushfire.bmp', cv2.transpose(cv2.flip(brushfire_map, -1)))
            self.get_logger().info("Exported brushfire.bmp")
        else:
            self.get_logger().info("Brushfire failed! Is brushfire implemented?")

    def export_wavefront_map(self, wavefront_map):
        # Adjust color: 0 (black) = goal, 10-255 (white) = farthest cells
        maximum = np.amax(wavefront_map)
        if maximum > 0:
            mask = wavefront_map == 1
            wavefront_map = wavefront_map.astype(float) / float(maximum) * 225.0 + 30.0
            wavefront_map[mask] = 0
            wavefront_map = wavefront_map.astype(np.uint8)  # Removes "type warning" from OpenCV
            # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
            cv2.imwrite('wavefront.bmp', cv2.transpose(cv2.flip(wavefront_map, -1)))
            self.get_logger().info("Exported wavefront.bmp")
        else:
            self.get_logger().info("Wavefront failed! Is wavefront implemented?")

    def export_combined_map(self, combined_map):
        # Adjust color: 0 (black) = goal/obstacle, 10-255 (white) = safest/farthest cells
        maximum = np.amax(combined_map)
        if maximum > 0:
            mask = combined_map == 1
            combined_map = combined_map.astype(float) / float(maximum) * 225.0 + 30.0
            combined_map[mask] = 0
            combined_map = combined_map.astype(np.uint8)  # Removes "type warning" from OpenCV
            # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
            cv2.imwrite('combined.bmp', cv2.transpose(cv2.flip(combined_map, -1)))
            self.get_logger().info("Exported combined.bmp")
        else:
            self.get_logger().info("Combined map failed! Is combine_maps implemented?")
    # ...existing code...
    def export_path_map(self, combined_map, path):
        """
        Saves combined_map as BMP and draws the optimal path in black.
        """
        # Normalization for display
        maximum = np.amax(combined_map)
        mask = combined_map == 1
        img = combined_map.astype(float) / float(maximum) * 225.0 + 30.0
        img[mask] = 0
        img = img.astype(np.uint8)

        # Convert to color image (BGR)
        img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        # Draw path in black (BGR: [0, 0, 0])
        for (r, c) in path:
            img_color[r, c] = [0]

        # Flip to match RVIZ orientation
        img_color = cv2.transpose(cv2.flip(img_color, -1))
        cv2.imwrite('path_on_combined.bmp', img_color)
        self.get_logger().info("Exported path_on_combined.bmp")
    

    def export_grid_map(self, grid):
        # Example to show grid with same color as RVIZ
        img = np.zeros_like(grid).astype(np.uint8)  # Avoids OverflowError from NumPy
        img[grid == -1] = 89
        img[grid == 0] = 178
        img[grid == 100] = 0
        # Flip image to get x->up, y->left (like top view in RVIZ looking towards x-axis)
        cv2.imwrite('map.bmp', cv2.transpose(cv2.flip(img, -1)))
        self.get_logger().info("Exported map.bmp")

    def get_map_callback(self, future):
        response = future.result()
        self.get_logger().info("Got map=%dx%d resolution=%f" %(response.map.info.height, response.map.info.width, response.map.info.resolution))
        #rospy.loginfo("Got map=%dx%d resolution=%f", response.map.info.height, response.map.info.width, response.map.info.resolution)
        grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
        brushfire_map = brushfire(grid)
        self.export_brushfire_map(brushfire_map)
        self.export_grid_map(grid)

        # Addition: calculate and export wavefront map
        goal = (60, 200)  # example: center of the map
        wavefront_map = wavefront(grid, goal)
        self.export_wavefront_map(wavefront_map)

        alpha = 5.0  # weight for combined map
        combined_map = combine_maps(wavefront_map, brushfire_map, alpha)
        self.export_combined_map(combined_map)

        start = (0, 0)  # example: start point
        path = extract_path_from_combined(combined_map, start, goal=goal)
        self.export_path_map(combined_map, path)

    def main(self):
        request = GetMap.Request()
        future = self.get_map_client.call_async(request)
        future.add_done_callback(self.get_map_callback)

def main(args=None):
    rclpy.init(args=args)
    path_panning_node = Path_Planning()
    path_panning_node.main()
    rclpy.spin(path_panning_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
