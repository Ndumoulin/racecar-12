#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from PIL import Image
from nav_msgs.srv import GetMap
import os
import time 

class PathToBitmap(Node):
    def __init__(self):
        super().__init__('path_to_bitmap')

        self.prefix = "rtabmap"   
        self.latest_map = None
        self.latest_path_pixels = []
        
        self.file_counter = 0

        # Get map via service
        self.get_map_client = self.create_client(GetMap, self.prefix + '/get_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Timer to call the service
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Subscribe to the path
        self.path_sub = self.create_subscription(
            Path,
            '/a_star_path',
            self.path_callback,
            10
        )
        
        # remembers last goal 
        self.last_goal = None


        self.get_logger().info("Waiting for map and path...")

        
    def timer_callback(self):
            request = GetMap.Request()
            future = self.get_map_client.call_async(request)
            future.add_done_callback(self.get_map_callback)

    def path_callback(self, msg: Path):
        self.latest_path_pixels = []

        if self.latest_map is None:
            return

        info = self.latest_map["info"]
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y

        for pose in msg.poses:
            wx = pose.pose.position.x
            wy = pose.pose.position.y

            gx = int((wx - ox) / res)
            gy = int((wy - oy) / res)

            self.latest_path_pixels.append((gx, gy))

        # ----- NEW: Check if goal changed -----
        new_goal = self.latest_path_pixels[-1]

        if self.last_goal == new_goal:
            self.get_logger().info("Goal unchanged → Skipping path rendering")
            return

        # Update last goal
        self.last_goal = new_goal

        # Goal changed → generate new BMP
        self.render_bitmap()



    def get_map_callback(self, future):
        response = future.result()

        h = response.map.info.height
        w = response.map.info.width

        grid = np.array(response.map.data, dtype=int).reshape((h, w))

        self.latest_map = {
            "grid": grid,
            "info": response.map.info
        }

        
        self.get_logger().info("Map updated.")


    def render_bitmap(self):
        if self.latest_map is None:
            return
        if not self.latest_path_pixels:
            return
    
        grid = self.latest_map["grid"]
        info = self.latest_map["info"]
    
        height, width = grid.shape
    
        image = np.zeros((height, width, 3), dtype=np.uint8)
    
        # Colors
        image[grid == -1] = [160, 160, 160]
        image[grid >= 50] = [0, 0, 0]
        image[grid >= 0]  = [255, 255, 255]
    
        # Draw path
        for (gx, gy) in self.latest_path_pixels:
            if 0 <= gx < width and 0 <= gy < height:
                image[gy, gx] = [255, 0, 0]
    
        # Draw goal (red)
        gx, gy = self.latest_path_pixels[-1]
        if 0 <= gx < width and 0 <= gy < height:
            image[gy, gx] = [255, 0, 0]
    
        # Convert → PIL image
        bmp = Image.fromarray(image)
    
        # Create ~/blob directory
        home = os.path.expanduser("~")
        out_dir = os.path.join(home, "blob")
        os.makedirs(out_dir, exist_ok=True)
    
        # Increment counter
        self.file_counter += 1
        filename = f"trajectory_object_{self.file_counter}.bmp"
    
        # Save inside ~/blob
        full_path = os.path.join(out_dir, filename)
        bmp.save(full_path)
    
        self.get_logger().info(f"Saved {full_path}")




def main(args=None):
    rclpy.init(args=args)
    node = PathToBitmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
