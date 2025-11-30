#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from PIL import Image
from nav_msgs.srv import GetMap

class PathToBitmap(Node):
    def __init__(self):
        super().__init__('path_to_bitmap')

        self.prefix = "rtabmap"   # FIXED
        self.latest_map = None
        self.latest_path_pixels = []

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

        # Try generating bitmap
        self.render_bitmap()


    def render_bitmap(self):
        """Render map + path to BMP file only when both map and path exist."""
        if self.latest_map is None:
            return
        if self.latest_path_pixels is None or len(self.latest_path_pixels) == 0:
            return

        grid = self.latest_map["grid"]
        info = self.latest_map["info"]

        height, width = grid.shape

        # Create empty RGB image
        image = np.zeros((height, width, 3), dtype=np.uint8)

        # ---- OCCUPANCY COLORS ----
        image[grid == -1] = [160, 160, 160]    # unknown
        image[grid >= 50] = [0, 0, 0]          # obstacle
        image[grid >= 0] = [255, 255, 255]     # free (0–49)

        # ---- DRAW RED PATH ----
        for (gx, gy) in self.latest_path_pixels:
            if 0 <= gx < width and 0 <= gy < height:
                image[gy, gx] = [255, 0, 0]

        # ---- DRAW RED GOAL DOT ----
        gx, gy = self.latest_path_pixels[-1]
        if 0 <= gx < width and 0 <= gy < height:
            image[gy, gx] = [255, 0, 0]

        # Convert array → image
        bmp = Image.fromarray(image)

        # Save BMP
        filename = "map_with_path.bmp"
        bmp.save(filename)
        self.get_logger().info(f"Bitmap saved: {filename}")



def main(args=None):
    rclpy.init(args=args)
    node = PathToBitmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
