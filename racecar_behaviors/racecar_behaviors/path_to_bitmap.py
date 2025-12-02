#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from racecar_interfaces.srv import PathToBitmap
import numpy as np
from nav_msgs.srv import GetMap
from PIL import Image
import os
from datetime import datetime

class PathToBitmapNode(Node):
    def __init__(self):
        super().__init__('path_to_bitmap')
        self.srv = self.create_service(PathToBitmap, 'path_to_bitmap', self.callback)
        self.get_logger().info('PathToBitmap service ready.')

        # RTAB-Map service client
        self.prefix = "rtabmap"
        self.get_map_client = self.create_client(GetMap, self.prefix + '/get_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Map service not available, waiting...')

        self.latest_map = None

        # Fetch map once at startup
        self.update_map()

    def update_map(self):
        request = GetMap.Request()
        future = self.get_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            h = response.map.info.height
            w = response.map.info.width
            grid = np.array(response.map.data, dtype=int).reshape((h, w))
            self.latest_map = {
                "grid": grid,
                "info": response.map.info
            }
            self.get_logger().info("Map updated successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to get map: {e}")
            self.latest_map = None

    def callback(self, request, response):
        try:
            if self.latest_map is None:
                self.get_logger().error("No map available, cannot generate bitmap.")
                response.success = False
                response.filepath = ''
                return response

            # Folder setup
            report_dir = os.path.expanduser('~/debris_report')
            os.makedirs(report_dir, exist_ok=True)

            # Unique file name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(report_dir, f"trajectory_{timestamp}.bmp")

            grid = self.latest_map["grid"]
            h, w = grid.shape
            image = np.zeros((h, w, 3), dtype=np.uint8)

            # Map colors
            image[grid == -1] = [160, 160, 160]  # unknown
            image[grid >= 50] = [0, 0, 0]       # occupied
            image[grid >= 0]  = [255, 255, 255] # free

            # Draw path if provided
            if hasattr(request, 'path') and len(request.path) > 0:
                for pose in request.path:
                    res = self.latest_map["info"].resolution
                    ox = self.latest_map["info"].origin.position.x
                    oy = self.latest_map["info"].origin.position.y

                    gx = int((pose.position.x - ox) / res)
                    gy = int((pose.position.y - oy) / res)

                    if 0 <= gx < w and 0 <= gy < h:
                        image[gy, gx] = [255, 0, 0]  # red path

                # Highlight last point (goal)
                gx, gy = int((request.path[-1].position.x - ox) / res), int((request.path[-1].position.y - oy) / res)
                if 0 <= gx < w and 0 <= gy < h:
                    image[gy, gx] = [255, 0, 0]

            # Convert to PIL and save
            bmp = Image.fromarray(image)
            bmp.save(filepath)

            self.get_logger().info(f"Bitmap saved at: {filepath}")
            response.success = True
            response.filepath = filepath

        except Exception as e:
            self.get_logger().error(f"Failed to create bitmap: {e}")
            response.success = False
            response.filepath = ''

        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathToBitmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
