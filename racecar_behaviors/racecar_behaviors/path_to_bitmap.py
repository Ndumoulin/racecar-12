#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from racecar_interfaces.srv import PathToBitmap
import numpy as np
import cv2
import os

class PathToBitmapNode(Node):
    def __init__(self):
        super().__init__('path_to_bitmap')
        self.srv = self.create_service(PathToBitmap, 'path_to_bitmap', self.callback)
        self.get_logger().info('PathToBitmap service ready.')

    def callback(self, request, response):
        try:
            # Folder setup
            report_dir = os.path.expanduser('~/debris_report')
            os.makedirs(report_dir, exist_ok=True)

            # Bitmap file path
            filepath = os.path.join(report_dir, "trajectory.bmp")

            # Create a simple bitmap of fixed size (adjust size as needed)
            width, height = 500, 500
            bmp = np.zeros((height, width), dtype=np.uint8)

            if hasattr(request, 'path') and len(request.path) > 0:
                for pose in request.path:
                    # Convert map coordinates to pixel coordinates (simple scaling)
                    x_px = int(pose.position.x * 50 + width // 2)
                    y_px = int(pose.position.y * 50 + height // 2)

                    if 0 <= x_px < width and 0 <= y_px < height:
                        bmp[y_px, x_px] = 255  # Mark path point

            # Save the bitmap
            cv2.imwrite(filepath, bmp)
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
