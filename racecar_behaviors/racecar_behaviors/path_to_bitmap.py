#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from racecar_interfaces.srv import PathToBitmap
import os
import numpy as np
from PIL import Image, ImageDraw

class PathToBitmapNode(Node):
    def __init__(self):
        super().__init__('path_to_bitmap')
        # Create the service
        self.srv = self.create_service(PathToBitmap, 'path_to_bitmap', self.callback)
        self.get_logger().info("Service 'path_to_bitmap' ready.")

    def callback(self, request, response):
        """
        Callback for the PathToBitmap service.
        request.path : list of geometry_msgs/Pose
        response.success : bool
        response.filepath : string
        """
        try:
            poses = request.path
            if not poses:
                response.success = False
                response.filepath = ""
                self.get_logger().warn("Received empty path.")
                return response

            # --- Create a simple bitmap image ---
            # Compute bounds
            xs = [p.position.x for p in poses]
            ys = [p.position.y for p in poses]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)

            margin = 1.0  # meters margin
            scale = 50  # pixels per meter
            width = int((max_x - min_x + 2*margin) * scale)
            height = int((max_y - min_y + 2*margin) * scale)

            # Create blank image
            img = Image.new('L', (width, height), 255)  # white background
            draw = ImageDraw.Draw(img)

            # Draw path points
            for pose in poses:
                px = int((pose.position.x - min_x + margin) * scale)
                py = int((pose.position.y - min_y + margin) * scale)
                # Invert y-axis for image coordinates
                py = height - py
                draw.ellipse((px-2, py-2, px+2, py+2), fill=0)  # black dot

            # Save bitmap
            filepath = os.path.expanduser("~/debris_path.bmp")
            img.save(filepath)

            response.success = True
            response.filepath = filepath
            self.get_logger().info(f"Bitmap saved at {filepath}")
            return response

        except Exception as e:
            self.get_logger().error(f"PathToBitmap callback error: {e}")
            response.success = False
            response.filepath = ""
            return response

def main(args=None):
    rclpy.init(args=args)
    node = PathToBitmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
