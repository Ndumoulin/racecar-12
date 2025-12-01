#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from racecar_interfaces.srv import PathToBitmap

from PIL import Image
import os
import re

class PathToBitmapNode(Node):
    def __init__(self):
        super().__init__('path_to_bitmap')

        self.srv = self.create_service(
            PathToBitmap,
            'path_to_bitmap',
            self.callback
        )

        self.report_dir = os.path.expanduser('~/debris_report')
        os.makedirs(self.report_dir, exist_ok=True)

        self.get_logger().info("Bitmap generator ready.")

    def extract_photo_id(self, photo_filename):
        """
        Extract number from debris_xxx.jpg or debris_xxx.jpeg.
        Returns string like '017'. If extraction fails, return '000'.
        """
        m = re.search(r'(\d+)', photo_filename)
        if m:
            return m.group(1)
        return "000"

    def callback(self, request, response):
        path = request.path
        photo_filename = request.photo_filename

        if len(path) == 0:
            self.get_logger().error("Empty path!")
            response.success = False
            return response

        # Extract ID from filename
        photo_id = self.extract_photo_id(photo_filename)
        bmp_name = f"trajectory_{photo_id}.bmp"
        bmp_path = os.path.join(self.report_dir, bmp_name)

        # -----------------------------
        # Convert path to pixel coords
        # -----------------------------
        coords = [(int(p.position.x), int(p.position.y)) for p in path]

        xs = [c[0] for c in coords]
        ys = [c[1] for c in coords]

        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        width  = max_x - min_x + 1
        height = max_y - min_y + 1

        # -----------------------------
        # Create BMP
        # -----------------------------
        img = Image.new("RGB", (width, height), color=(0,0,0))
        pixels = img.load()

        # Draw path as white pixels
        for (x, y) in coords:
            px = x - min_x
            py = y - min_y
            pixels[px, py] = (0, 0, 255)

        # Save file
        img.save(bmp_path)
        self.get_logger().info(f"Bitmap saved at: {bmp_path}")

        response.success = True
        response.filepath = bmp_path
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathToBitmapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
