#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from racecar_behaviors.libbehaviors import brushfire

class Brushfire(Node):
    def __init__(self):
        super().__init__('brushfire')
        self.prefix = "rtabmap"
        self.get_map_client = self.create_client(GetMap, self.prefix + '/get_map')
        while not self.get_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.costmap_pub = self.create_publisher(OccupancyGrid, '/my_costmap', 10)
        
        # Timer: call self.timer_callback() every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        request = GetMap.Request()
        future = self.get_map_client.call_async(request)
        future.add_done_callback(self.get_map_callback)

    def get_map_callback(self, future):
        response = future.result()
        self.get_logger().info("Got map=%dx%d resolution=%f" %(
            response.map.info.height, response.map.info.width, response.map.info.resolution))

        grid = np.reshape(response.map.data, [response.map.info.height, response.map.info.width])
        brushfire_map = brushfire(grid)
        self.publish_costmap(brushfire_map, response.map.info)

    def publish_costmap(self, grid, map_info):
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = '/racecar/map'
            msg.info = map_info
    
            occupancy = np.zeros_like(grid, dtype=int)
    
            # Keep unknowns as -1
            occupancy[grid < 0] = -1
    
            # Only scale valid brushfire values
            valid_cells = grid >= 0
            if np.any(valid_cells):
                max_val = grid[valid_cells].max()
                scaled = (grid[valid_cells] / max_val * 100).astype(int)
                # Scale to 0-100
                occupancy[valid_cells] = 100 - scaled
    
            msg.data = occupancy.flatten(order='C').tolist()
            self.costmap_pub.publish(msg)
            self.get_logger().info("Published costmap with gradients")


def main(args=None):
    rclpy.init(args=args)
    brushfire_node = Brushfire()
    rclpy.spin(brushfire_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
