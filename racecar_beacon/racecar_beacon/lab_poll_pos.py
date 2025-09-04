#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from racecar_beacon.utils import yaw_from_quaternion


class PositionPoller(Node):
    def __init__(self):
        super().__init__("position_poller")

        self.odom_sub = self.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10) 
        self.get_logger().info(f"{self.get_name()} started.")
        

    def odom_callback(self, msg: Odometry) -> None:
        
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        
        # Extract orientation quaternion
        orientation = msg.pose.pose.orientation
        Quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        roll, pitch, yaw = euler_from_quaternion(Quaternion)

        print(f"Position -> X: {x_position:.3f}, Y: {y_position:.3f}, Yaw: {yaw:.3f}")


    
        
def main(args=None):
    try:
        rclpy.init(args=args)
        node = PositionPoller()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()
