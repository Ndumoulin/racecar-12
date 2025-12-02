#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Detection distance
        self.front_limit = 0.75     # 75 cm
        self.back_limit  = 0.75     # 75 cm
        self.backup_distance = 1.5  # Backup distance in meters
        
        # State machine
        self.state = 'NORMAL'  # NORMAL, STOPPING, BACKING_UP, WAITING
        self.initial_position = 0.0
        self.backup_speed = -0.3  # Backup speed (m/s)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        
        # Timer to estimate distance traveled
        self.last_time = self.get_clock().now()
        self.distance_traveled = 0.0
        
    def scan_callback(self, msg):
        n = len(msg.ranges)
        half = n // 2
        
        # Recenter LIDAR
        ranges = msg.ranges[half:] + msg.ranges[:half]
        
        # Sector definition
        front_start = half - half//8
        front_end   = half + half//8
        back_start  = half//8
        back_end    = 3*half//8
        
        # --- Front obstacle detection ---
        obstacle_front = False
        for i in range(front_start, front_end):
            d = ranges[i]
            if np.isfinite(d) and 0 < d < self.front_limit:
                obstacle_front = True
                break
        
        # --- Back obstacle detection ---
        obstacle_back = False
        for i in range(back_start, back_end):
            d = ranges[i]
            if np.isfinite(d) and 0 < d < self.back_limit:
                obstacle_back = True
                break
        
        # --- STATE MACHINE ---
        twist = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.state == 'NORMAL':
            if obstacle_front:
                # Obstacle detected -> switch to STOPPING
                self.state = 'STOPPING'
                twist.linear.x = 0.0
                self.distance_traveled = 0.0
                self.get_logger().info("Obstacle detected at <0.75m")
                self.cmd_vel_pub.publish(twist)
            else:
                # No obstacle -> do not publish (let other node control)
                return
                
        elif self.state == 'STOPPING':
            # Robot stopped, switch to backup if back is free
            twist.linear.x = 0.0
            if obstacle_back:
                self.get_logger().info("Back obstacle detected")
                self.cmd_vel_pub.publish(twist)
            else:
                self.state = 'BACKING_UP'
                self.distance_traveled = 0.0
                self.get_logger().info("Starting 1.5m backup")
                
        elif self.state == 'BACKING_UP':
            # Calculate distance traveled
            self.distance_traveled += abs(self.backup_speed) * dt
            
            if obstacle_back:
                # Back obstacle detected during backup
                twist.linear.x = 0.0
                self.get_logger().warn("Back obstacle during backup")
                self.state = 'WAITING'
            elif self.distance_traveled >= self.backup_distance:
                # Backup distance reached
                twist.linear.x = 0.0
                self.state = 'WAITING'
                self.get_logger().info("1.5m backup completed")
            else:
                # Continue backup
                twist.linear.x = self.backup_speed
            
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'WAITING':
            # Wait for front obstacle to disappear
            twist.linear.x = 0.0
            
            if not obstacle_front:
                # Obstacle disappeared -> return to normal
                self.state = 'NORMAL'
                self.get_logger().info("Obstacle disappeared")
                # Do not publish, let other node resume control
                return
            else:
                self.get_logger().info("Waiting for obstacle to disappear")
            
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()