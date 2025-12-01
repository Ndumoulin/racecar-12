#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        self.front_limit = 0.75
        self.safe_distance = 1.5
        self.back_limit = 0.75
        self.backup_distance = 1.5
        
        self.state = 'NORMAL'
        self.initial_position = 0.0
        self.backup_speed = -0.3
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        
        self.last_time = self.get_clock().now()
        self.distance_traveled = 0.0
        
    def scan_callback(self, msg):
        n = len(msg.ranges)
        half = n // 2
        
        ranges = msg.ranges[half:] + msg.ranges[:half]
        
        front_start = half - half//8
        front_end = half + half//8
        back_start = half//8
        back_end = 3*half//8
        
        obstacle_front = False
        min_front_distance = float('inf')
        for i in range(front_start, front_end):
            d = ranges[i]
            if np.isfinite(d) and d > 0:
                min_front_distance = min(min_front_distance, d)
                if d < self.front_limit:
                    obstacle_front = True
        
        obstacle_back = False
        for i in range(back_start, back_end):
            d = ranges[i]
            if np.isfinite(d) and 0 < d < self.back_limit:
                obstacle_back = True
                break
        
        twist = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.state == 'NORMAL':
            if obstacle_front:
                self.state = 'STOPPING'
                twist.linear.x = 0.0
                self.distance_traveled = 0.0
                self.get_logger().info("⚠ Obstacle détecté à <0.75m → ARRÊT")
                self.cmd_vel_pub.publish(twist)
            else:
                return
                
        elif self.state == 'STOPPING':
            twist.linear.x = 0.0
            if obstacle_back:
                self.get_logger().info("⚠ Obstacle arrière détecté → impossible de reculer")
                self.cmd_vel_pub.publish(twist)
            else:
                self.state = 'BACKING_UP'
                self.distance_traveled = 0.0
                self.get_logger().info("↩ Début du recul de 1.5m")
                
        elif self.state == 'BACKING_UP':
            self.distance_traveled += abs(self.backup_speed) * dt
            
            if obstacle_back:
                twist.linear.x = 0.0
                self.get_logger().warn("⚠ Obstacle arrière pendant le recul → arrêt")
                self.state = 'WAITING'
            elif self.distance_traveled >= self.backup_distance:
                twist.linear.x = 0.0
                self.state = 'WAITING'
                self.get_logger().info("✓ Recul de 1.5m terminé → ATTENTE")
            else:
                twist.linear.x = self.backup_speed
                self.get_logger().info(f"↩ Recul en cours: {self.distance_traveled:.2f}m / {self.backup_distance}m")
            
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'WAITING':
            twist.linear.x = 0.0
            
            if min_front_distance > self.safe_distance:
                self.state = 'NORMAL'
                self.get_logger().info(f"✓ Obstacle maintenant à {min_front_distance:.2f}m (>1.5m) → reprise normale")
                return
            else:
                self.get_logger().info(f"⏸ En attente: obstacle à {min_front_distance:.2f}m (doit être >1.5m)")
            
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()