#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        
        # Distance de détection
        self.front_limit = 0.75     # 75 cm
        self.back_limit  = 0.75     # 75 cm
        self.backup_distance = 1.5  # Distance de recul en mètres
        
        # Machine à états
        self.state = 'NORMAL'  # NORMAL, STOPPING, BACKING_UP, WAITING
        self.initial_position = 0.0
        self.backup_speed = -0.3  # Vitesse de recul (m/s)
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        
        # Timer pour estimer la distance parcourue
        self.last_time = self.get_clock().now()
        self.distance_traveled = 0.0
        
    def scan_callback(self, msg):
        n = len(msg.ranges)
        half = n // 2
        
        # Recentrer le LIDAR
        ranges = msg.ranges[half:] + msg.ranges[:half]
        
        # Définition des secteurs
        front_start = half - half//8
        front_end   = half + half//8
        back_start  = half//8
        back_end    = 3*half//8
        
        # --- Détection obstacle avant ---
        obstacle_front = False
        for i in range(front_start, front_end):
            d = ranges[i]
            if np.isfinite(d) and 0 < d < self.front_limit:
                obstacle_front = True
                break
        
        # --- Détection obstacle arrière ---
        obstacle_back = False
        for i in range(back_start, back_end):
            d = ranges[i]
            if np.isfinite(d) and 0 < d < self.back_limit:
                obstacle_back = True
                break
        
        # --- MACHINE À ÉTATS ---
        twist = Twist()
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        if self.state == 'NORMAL':
            if obstacle_front:
                # Obstacle détecté → passer en mode STOPPING
                self.state = 'STOPPING'
                twist.linear.x = 0.0
                self.distance_traveled = 0.0
                self.get_logger().info("⚠ Obstacle détecté à <0.75m → ARRÊT")
                self.cmd_vel_pub.publish(twist)
            else:
                # Aucun obstacle → ne pas publier (laisser l'autre nœud commander)
                return
                
        elif self.state == 'STOPPING':
            # Robot arrêté, passer au recul si l'arrière est libre
            twist.linear.x = 0.0
            if obstacle_back:
                self.get_logger().info("⚠ Obstacle arrière détecté → impossible de reculer")
                self.cmd_vel_pub.publish(twist)
            else:
                self.state = 'BACKING_UP'
                self.distance_traveled = 0.0
                self.get_logger().info("↩ Début du recul de 1.5m")
                
        elif self.state == 'BACKING_UP':
            # Calculer la distance parcourue
            self.distance_traveled += abs(self.backup_speed) * dt
            
            if obstacle_back:
                # Obstacle arrière détecté pendant le recul → arrêt
                twist.linear.x = 0.0
                self.get_logger().warn("⚠ Obstacle arrière pendant le recul → arrêt")
                self.state = 'WAITING'
            elif self.distance_traveled >= self.backup_distance:
                # Distance de recul atteinte → arrêt et attente
                twist.linear.x = 0.0
                self.state = 'WAITING'
                self.get_logger().info("✓ Recul de 1.5m terminé → ATTENTE")
            else:
                # Continuer le recul
                twist.linear.x = self.backup_speed
                self.get_logger().info(f"↩ Recul en cours: {self.distance_traveled:.2f}m / {self.backup_distance}m")
            
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == 'WAITING':
            # Attendre que l'obstacle avant disparaisse
            twist.linear.x = 0.0
            
            if not obstacle_front:
                # Obstacle disparu → retour à la normale
                self.state = 'NORMAL'
                self.get_logger().info("✓ Obstacle disparu → reprise normale")
                # Ne pas publier, laisser l'autre nœud reprendre le contrôle
                return
            else:
                self.get_logger().info("⏸ En attente que l'obstacle disparaisse...")
            
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()