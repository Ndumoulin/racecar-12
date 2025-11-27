#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
import numpy as np
import math

class PathFollowing(Node):
    def __init__(self):
        super().__init__('path_following')
        
        # Parameters
        self.angle_div = self.declare_parameter('angle_div', 8).value
        self.distance = self.declare_parameter('distance', 0.7).value
        self.distance_short = self.declare_parameter('distance_short', 0.45).value
        self.max_speed = self.declare_parameter('max_speed', 0.5).value
        self.max_steering = self.declare_parameter('max_steering', 0.37).value
        
        # Pure Pursuit parameters
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 1.5).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.3).value
        self.wheelbase = self.declare_parameter('wheelbase', 0.32).value  # Distance entre essieux
        
        # State
        self.current_path = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.scan_data = None
        self.step = 1  # 1: going to goal, 2: uturn, 3: go to 0,0
        
        # U-turn state machine
        self.uturn_stage = 0  # 0: backward+right, 1: left+straight
        self.uturn_start_time = None
        self.uturn_backward_duration = 5  # seconds to go backward
        self.uturn_forward_duration = 4  # seconds to go forward
        self.uturn_speed = 0.3
        self.uturn_steering = 0.5
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/racecar/cmd_vel', 1)
        self.goal_pub = self.create_publisher(Float32MultiArray, '/goal_coordinates', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/racecar/odom', self.odom_callback, 1)
        self.path_sub = self.create_subscription(Path, '/a_star_path', self.path_callback, 1)
        
        # Control timer
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('Path Following initialized with Pure Pursuit controller')
    
    def path_callback(self, msg):
        if self.step == 1:
            self.step = None
            self.current_path = msg
            self.get_logger().info(f'Received new path with {len(msg.poses)} points')
        elif self.step == 3:
            self.step = None
            self.current_path = msg
            self.get_logger().info(f'Received return path to (0,0) with {len(msg.poses)} points')
    
    def scan_callback(self, msg):
        """Store laser scan data for obstacle detection"""
        self.scan_data = msg
    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_pose = msg.pose.pose.position
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def get_lookahead_point(self):
        """Find the lookahead point on the path using Pure Pursuit"""
        if self.current_path is None or self.current_pose is None:
            return None
        
        path_poses = self.current_path.poses
        if len(path_poses) == 0:
            return None
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(path_poses):
            dx = pose.pose.position.x - self.current_pose.x
            dy = pose.pose.position.y - self.current_pose.y
            dist = math.hypot(dx, dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Check if we reached the goal
        goal_pose = path_poses[-1].pose.position
        dist_to_goal = math.hypot(
            goal_pose.x - self.current_pose.x,
            goal_pose.y - self.current_pose.y
        )
        
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.current_path = None
            self.step = 2
            self.do_uturn()
            return None
        
        # Search for lookahead point starting from closest point
        for i in range(closest_idx, len(path_poses)):
            pose = path_poses[i].pose.position
            dx = pose.x - self.current_pose.x
            dy = pose.y - self.current_pose.y
            dist = math.hypot(dx, dy)
            
            if dist >= self.lookahead_distance:
                return (pose.x, pose.y)
        
        # If no point at lookahead distance, return last point
        last_pose = path_poses[-1].pose.position
        return (last_pose.x, last_pose.y)
    
    def compute_pure_pursuit_steering(self, lookahead_point):
        """Compute steering angle using Pure Pursuit algorithm"""
        if lookahead_point is None or self.current_pose is None:
            return 0.0
        
        # Transform lookahead point to robot frame
        dx = lookahead_point[0] - self.current_pose.x
        dy = lookahead_point[1] - self.current_pose.y
        
        # Rotate to robot's local frame
        dx_robot = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
        dy_robot = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
        
        # Pure Pursuit formula
        ld = math.hypot(dx_robot, dy_robot)
        
        if ld < 0.1:  # Avoid division by zero
            return 0.0
        
        # Curvature = 2 * lateral_offset / lookahead_distance^2
        curvature = 2.0 * dy_robot / (ld * ld)
        
        # Steering angle (Ackermann steering)
        steering_angle = math.atan(curvature * self.wheelbase)
        
        # Clamp steering
        steering_angle = np.clip(steering_angle, -self.max_steering, self.max_steering)
        
        return steering_angle

    def do_uturn(self):
        """Start U-turn maneuver"""
        self.uturn_stage = 0
        self.uturn_start_time = self.get_clock().now()
        self.get_logger().info('Starting U-turn maneuver: backward with right steering')

    def send_goal_to_origin(self):
        """Send goal to origin (0, 0) to request path back"""
        goal_msg = Float32MultiArray()
        goal_msg.data = [0.0, 0.0]
        self.goal_pub.publish(goal_msg)
        self.get_logger().info('Requesting path to origin (0, 0)')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    def check_obstacle_ahead(self):
        """Check for obstacles in front using laser scan"""
        if self.scan_data is None:
            return False
        
        ranges = np.array(self.scan_data.ranges)
        
        # Check front cone (±30 degrees)
        num_ranges = len(ranges)
        front_angle = 30  # degrees
        front_indices = int(num_ranges * front_angle / 360)
        
        # Front = beginning and end of array (lidar oriented backward)
        front_ranges = np.concatenate([
            ranges[:front_indices],
            ranges[-front_indices:]
        ])
        
        # Filter out invalid readings
        valid_ranges = front_ranges[(front_ranges > 0.1) & (front_ranges < 10.0)]
        
        if len(valid_ranges) == 0:
            return False
        
        min_distance = np.min(valid_ranges)
        
        # Obstacle too close
        if min_distance < self.distance_short:
            return True
        
        return False
    
    def control_loop(self):
        """Main control loop - called at 20 Hz"""
        twist = Twist()
        
        # Handle U-turn if in progress (step 2)
        if self.step == 2 and self.uturn_start_time is not None:
            elapsed = (self.get_clock().now() - self.uturn_start_time).nanoseconds / 1e9
            
            if self.uturn_stage == 0:
                # Stage 0: Go backward with right steering
                if elapsed < self.uturn_backward_duration:
                    twist.linear.x = -self.uturn_speed  # Backward
                    twist.angular.z = self.uturn_steering  # Right steering
                    self.get_logger().info(f'U-turn stage 0: backward+right ({elapsed:.2f}s)', throttle_duration_sec=0.5)
                else:
                    # Switch to stage 1
                    self.uturn_stage = 1
                    self.uturn_start_time = self.get_clock().now()
                    self.get_logger().info('U-turn switching to stage 1: left steering+forward')
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            elif self.uturn_stage == 1:
                # Stage 1: Go forward with left steering for 10 seconds
                if elapsed < self.uturn_forward_duration:
                    twist.linear.x = self.uturn_speed  # Forward
                    twist.angular.z = self.uturn_steering  # Left steering
                    self.get_logger().info(f'U-turn stage 1: forward+left ({elapsed:.2f}s)', throttle_duration_sec=0.5)
                else:
                    # U-turn complete
                    self.step = 3  # Move to next step
                    self.uturn_start_time = None
                    self.get_logger().info('U-turn completed!')
                    self.send_goal_to_origin()  # Request path to (0, 0)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
            
            self.cmd_vel_pub.publish(twist)
            return
        
        # No path available
        if self.current_path is None:
            self.get_logger().warn('No path available, stopping', throttle_duration_sec=2.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # No pose available
        if self.current_pose is None:
            self.get_logger().warn('No pose available, stopping', throttle_duration_sec=2.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # Get lookahead point
        lookahead_point = self.get_lookahead_point()
        
        if lookahead_point is None:
            # Goal reached or no valid point
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return
        
        # Compute steering
        steering = self.compute_pure_pursuit_steering(lookahead_point)
        
        # Check for obstacles
        obstacle_detected = self.check_obstacle_ahead()
        
        if obstacle_detected:
            self.get_logger().warn('Obstacle detected! Stopping', throttle_duration_sec=1.0)
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # Adaptive speed based on steering angle
            speed_factor = 1.0 - abs(steering) / self.max_steering
            speed = self.max_speed * max(0.3, speed_factor)  # Min 30% speed
            
            twist.linear.x = float(speed)
            twist.angular.z = float(steering)
            
            self.get_logger().info(
                f'Speed: {speed:.2f} m/s, Steering: {steering:.2f} rad',
                throttle_duration_sec=1.0
            )
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    path_following = PathFollowing()
    rclpy.spin(path_following)
    path_following.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()