#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
 
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener
 
import heapq
import math
 
class AStarPlanner(Node):
    def __init__(self):
        super().__init__("path_planner")
 
        # Goal coordinates (can be updated via subscriber)
        self.goal_x = 13.5
        self.goal_y = 2.1
        self.goal_updated = False
 
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
 
        # Costmap subscriber
        self.costmap = None
        self.create_subscription(
            OccupancyGrid,
            "/my_costmap",
            self.costmap_callback,
            10
        )
        
        # Goal subscriber - receives [x, y] coordinates
        self.create_subscription(
            Float32MultiArray,
            "/goal_coordinates",
            self.goal_callback,
            10
        )
 
        self.path_pub = self.create_publisher(Path, "/a_star_path", 10)
 
        self.timer = self.create_timer(1.0, self.plan_timer)
 
        self.get_logger().info("A* planner initialized. Listening for goal coordinates on /goal_coordinates topic.")
       
        self.max_safe_cost = 75
       
        # Store last valid path to avoid publishing empty paths
        self.last_valid_path = None
 
    # -------------------------
    #   Costmap updated
    # -------------------------
    def costmap_callback(self, msg):
        self.costmap = msg
    
    # -------------------------
    #   Goal coordinates received
    # -------------------------
    def goal_callback(self, msg):
        """Receive new goal coordinates [x, y]"""
        if len(msg.data) >= 2:
            self.goal_x = msg.data[0]
            self.goal_y = msg.data[1]
            self.goal_updated = True
            self.get_logger().info(f"Received new goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")
        else:
            self.get_logger().warn("Goal message must contain at least [x, y]")
    
    # -------------------------
    #   TF lookup - FIXED TO USE MAP FRAME
    # -------------------------
    def get_robot_pose(self):
        try:
            # CRITICAL FIX: Get robot pose IN the map frame
            tf = self.tf_buffer.lookup_transform(
                "racecar/map",  # Target frame (where costmap lives)
                "racecar/base_footprint",  # Source frame (robot)
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"TF unavailable: {e}", throttle_duration_sec=2.0)
            return None
 
    # -------------------------
    #   World <-> Grid
    # -------------------------
    def world_to_grid(self, x, y):
        origin = self.costmap.info.origin.position
        res = self.costmap.info.resolution
        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)
        return gx, gy
 
    def grid_to_world(self, gx, gy):
        origin = self.costmap.info.origin.position
        res = self.costmap.info.resolution
        return origin.x + gx * res, origin.y + gy * res
 
    # -------------------------
    #   Free cell check
    # -------------------------
    def is_free(self, gx, gy):
        w = self.costmap.info.width
        h = self.costmap.info.height
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False
 
        cost = self.costmap.data[gy * w + gx]
 
        if cost < 0:         # unknown space
            return False
        if cost >= 100:      # lethal obstacle
            return False
        if cost > self.max_safe_cost:   # too close to obstacle
            return False
 
        return True
 
    # -------------------------
    #   A* Algorithm
    # -------------------------
    def a_star(self, start, goal):
        sx, sy = start
        gx, gy = goal
       
        # Validate start and goal
        if not self.is_free(sx, sy):
            self.get_logger().error(f"Start cell ({sx}, {sy}) is not free!")
            return None
        if not self.is_free(gx, gy):
            self.get_logger().error(f"Goal cell ({gx}, {gy}) is not free!")
            return None
 
        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))
 
        gscore = { (sx, sy): 0 }
        came_from = {}
       
        nodes_expanded = 0
 
        directions = [
            (1,0), (-1,0), (0,1), (0,-1),
            (1,1), (1,-1), (-1,1), (-1,-1)
        ]
 
        while open_set:
            _, current = heapq.heappop(open_set)
            cx, cy = current
            nodes_expanded += 1
 
            if current == (gx, gy):
                self.get_logger().info(f"Path found! Expanded {nodes_expanded} nodes.")
                return self.reconstruct_path(came_from, current)
 
            for dx, dy in directions:
                nx, ny = cx + dx, cy + dy
                if not self.is_free(nx, ny):
                    continue
 
                step_cost = math.hypot(dx, dy)
                new_g = gscore[current] + step_cost
 
                if (nx, ny) not in gscore or new_g < gscore[(nx, ny)]:
                    gscore[(nx, ny)] = new_g
                    f = new_g + math.hypot(gx - nx, gy - ny)
                    heapq.heappush(open_set, (f, (nx, ny)))
                    came_from[(nx, ny)] = current
 
        self.get_logger().error(f"No path found. Expanded {nodes_expanded} nodes.")
        return None
 
    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return list(reversed(path))
 
    # -------------------------
    #   Timer callback
    # -------------------------
    def plan_timer(self):
        if self.costmap is None:
            self.get_logger().warn("No costmap received yet.")
            return
 
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return
 
        rx, ry = robot_pose
        sx, sy = self.world_to_grid(rx, ry)
        gx, gy = self.world_to_grid(self.goal_x, self.goal_y)
       
        self.get_logger().info(f"Robot world pose: ({rx:.2f}, {ry:.2f}), grid: ({sx}, {sy})", throttle_duration_sec=2.0)
        self.get_logger().info(f"Goal world pose: ({self.goal_x:.2f}, {self.goal_y:.2f}), grid: ({gx}, {gy})", throttle_duration_sec=2.0)
 
        # Check bounds
        w = self.costmap.info.width
        h = self.costmap.info.height
 
        if not (0 <= sx < w and 0 <= sy < h):
            self.get_logger().error("Start pose is OUTSIDE the costmap!")
            return
 
        if not (0 <= gx < w and 0 <= gy < h):
            self.get_logger().error("Goal pose is OUTSIDE the costmap!")
            return
 
        # Check occupancy
        index_s = sy * w + sx
        index_g = gy * w + gx
        self.get_logger().info(f"Start cost: {self.costmap.data[index_s]}", throttle_duration_sec=2.0)
        self.get_logger().info(f"Goal cost: {self.costmap.data[index_g]}", throttle_duration_sec=2.0)
 
        path_cells = self.a_star((sx, sy), (gx, gy))
       
        if path_cells is None:
            self.get_logger().warn("A* failed: no path found. Keeping last valid path.")
            # DON'T publish - keep the robot following the last valid path
            return
 
        # Build path message
        path_msg = Path()
        path_msg.header.frame_id = "racecar/map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
 
        for (cx, cy) in path_cells:
            wx, wy = self.grid_to_world(cx, cy)
            pose = PoseStamped()
            pose.header.frame_id = "racecar/map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
 
        # Only publish if path is valid
        if len(path_msg.poses) > 0:
            self.path_pub.publish(path_msg)
            self.last_valid_path = path_msg
            self.get_logger().info(f"Published A* path with {len(path_msg.poses)} points.")
        else:
            self.get_logger().warn("Generated empty path, not publishing.")
 
def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()