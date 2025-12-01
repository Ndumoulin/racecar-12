#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan
from tf2_ros import Buffer, TransformListener

import heapq
import math

class AStarPlannerService(Node):
    def __init__(self):
        super().__init__("path_planner_service")

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

        # Create service
        self.srv = self.create_service(
            GetPlan,
            '/plan_path',
            self.plan_path_callback
        )

        self.get_logger().info("A* planner service initialized. Call '/plan_path' service to get a path.")

    # -------------------------
    #   Costmap updated
    # -------------------------
    def costmap_callback(self, msg):
        self.costmap = msg

    # -------------------------
    #   Service callback
    # -------------------------
    def plan_path_callback(self, request, response):
        self.get_logger().info("Received path planning request")

        if self.costmap is None:
            self.get_logger().error("No costmap available!")
            response.plan = Path()
            return response

        # Extract start and goal
        start_x = request.start.pose.position.x
        start_y = request.start.pose.position.y
        goal_x = request.goal.pose.position.x
        goal_y = request.goal.pose.position.y

        self.get_logger().info(f"Planning from ({start_x:.2f}, {start_y:.2f}) to ({goal_x:.2f}, {goal_y:.2f})")

        sx, sy = self.world_to_grid(start_x, start_y)
        gx, gy = self.world_to_grid(goal_x, goal_y)

        w, h = self.costmap.info.width, self.costmap.info.height
        if not (0 <= sx < w and 0 <= sy < h) or not (0 <= gx < w and 0 <= gy < h):
            self.get_logger().error("Start or goal OUTSIDE costmap!")
            response.plan = Path()
            return response

        # Run A* algorithm
        path_cells = self.a_star((sx, sy), (gx, gy))

        if path_cells is None:
            self.get_logger().warn("A* failed: no path found")
            response.plan = Path()
            return response

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

        response.plan = path_msg
        self.get_logger().info(f"Returning path with {len(path_msg.poses)} points")
        return response

    # -------------------------
    #   TF lookup
    # -------------------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "racecar/map",
                "racecar/base_footprint",
                rclpy.time.Time()
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            self.get_logger().warn(f"TF unavailable: {e}", throttle_duration_sec=2.0)
            return None

    # -------------------------
    #   World <-> Grid
    # -------------------------
    def world_to_grid(self, x, y):
        origin = self.costmap.info.origin.position
        res = self.costmap.info.resolution
        return int((x - origin.x) / res), int((y - origin.y) / res)

    def grid_to_world(self, gx, gy):
        origin = self.costmap.info.origin.position
        res = self.costmap.info.resolution
        return origin.x + gx * res, origin.y + gy * res

    # -------------------------
    #   Free cell check
    # -------------------------
    def is_free(self, gx, gy):
        w, h = self.costmap.info.width, self.costmap.info.height
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return False

        cost = self.costmap.data[gy * w + gx]
        if cost < 0 or cost >= 100:   # unknown or lethal
            return False
        return True

    # -------------------------
    #   Penalty for being near walls
    # -------------------------
    def inflation_penalty(self, gx, gy):
        """Prefer cells far from obstacles. Returns small extra cost."""
        w = self.costmap.info.width
        cost = self.costmap.data[gy * w + gx]

        if cost < 0 or cost >= 100:
            return 1000  # avoid unknown/lethal cells

        # Quadratic penalty (0 free → 0, 100 → big)
        return (cost / 100.0) ** 2 * 5.0

    # -------------------------
    #   A* Algorithm
    # -------------------------
    def a_star(self, start, goal):
        sx, sy = start
        gx, gy = goal

        if not self.is_free(sx, sy) or not self.is_free(gx, gy):
            self.get_logger().error("Start or goal not free!")
            return None

        open_set = []
        heapq.heappush(open_set, (0, (sx, sy)))
        gscore = {(sx, sy): 0}
        came_from = {}
        nodes_expanded = 0

        directions = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (1,-1), (-1,1), (-1,-1)]

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
                inflation = self.inflation_penalty(nx, ny)
                new_g = gscore[current] + step_cost + inflation

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

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
