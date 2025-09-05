#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import socket
import threading
from struct import pack

from racecar_beacon.utils import euler_from_quaternion


class ROSMonitor(Node):
    def __init__(self):
        super().__init__("ros_monitor")

        # Robot state
        self.id = int(0x7F000001)
        self.position = tuple([float(0), float(0), float(0)])
        self.obstacle_detected = bool(False)

        # Socket parameters
        self.host = self.declare_parameter("host", "127.0.0.1").value
        self.remote_request_port = self.declare_parameter(
            "remote_request_port", 65432
        ).value
        self.broadcast = self.declare_parameter("broadcast", "127.0.0.255").value
        self.position_broad_port = self.declare_parameter(
            "pos_broadcast_port", 65431
        ).value

        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.create_timer(1.0, self.broadcast_callback)

        self.remote_request_t = threading.Thread(target=self.remote_request_loop)

        self.odom_sub = self.create_subscription(Odometry,
                                                 "/odometry/filtered",
                                                 self.odom_callback,
                                                 10) 
        
        self.laser_sub = self.create_subscription(LaserScan,
                                                  "/scan",
                                                  self.laser_callback,
                                                  10)

        self.remote_request_t.start()

        self.get_logger().info(f"{self.get_name()} started.")

    def odom_callback(self, msg : Odometry):
        
        x_position = msg.pose.pose.position.x
        y_position = msg.pose.pose.position.y
        
        # Extract orientation quaternion
        orientation = msg.pose.pose.orientation
        Quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]

        roll, pitch, yaw = euler_from_quaternion(Quaternion)

        self.position = (x_position, y_position, yaw)

        #self.get_logger().info(f"Position -> X: {x_position:.3f}, Y: {y_position:.3f}, Yaw: {yaw:.3f}")


    def laser_callback(self, msg : LaserScan):
        self.obstacle_detected = any(r < 1.0 for r in msg.ranges if r > 0.0)
        #self.get_logger().info(f"OBSF = {self.obstacle_detected}")

    def remote_request_loop(self):
        """TCP server handling RemoteRequest commands (RPOS, OBSF, RBID)."""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.remote_request_port))
        server_socket.listen(1)

        self.get_logger().info(f"RemoteRequest server listening on {self.host}:{self.remote_request_port}")

        while rclpy.ok():
            conn, addr = server_socket.accept()
            self.get_logger().info(f"RemoteRequest client connected: {addr}")
            with conn:
                try:
                    while True:
                        # Expect a 4-byte command
                        cmd = conn.recv(4)
                        if not cmd:
                            break
                        cmd = cmd.decode("utf-8")

                        if cmd == "RPOS":
                            x, y, yaw = self.position
                            data = pack("<fffI", x, y, yaw, 0)  # last 4 bytes unused
                        elif cmd == "OBSF":
                            val = 1 if self.obstacle_detected else 0
                            data = pack("<I12x", val)  # uint32 + 12 bytes padding
                        elif cmd == "RBID":
                            data = pack("<I12x", self.id)  # uint32 + 12 bytes padding
                        else:
                            self.get_logger().warn(f"Unknown command: {cmd}")
                            data = b"\x00" * 16

                        conn.sendall(data)

                except Exception as e:
                    self.get_logger().error(f"RemoteRequest error: {e}")
            self.get_logger().info("Client disconnected")


    # TODO: Implement the PositionBroadcast service here.
    # NOTE: It is recommended to initializae your socket locally.

    def broadcast_callback(self):
        x, y, yaw = self.position
        
        data = pack("<fffi", x, y, yaw, self.id)
        #self.get_logger().info(data)
        self.s.sendto(data, (self.broadcast, self.position_broad_port))


    def shutdown(self):
        """Gracefully shutdown the threads BEFORE terminating the node."""
        self.remote_request_t.join()
        


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ROSMonitor()
        rclpy.get_default_context().on_shutdown(node.shutdown)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
