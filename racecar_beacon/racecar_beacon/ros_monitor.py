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

        self.s_UDP = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.s_UDP.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        self.s_TCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s_TCP.bind((self.host, self.remote_request_port))

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
        self.s_TCP.listen()  # le socket serveur est prêt à écouter
        print("Serveur RemoteRequest en écoute...")

        while rclpy.ok():
            conn, addr = self.s_TCP.accept()
            print(f"Connexion acceptée de {addr}")

            with conn:
                while True:
                    msg = conn.recv(1024)
                    if not msg:
                        break  # Client closed connected

                    command = msg.decode().strip()
                    print(f"Commande reçue: {command}")

                    if command == "RPOS":
                        response = pack("!fff", 1.23, 4.56, 0.78)
                        conn.sendall(response)
                    else:
                        # Réponse texte générique
                        conn.sendall(f"Commande inconnue: {command}".encode())


    def broadcast_callback(self):
        x, y, yaw = self.position
        
        data = pack("<fffi", x, y, yaw, self.id)
        #self.get_logger().info(data)
        self.s_UDP.sendto(data, (self.broadcast, self.position_broad_port))


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
