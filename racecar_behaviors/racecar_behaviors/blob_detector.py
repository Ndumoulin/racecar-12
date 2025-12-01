#!/usr/bin/env python3

# Band-Aid to be able to use `ros2 launch`
import sys
if "/usr/local/lib/python3.12/dist-packages" in sys.path:
    sys.path.remove("/usr/local/lib/python3.12/dist-packages")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, TransformStamped
import message_filters

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf_transformations
from visualization_msgs.msg import Marker
from racecar_behaviors.libbehaviors import *
import os
from datetime import datetime

class BlobDetector(Node):
    def __init__(self):
        super().__init__('blob_detector')
        self.bridge = CvBridge()

        # params
        self.map_frame_id = self.declare_parameter('map_frame_id', 'map').value
        self.frame_id = self.declare_parameter('frame_id', 'base_link').value
        self.object_frame_id = self.declare_parameter('object_frame_id', 'object').value
        self.color_hue = self.declare_parameter('color_hue', 100).value
        self.color_range = self.declare_parameter('color_range', 20).value
        self.color_saturation = self.declare_parameter('color_saturation', 100).value
        self.color_value = self.declare_parameter('color_value', 1).value
        self.border = self.declare_parameter('border', 1).value

        # État de détection des débris
        self.detected_debris_positions = []  # Liste de (x, y) dans le référentiel map
        self.stop_until = None
        self.photo_taken = False
        self.current_debris_position = None
        self.replan_requested = False
        
        # Créer un dossier pour sauvegarder les photos
        self.photo_dir = os.path.expanduser("~/debris_photos")
        os.makedirs(self.photo_dir, exist_ok=True)
        
        # Service client pour replan_path
        self.replan_client = self.create_client(Trigger, '/replan_path')
        self.get_logger().info("En attente du service /replan_path...")

        # blob detector params
        params = cv2.SimpleBlobDetector_Params()
        params.thresholdStep = 10
        params.minThreshold = 150
        params.maxThreshold = 220
        params.minRepeatability = 2
        params.minDistBetweenBlobs = 10
        params.filterByColor = False
        params.blobColor = 255
        params.filterByArea = True
        params.minArea = 100
        params.maxArea = 5000000000
        params.filterByCircularity = True
        params.minCircularity = 0.3
        params.filterByConvexity = False
        params.minConvexity = 0.2
        params.filterByInertia = False
        params.minInertiaRatio = 0.001

        self.detector = cv2.SimpleBlobDetector_create(params)

        # TF
        self.br = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # pubs / subs
        qos = QoSProfile(depth=10)
        self.image_pub = self.create_publisher(Image, 'image_detections', qos)
        self.object_pub = self.create_publisher(String, 'object_detected', qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        # synchronized subscribers (image, depth, camera_info)
        self.image_sub = message_filters.Subscriber(self, Image, 'image')
        self.depth_sub = message_filters.Subscriber(self, Image, 'depth')
        self.info_sub = message_filters.Subscriber(self, CameraInfo, 'camera_info')
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub, self.info_sub], 10)
        self.ts.registerCallback(self.image_callback)

        self.timer = self.create_timer(1.0, self.param_callback)

    def param_callback(self):
        # update params if changed dynamically
        try:
            self.color_hue = self.get_parameter('color_hue').get_parameter_value().integer_value
            self.color_range = self.get_parameter('color_range').get_parameter_value().integer_value
            self.color_saturation = self.get_parameter('color_saturation').get_parameter_value().integer_value
            self.color_value = self.get_parameter('color_value').get_parameter_value().integer_value
            self.border = self.get_parameter('border').get_parameter_value().integer_value
            self.get_logger().debug(f"param values: {self.color_hue}, {self.color_range}, {self.color_saturation}, {self.color_value}, {self.border}")
        except Exception:
            pass

    def is_debris_already_detected(self, position_map):
        """Vérifie si un débris à cette position a déjà été détecté (< 1m)"""
        for prev_pos in self.detected_debris_positions:
            distance = np.sqrt((position_map[0] - prev_pos[0])**2 + 
                             (position_map[1] - prev_pos[1])**2)
            if distance < 1.0:  # Moins de 1 mètre
                return True
        return False

    def save_photo(self, cv_image, debris_id):
        """Sauvegarde une photo du débris"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.photo_dir, f"debris_{debris_id}_{timestamp}.jpg")
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f"Photo sauvegardée: {filename}")
        return filename

    def call_replan_service(self):
        """Appelle le service /replan_path de manière asynchrone"""
        if not self.replan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Service /replan_path non disponible")
            self.replan_requested = True  # Marquer comme fait même si échec
            return
        
        request = Trigger.Request()
        future = self.replan_client.call_async(request)
        future.add_done_callback(self.replan_callback)
        self.get_logger().info("Demande de replanification du chemin envoyée...")
    
    def replan_callback(self, future):
        """Callback appelé quand le service /replan_path répond"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Replanification réussie: {response.message}")
            else:
                self.get_logger().warn(f"Replanification échouée: {response.message}")
            self.replan_requested = True
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la replanification: {e}")
            self.replan_requested = True  # Continuer quand même

    def image_callback(self, image_msg, depth_msg, info_msg):
        """Main callback: detect blobs in RGB, validate with depth and control robot."""
        # convert color image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV bridge error (image): {e}")
            return

        # convert depth image
        try:
            if depth_msg.encoding == '32FC1' or depth_msg.encoding == '32FC3':
                cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
            elif depth_msg.encoding == '16UC1':
                cv_depth_u16 = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
                cv_depth = cv_depth_u16.astype(np.float32)
                cv_depth /= 1000.0
            else:
                cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().warn(f"Failed to convert depth image: {e}")
            return

        # camera intrinsics
        try:
            K = np.array(info_msg.k).reshape((3, 3))
            fx = K[0, 0]
            fy = K[1, 1]
            cx = K[0, 2]
            cy = K[1, 2]
        except Exception as e:
            self.get_logger().warn(f"CameraInfo invalid: {e}")
            return

        # create HSV mask
        try:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        except Exception as e:
            self.get_logger().error(f"Failed to convert BGR->HSV: {e}")
            return

        lower = np.array([max(0, self.color_hue - self.color_range), self.color_saturation, self.color_value])
        upper = np.array([min(179, self.color_hue + self.color_range), 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        # detect blobs on mask
        keypoints = self.detector.detect(mask)

        # draw keypoints for debugging
        if len(keypoints) > 0:
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # choose the keypoint with the smallest valid depth (closest)
        closest_idx = -1
        closest_depth = float('inf')
        closest_uv = None
        search_radius = max(1, int(np.mean([kp.size for kp in keypoints]) / 4)) if len(keypoints) > 0 else 2

        for i, kp in enumerate(keypoints):
            u_px = int(round(kp.pt[0]))
            v_px = int(round(kp.pt[1]))

            if u_px < self.border or u_px >= cv_image.shape[1] - self.border or v_px < self.border or v_px >= cv_image.shape[0] - self.border:
                continue

            r = 2 + search_radius
            u0 = max(0, u_px - r)
            u1 = min(cv_depth.shape[1] - 1, u_px + r)
            v0 = max(0, v_px - r)
            v1 = min(cv_depth.shape[0] - 1, v_px + r)

            neighborhood = cv_depth[v0:v1+1, u0:u1+1]
            if neighborhood.size == 0:
                continue
            valid = neighborhood[np.isfinite(neighborhood) & (neighborhood > 0.01) & (neighborhood < 20.0)]
            if valid.size == 0:
                continue

            z_val = float(np.median(valid))

            if z_val is not None and z_val < closest_depth:
                closest_depth = z_val
                closest_idx = i
                closest_uv = (u_px, v_px)

        # Process valid object
        if closest_idx != -1 and closest_depth > 0:
            # Accepter objets entre 0.5 et 5.0 mètres pour la détection
            if 0.5 < closest_depth < 5.0:
                u_px, v_px = closest_uv
                z = closest_depth
                X = (u_px - cx) * z / fx
                Y = (v_px - cy) * z / fy
                Z = z
                transObj = (float(X), float(Y), float(Z))

                rotObj = tf_transformations.quaternion_from_euler(0, np.pi/2, -np.pi/2)

                # publish object TF
                transform = TransformStamped()
                transform.header = image_msg.header
                transform.child_frame_id = self.object_frame_id
                transform.transform.translation.x = transObj[0]
                transform.transform.translation.y = transObj[1]
                transform.transform.translation.z = transObj[2]
                transform.transform.rotation.x = rotObj[0]
                transform.transform.rotation.y = rotObj[1]
                transform.transform.rotation.z = rotObj[2]
                transform.transform.rotation.w = rotObj[3]
                self.br.sendTransform(transform)

                msg = String()
                msg.data = self.object_frame_id
                self.object_pub.publish(msg)

                # Calculer la position dans le référentiel map
                debris_pos_map = None
                try:
                    tmap = self.tf_buffer.lookup_transform(self.map_frame_id, image_msg.header.frame_id, 
                                                          image_msg.header.stamp, Duration(nanoseconds=500000000))
                    transMap = [tmap.transform.translation.x, tmap.transform.translation.y, tmap.transform.translation.z]
                    rotMap = [tmap.transform.rotation.x, tmap.transform.rotation.y, tmap.transform.rotation.z, tmap.transform.rotation.w]
                    (transMap, rotMap) = multiply_transforms(transMap, rotMap, transObj, rotObj)
                    debris_pos_map = (transMap[0], transMap[1])
                except Exception as e:
                    self.get_logger().warn(f"Map TF not available: {e}")
                
                # Vérifier si c'est un débris déjà traité (seulement si on n'est pas déjà en train de traiter un débris)
                if debris_pos_map is not None and self.stop_until is None:
                    if self.is_debris_already_detected(debris_pos_map):
                        self.get_logger().debug("Débris déjà traité (< 1m d'un précédent), ignoré.")
                        return

                # Centrage du blob dans l'image
                img_center_x = cv_image.shape[1] / 2.0
                blob_x_px = float(keypoints[closest_idx].pt[0])
                error_x = blob_x_px - img_center_x
                tolerance_px = 30  # tolérance en pixels

                self.get_logger().info(f"Blob détecté: erreur x={error_x:.1f}px, distance={closest_depth:.2f}m")

                twist = Twist()
                current_time = self.get_clock().now()

                # Si en période d'attente de 5 secondes
                if self.stop_until is not None:
                    if current_time < self.stop_until:
                        # Rester immobile
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        remaining = (self.stop_until - current_time).nanoseconds / 1e9
                        
                        # Appeler replan_path une seule fois au début de l'attente
                        if not self.replan_requested:
                            self.call_replan_service()
                        
                        self.get_logger().info(f"Attente après photo: {remaining:.1f}s restantes")
                        self.cmd_vel_pub.publish(twist)
                        return
                    else:
                        # Période terminée, MAINTENANT enregistrer le débris dans la liste
                        if self.current_debris_position is not None:
                            self.detected_debris_positions.append(self.current_debris_position)
                            self.get_logger().info(f"Débris confirmé à [{self.current_debris_position[0]:.2f}, {self.current_debris_position[1]:.2f}]")
                        
                        self.get_logger().info("Fin de l'attente de 5s, prêt pour nouveau débris")
                        self.stop_until = None
                        self.photo_taken = False
                        self.current_debris_position = None
                        self.replan_requested = False  # Réinitialiser pour le prochain débris
                        # Sortir pour que la prochaine frame vérifie si c'est un débris déjà traité
                        return

                # Vérifier si on est à moins de 2 mètres ET centré
                if closest_depth <= 2.0 and abs(error_x) <= tolerance_px:
                    # Conditions remplies : arrêter et prendre photo
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    
                    if not self.photo_taken:
                        # Prendre la photo
                        debris_id = len(self.detected_debris_positions) + 1
                        self.save_photo(cv_image, debris_id)
                        self.photo_taken = True
                        
                        # Stocker temporairement la position (sera ajoutée à la liste après les 5s)
                        if debris_pos_map is not None:
                            self.current_debris_position = debris_pos_map
                            self.get_logger().info(f"Photo du débris à [{debris_pos_map[0]:.2f}, {debris_pos_map[1]:.2f}]")
                            #self.report_debris_service(self.current_debris_position, f"debris_{debris_id}.jpg")
                    
                        # Démarrer le timer de 5 secondes
                        self.stop_until = current_time + Duration(seconds=5.0)
                        self.get_logger().info("Photo prise! Attente de 5 secondes...")
                    
                elif closest_depth > 2.0:
                    # Trop loin : avancer en centrant
                    ang_gain = 0.003
                    twist.angular.z = float(-ang_gain * error_x)
                    
                    if abs(error_x) > tolerance_px:
                        # Pas centré : avancer lentement
                        twist.linear.x = 0.15
                        self.get_logger().info(f"Approche + centrage: {closest_depth:.2f}m, erreur={error_x:.1f}px")
                    else:
                        # Centré : avancer plus vite
                        twist.linear.x = 0.20
                        self.get_logger().info(f"Approche (centré): {closest_depth:.2f}m")
                    
                    self.cmd_vel_pub.publish(twist)
                    
                else:
                    # Distance OK (< 2m) mais pas centré : rotation sur place
                    ang_gain = 0.004
                    twist.linear.x = 0.0
                    twist.angular.z = float(-ang_gain * error_x)
                    self.get_logger().info(f"Centrage final: erreur={error_x:.1f}px à {closest_depth:.2f}m")
                    self.cmd_vel_pub.publish(twist)

            else:
                self.get_logger().debug(f"Depth {closest_depth:.2f}m hors limites")
        else:
            # Pas de blob détecté
            current_time = self.get_clock().now()
            
            # Si on est en période d'attente, continuer à attendre
            if self.stop_until is not None:
                if current_time < self.stop_until:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    remaining = (self.stop_until - current_time).nanoseconds / 1e9
                    
                    # Appeler replan_path une seule fois
                    if not self.replan_requested:
                        self.call_replan_service()
                    
                    self.get_logger().info(f"Attente (pas de blob visible): {remaining:.1f}s restantes")
                    self.cmd_vel_pub.publish(twist)
                else:
                    # Attente terminée, enregistrer le débris
                    if self.current_debris_position is not None:
                        self.detected_debris_positions.append(self.current_debris_position)
                        self.get_logger().info(f"Débris confirmé à [{self.current_debris_position[0]:.2f}, {self.current_debris_position[1]:.2f}]")
                    
                    self.get_logger().info("Fin de l'attente de 5s (pas de blob), prêt pour nouveau débris")
                    self.stop_until = None
                    self.photo_taken = False
                    self.current_debris_position = None
                    self.replan_requested = False  # Réinitialiser pour le prochain débris
                    # Sortir pour éviter de retraiter immédiatement
                    return
            else:
                self.get_logger().debug("Aucun blob valide détecté")

        # publish debug image
        try:
            debug_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        except CvBridgeError as e:
            self.get_logger().warn(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    blobDetector = BlobDetector()
    rclpy.spin(blobDetector)
    blobDetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()