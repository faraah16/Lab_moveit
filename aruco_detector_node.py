#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CustomArucoDetector(Node):
    def __init__(self):
        super().__init__('custom_aruco_detector')
        self.bridge = CvBridge()
        
        # DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            10)
        
        self.pub_result = self.create_publisher(Image, '/aruco_detection/result', 10)
        
        self.frame_count = 0
        self.get_logger().info('=== Custom ArUco Detector DEMARRE ===')
    
    def image_callback(self, msg):
        try:
            self.frame_count += 1
            
            # Log TOUTES les 30 frames (environ chaque seconde)
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Frame {self.frame_count} - Noeud actif')
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Détecter
            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.parameters)
            
            result_image = cv_image.copy()
            
            # Dessiner ET logger
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(result_image, corners, ids)
                self.get_logger().info(f'>>> DETECTION: Markers {ids.flatten()} <<<', throttle_duration_sec=0.5)
            
            # Publier l'image (avec ou sans détection)
            result_msg = self.bridge.cv2_to_imgmsg(result_image, encoding="bgr8")
            result_msg.header.stamp = self.get_clock().now().to_msg()
            result_msg.header.frame_id = "camera_link"
            self.pub_result.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f'ERREUR: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CustomArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('=== Arret du noeud ===')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
