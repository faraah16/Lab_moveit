#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoTester(Node):
    def __init__(self):
        super().__init__('aruco_tester')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # Tester TOUS les dictionnaires courants
        self.dictionaries = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        }
        
        self.get_logger().info('Aruco Tester demarre - Test de tous les dictionnaires...')
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        for dict_name, dict_id in self.dictionaries.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            parameters = cv2.aruco.DetectorParameters()
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
            
            if ids is not None and len(ids) > 0:
                self.get_logger().info(f'DETECTION REUSSIE avec {dict_name} ! IDs detectes: {ids.flatten()}')
            else:
                self.get_logger().info(f'Aucun marker avec {dict_name}')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTester()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
