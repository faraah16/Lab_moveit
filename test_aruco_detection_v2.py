#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArucoTester(Node):
    def __init__(self):
        super().__init__('aruco_tester')
        self.bridge = CvBridge()
        self.image_received = False
        self.subscription = self.create_subscription(
            Image,
            '/rgb_camera/image_raw',
            self.image_callback,
            10)
        
        self.dictionaries = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
        }
        
        print('\n=== DEBUT DU TEST ARUCO ===')
        print('En attente d\'une image de /rgb_camera/image_raw...\n')
    
    def image_callback(self, msg):
        if self.image_received:
            return
        
        self.image_received = True
        print('Image recue ! Test en cours...\n')
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        found_any = False
        for dict_name, dict_id in self.dictionaries.items():
            aruco_dict = cv2.aruco.Dictionary_get(dict_id)
            parameters = cv2.aruco.DetectorParameters_create()
            corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
            
            if ids is not None and len(ids) > 0:
                print(f'SUCCES avec {dict_name} !')
                print(f'   IDs detectes: {ids.flatten()}')
                print(f'   Nombre de markers: {len(ids)}\n')
                found_any = True
            else:
                print(f'{dict_name}: Aucun marker')
        
        if not found_any:
            print('\nAUCUN MARKER DETECTE avec aucun dictionnaire!')
        
        print('\n=== FIN DU TEST ===\n')

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTester()
    
    for i in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.image_received:
            break
    
    if not node.image_received:
        print('ERREUR: Aucune image recue')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
