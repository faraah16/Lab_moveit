#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from std_msgs.msg import String
import json

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        
        # Client pour interroger Gazebo (ENTITY au lieu de MODEL)
        self.get_entity_state_client = self.create_client(
            GetEntityState,
            '/gazebo/get_entity_state'
        )
        
        # Publisher pour les boxes détectées
        self.detected_boxes_pub = self.create_publisher(
            String,
            '/detected_boxes',
            10
        )
        
        # Attendre que Gazebo soit prêt
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Attente du service Gazebo...')
        
        self.get_logger().info('✅ Box Detector prêt !')
    
    def detect_boxes_at_depot(self):
        """Détecte toutes les boxes sur la grande table noire (depot)"""
        
        # Liste des boxes à chercher
        box_names = [
            'red_box_1',
            'red_box_2',
            'blue_box_1',
            'yellow_box_1',
            'yellow_box_2'
        ]
        
        detected_boxes = []
        
        self.get_logger().info('🔍 Détection des boxes en cours...')
        
        for box_name in box_names:
            request = GetEntityState.Request()
            request.name = box_name
            request.reference_frame = ''
            
            try:
                future = self.get_entity_state_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                
                if future.result() is not None:
                    response = future.result()
                    
                    if response.success:
                        # Extraire couleur du nom
                        if 'red' in box_name:
                            color = 'red'
                        elif 'blue' in box_name:
                            color = 'blue'
                        elif 'yellow' in box_name:
                            color = 'yellow'
                        else:
                            color = 'unknown'
                        
                        box_info = {
                            'name': box_name,
                            'color': color,
                            'x': response.state.pose.position.x,
                            'y': response.state.pose.position.y,
                            'z': response.state.pose.position.z
                        }
                        
                        detected_boxes.append(box_info)
                        
                        self.get_logger().info(
                            f'  ✓ {box_name} ({color}) → '
                            f'X={box_info["x"]:.2f}, '
                            f'Y={box_info["y"]:.2f}, '
                            f'Z={box_info["z"]:.2f}'
                        )
                    else:
                        self.get_logger().warn(f'  ✗ {box_name} non trouvée')
                        
            except Exception as e:
                self.get_logger().error(f'Erreur pour {box_name}: {e}')
        
        # Publier les résultats
        if detected_boxes:
            msg = String()
            msg.data = json.dumps(detected_boxes)
            self.detected_boxes_pub.publish(msg)
            
            self.get_logger().info(
                f'\n📦 {len(detected_boxes)} box(es) détectée(s) !'
            )
        
        return detected_boxes

def main():
    rclpy.init()
    detector = BoxDetector()
    
    try:
        # Détecter les boxes
        boxes = detector.detect_boxes_at_depot()
        
        # Garder le node actif
        rclpy.spin(detector)
        
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
