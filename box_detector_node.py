#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from gazebo_msgs.srv import GetEntityState
from std_msgs.msg import String
import json
import threading

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        
        # Client pour interroger Gazebo
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
        
        # Timer pour détection périodique
        self.timer = self.create_timer(2.0, self.detection_timer_callback)
        
        # Détecter une fois au démarrage (dans un thread)
        threading.Thread(target=self.detect_boxes_at_depot, daemon=True).start()
    
    def detection_timer_callback(self):
        """Appelé toutes les 2 secondes par le timer"""
        # Lancer détection dans un thread séparé
        threading.Thread(target=self.detect_boxes_at_depot, daemon=True).start()
    
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
                # Appel asynchrone
                future = self.get_entity_state_client.call_async(request)
                
                # Attendre réponse (max 1 seconde)
                import time
                timeout = 1.0
                start = time.time()
                
                while not future.done():
                    time.sleep(0.01)
                    if time.time() - start > timeout:
                        self.get_logger().warn(f'  ⚠️  Timeout pour {box_name}')
                        break
                
                if future.done():
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
                self.get_logger().error(f'  ❌ Erreur pour {box_name}: {e}')
        
        # Publier les résultats
        msg = String()
        msg.data = json.dumps(detected_boxes)
        self.detected_boxes_pub.publish(msg)
        
        if detected_boxes:
            self.get_logger().info(f'📦 {len(detected_boxes)} box(es) détectée(s) !')
        else:
            self.get_logger().warn(f'⚠️  AUCUNE box détectée cette fois !')
        
        return detected_boxes

def main():
    rclpy.init()
    detector = BoxDetector()
    
    # Utiliser MultiThreadedExecutor pour gérer les threads
    executor = MultiThreadedExecutor()
    executor.add_node(detector)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()