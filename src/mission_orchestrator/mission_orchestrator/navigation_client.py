#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time


class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        
        # Action client pour Nav2
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        self.get_logger().info('🧭 Navigation Client initialisé')
    
    def navigate_to_pose(self, x, y, z=0.0, orientation_w=1.0, frame_id='map'):
        """
        Navigue vers une position donnée
        
        Args:
            x, y, z: Position en mètres
            orientation_w: Orientation (quaternion w)
            frame_id: Frame de référence (défaut: 'map')
            
        Returns:
            bool: True si succès, False sinon
        """
        # Attendre que le serveur d'action soit disponible
        self.get_logger().info(f'⏳ Attente du serveur Nav2...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Serveur Nav2 non disponible !')
            return False
        
        # Créer le goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = orientation_w
        
        self.get_logger().info(f'🎯 Navigation vers ({x:.2f}, {y:.2f})')
        
        # Envoyer le goal
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Attendre que le goal soit accepté
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejeté par Nav2')
            return False
        
        self.get_logger().info('✅ Goal accepté, navigation en cours...')
        
        # Attendre le résultat
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.get_logger().info('✅ Navigation réussie !')
            return True
        else:
            self.get_logger().error(f'❌ Navigation échouée (status: {result.status})')
            return False
    
    def feedback_callback(self, feedback_msg):
        """Callback pour le feedback de navigation"""
        feedback = feedback_msg.feedback
        # On peut ajouter un log de progression ici si nécessaire
        # Pour l'instant on reste silencieux pour ne pas polluer les logs
        pass


def main(args=None):
    rclpy.init(args=args)
    nav_client = NavigationClient()
    
    try:
        rclpy.spin(nav_client)
    except KeyboardInterrupt:
        pass
    finally:
        nav_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

