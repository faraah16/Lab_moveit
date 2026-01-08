#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time


class NavigationClient(Node):
    def __init__(self, orchestrator=None):
        super().__init__('navigation_client')
        
        # ⭐ NOUVEAU : Référence au MissionOrchestrator pour vérifier batterie
        self.orchestrator = orchestrator
        
        # Action client pour Nav2
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        
        # ⭐ NOUVEAU : Stocker le goal_handle pour pouvoir l'annuler
        self.current_goal_handle = None
        
        self.get_logger().info('🧭 Navigation Client initialisé')
    
    def cancel_goal(self):
        """Annule le goal de navigation en cours"""
        if self.current_goal_handle:
            self.get_logger().warn('🛑 Annulation du goal Nav2...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            self.get_logger().info('✅ Goal Nav2 annulé')
            self.current_goal_handle = None
    
    def navigate_to_pose(self, x, y, z=0.0, orientation_w=1.0, frame_id='map', is_charging_mission=False):
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
        
        # ⭐ NOUVEAU : Stocker le goal_handle
        self.current_goal_handle = goal_handle
        
        # ═══════════════════════════════════════════════════════
        # ATTENTE DU RÉSULTAT AVEC VÉRIFICATION BATTERIE
        # ═══════════════════════════════════════════════════════
        result_future = goal_handle.get_result_async()
        
        # Boucle d'attente avec vérification batterie toutes les 0.5s
        while not result_future.done():
            # ⭐ VÉRIFICATION BATTERIE (sauf si on va charger !)
            if not is_charging_mission and self.orchestrator and self.orchestrator.low_battery_mode:
                self.get_logger().warn('')
                self.get_logger().warn('🚨🚨🚨 BATTERIE FAIBLE DÉTECTÉE PENDANT NAV2 ! 🚨🚨🚨')
                self.get_logger().warn(f'   Niveau: {self.orchestrator.battery_level:.1f}%')
                self.get_logger().warn('   → ANNULATION du goal de navigation')
                self.get_logger().warn('')
                
                # Annuler le goal
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                
                self.get_logger().info('✅ Goal Nav2 annulé - Retour pour aller charger')
                self.current_goal_handle = None
                return False
            
            # Attendre un peu avant de revérifier
            rclpy.spin_once(self, timeout_sec=0.5)
        
        # Récupérer le résultat
        result = result_future.result()
        self.current_goal_handle = None
        
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