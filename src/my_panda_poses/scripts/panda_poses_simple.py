#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveGroupActionGoal, RobotState, Constraints, JointConstraint
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
import time

class PandaCustomPoses(Node):
    def __init__(self):
        super().__init__('panda_custom_poses')
        
        # Client pour MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Subscriber pour l'état actuel du robot
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        
        # Définir vos poses personnalisées (en radians)
        self.custom_poses = {
            'my_home': {
                'panda_joint1': 0.0,
                'panda_joint2': 0.0,
                'panda_joint3': 0.0,
                'panda_joint4': 0.0,
                'panda_joint5': 0.0,
                'panda_joint6': 0.0,
                'panda_joint7': 0.0
            },
            
            'my_ready': {
                'panda_joint1': 0.0,
                'panda_joint2': -0.785,
                'panda_joint3': 0.0,
                'panda_joint4': -2.356,
                'panda_joint5': 0.0,
                'panda_joint6': 1.571,
                'panda_joint7': 0.785
            },
            
            'my_pick': {
                'panda_joint1': 0.0,
                'panda_joint2': 0.5,
                'panda_joint3': 0.0,
                'panda_joint4': -1.5,
                'panda_joint5': 0.0,
                'panda_joint6': 2.0,
                'panda_joint7': 0.785
            },
            
            'my_extended': {
                'panda_joint1': 0.0,
                'panda_joint2': -0.3,
                'panda_joint3': 0.0,
                'panda_joint4': -2.2,
                'panda_joint5': 0.0,
                'panda_joint6': 2.0,
                'panda_joint7': 0.785
            },
        }
        
        self.get_logger().info('✅ Panda Custom Poses initialisé!')
        self.get_logger().info(f'📋 Poses disponibles : {list(self.custom_poses.keys())}')
    
    def joint_state_callback(self, msg):
        """Recevoir l'état actuel des joints"""
        self.current_joint_state = msg
    
    def print_current_pose(self):
        """Afficher la pose actuelle du robot"""
        if self.current_joint_state is None:
            self.get_logger().warn('⚠️  Aucun état de joint reçu encore')
            return
        
        self.get_logger().info('📍 Position actuelle du robot :')
        for name, position in zip(self.current_joint_state.name, self.current_joint_state.position):
            if 'panda_joint' in name:
                self.get_logger().info(f'  {name}: {position:.3f} rad')
    
    def list_poses(self):
        """Afficher toutes les poses disponibles"""
        self.get_logger().info('='*60)
        self.get_logger().info('📋 POSES PERSONNALISÉES DISPONIBLES :')
        self.get_logger().info('='*60)
        for pose_name, joint_values in self.custom_poses.items():
            self.get_logger().info(f'\n  🎯 {pose_name}:')
            for joint_name, value in joint_values.items():
                self.get_logger().info(f'    {joint_name}: {value:.3f}')
        self.get_logger().info('='*60)


def main():
    rclpy.init()
    
    node = PandaCustomPoses()
    
    # Menu interactif
    print("\n" + "="*60)
    print("🤖 VISUALISEUR DE POSES PERSONNALISÉES POUR PANDA")
    print("="*60)
    print("\nCommandes disponibles :")
    print("  1 - Lister toutes les poses personnalisées")
    print("  2 - Afficher la position actuelle du robot")
    print("  q - Quitter")
    print("\n💡 Utilisez RViz pour déplacer le robot manuellement")
    print("   vers vos poses personnalisées!")
    print("="*60 + "\n")
    
    try:
        while rclpy.ok():
            command = input("Entrez une commande : ").strip()
            
            if command == '1':
                node.list_poses()
            elif command == '2':
                node.print_current_pose()
            elif command == 'q':
                print("👋 Au revoir!")
                break
            else:
                print(f"❌ Commande inconnue: {command}")
            
            # Petit spin pour traiter les callbacks ROS
            rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        print("\n👋 Arrêt du programme...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
