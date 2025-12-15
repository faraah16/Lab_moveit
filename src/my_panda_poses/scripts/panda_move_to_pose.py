#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, 
    Constraints, 
    JointConstraint,
    PlanningOptions,
    RobotState
)
from sensor_msgs.msg import JointState
import time

class PandaMover(Node):
    def __init__(self):
        super().__init__('panda_mover')
        
        # Action client pour MoveGroup
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Publisher pour les commandes de joint
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )
        
        # Subscriber pour l'état des joints
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_state = None
        
        # Vos poses personnalisées
        self.poses = {
            'my_home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'my_ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            'my_pick': [0.0, 0.5, 0.0, -1.5, 0.0, 2.0, 0.785],
            'my_extended': [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785],
        }
        
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        self.get_logger().info('✅ Panda Mover initialisé!')
        self.get_logger().info('⏳ Attente de la connexion au serveur d\'action...')
        self._action_client.wait_for_server()
        self.get_logger().info('✅ Connecté au serveur MoveGroup!')
    
    def joint_state_callback(self, msg):
        self.current_joint_state = msg
    
    def move_to_pose(self, pose_name):
        """Déplacer le robot vers une pose nommée"""
        
        if pose_name not in self.poses:
            self.get_logger().error(f'❌ Pose "{pose_name}" inconnue!')
            return False
        
        target_positions = self.poses[pose_name]
        
        self.get_logger().info(f'🎯 Déplacement vers: {pose_name}')
        self.get_logger().info(f'   Valeurs: {target_positions}')
        
        # Créer une trajectoire de joints
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Point de trajectoire
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start.sec = 3  # 3 secondes pour atteindre la pose
        
        trajectory.points = [point]
        
        # Publier la trajectoire
        self.get_logger().info('📤 Envoi de la commande...')
        self.joint_pub.publish(trajectory)
        self.get_logger().info('✅ Commande envoyée!')
        
        return True
    
    def move_to_joint_values(self, joint_values):
        """Déplacer vers des valeurs de joints personnalisées"""
        
        if len(joint_values) != 7:
            self.get_logger().error('❌ Il faut 7 valeurs de joints!')
            return False
        
        self.get_logger().info(f'🎯 Déplacement vers valeurs personnalisées')
        
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start.sec = 3
        
        trajectory.points = [point]
        
        self.joint_pub.publish(trajectory)
        self.get_logger().info('✅ Commande envoyée!')
        
        return True
    
    def print_poses(self):
        """Afficher toutes les poses"""
        print('\n' + '='*60)
        print('📋 POSES DISPONIBLES')
        print('='*60)
        for name, values in self.poses.items():
            print(f'\n🎯 {name}:')
            for i, joint_name in enumerate(self.joint_names):
                print(f'   {joint_name}: {values[i]:.4f}')
        print('='*60 + '\n')


def main():
    rclpy.init()
    
    node = PandaMover()
    
    # Attendre un peu
    time.sleep(1)
    
    print("\n" + "="*60)
    print("🤖 CONTRÔLE AUTOMATIQUE DU PANDA")
    print("="*60)
    print("\nCommandes :")
    print("  1 - Aller à my_home")
    print("  2 - Aller à my_ready")
    print("  3 - Aller à my_pick")
    print("  4 - Aller à my_extended")
    print("  5 - Lister toutes les poses")
    print("  6 - Séquence automatique")
    print("  q - Quitter")
    print("="*60 + "\n")
    
    try:
        while rclpy.ok():
            command = input("Commande : ").strip()
            
            if command == '1':
                node.move_to_pose('my_home')
            elif command == '2':
                node.move_to_pose('my_ready')
            elif command == '3':
                node.move_to_pose('my_pick')
            elif command == '4':
                node.move_to_pose('my_extended')
            elif command == '5':
                node.print_poses()
            elif command == '6':
                print("🎬 Séquence automatique...")
                for pose_name in ['my_home', 'my_ready', 'my_extended', 'my_pick', 'my_home']:
                    node.move_to_pose(pose_name)
                    time.sleep(4)  # Attendre 4 secondes entre chaque mouvement
                print("✅ Séquence terminée!")
            elif command == 'q':
                print("👋 Au revoir!")
                break
            else:
                print(f"❌ Commande inconnue")
            
            rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        print("\n👋 Arrêt...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
