#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
import time

class PandaCustomPoses(Node):
    def __init__(self):
        super().__init__('panda_custom_poses')
        
        # Initialiser MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        self.panda_arm = self.moveit.get_planning_component("panda_arm")
        
        # Définir vos poses personnalisées (en radians)
        self.custom_poses = {
            'my_home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            
            'my_ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            
            'my_pick': [0.0, 0.5, 0.0, -1.5, 0.0, 2.0, 0.785],
            
            'my_extended': [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785],
            
            'my_transport': [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0],
            
            'my_left_side': [1.57, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0],
            
            'my_right_side': [-1.57, -0.5, 0.0, -2.0, 0.0, 1.5, 0.0],
        }
        
        self.get_logger().info('✅ Panda Custom Poses initialisé!')
        self.get_logger().info(f'📋 Poses disponibles : {list(self.custom_poses.keys())}')
        
    def go_to_pose(self, pose_name):
        """Déplacer le robot vers une pose personnalisée"""
        
        if pose_name not in self.custom_poses:
            self.get_logger().error(f'❌ Pose "{pose_name}" non trouvée!')
            self.get_logger().info(f'Poses disponibles : {list(self.custom_poses.keys())}')
            return False
        
        self.get_logger().info(f'🎯 Déplacement vers la pose : {pose_name}')
        
        # Récupérer les valeurs des joints
        joint_values = self.custom_poses[pose_name]
        
        # Créer le dictionnaire de joints
        joint_state = {
            'panda_joint1': joint_values[0],
            'panda_joint2': joint_values[1],
            'panda_joint3': joint_values[2],
            'panda_joint4': joint_values[3],
            'panda_joint5': joint_values[4],
            'panda_joint6': joint_values[5],
            'panda_joint7': joint_values[6]
        }
        
        # Définir la cible
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name="", joint_state=joint_state)
        
        # Planifier
        self.get_logger().info('🔍 Planification en cours...')
        plan_result = self.panda_arm.plan()
        
        if plan_result:
            self.get_logger().info('✅ Plan trouvé! Exécution...')
            # Exécuter le mouvement
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
            self.get_logger().info(f'✅ Pose "{pose_name}" atteinte!')
            return True
        else:
            self.get_logger().error('❌ Échec de la planification')
            return False
    
    def list_poses(self):
        """Afficher toutes les poses disponibles"""
        self.get_logger().info('='*50)
        self.get_logger().info('📋 POSES PERSONNALISÉES DISPONIBLES :')
        self.get_logger().info('='*50)
        for pose_name, joint_values in self.custom_poses.items():
            self.get_logger().info(f'  • {pose_name}')
        self.get_logger().info('='*50)
    
    def add_new_pose(self, pose_name, joint_values):
        """Ajouter une nouvelle pose"""
        if len(joint_values) != 7:
            self.get_logger().error('❌ Il faut exactement 7 valeurs de joints!')
            return False
        
        self.custom_poses[pose_name] = joint_values
        self.get_logger().info(f'✅ Nouvelle pose "{pose_name}" ajoutée!')
        return True
    
    def demo_sequence(self):
        """Démonstration : séquence de mouvements"""
        self.get_logger().info('🎬 Démarrage de la séquence de démonstration...')
        
        poses_sequence = ['my_home', 'my_ready', 'my_extended', 'my_pick', 'my_home']
        
        for pose_name in poses_sequence:
            if self.go_to_pose(pose_name):
                time.sleep(2)  # Pause de 2 secondes entre chaque mouvement
            else:
                self.get_logger().error(f'Arrêt de la séquence à cause d\'une erreur')
                break
        
        self.get_logger().info('🎬 Séquence terminée!')


def main():
    rclpy.init()
    
    node = PandaCustomPoses()
    
    # Menu interactif
    print("\n" + "="*50)
    print("🤖 CONTRÔLE DU PANDA - POSES PERSONNALISÉES")
    print("="*50)
    print("\nCommandes disponibles :")
    print("  1 - Lister toutes les poses")
    print("  2 - Aller à my_home")
    print("  3 - Aller à my_ready")
    print("  4 - Aller à my_pick")
    print("  5 - Aller à my_extended")
    print("  6 - Lancer séquence de démonstration")
    print("  q - Quitter")
    print("="*50 + "\n")
    
    try:
        while rclpy.ok():
            command = input("Entrez une commande : ").strip()
            
            if command == '1':
                node.list_poses()
            elif command == '2':
                node.go_to_pose('my_home')
            elif command == '3':
                node.go_to_pose('my_ready')
            elif command == '4':
                node.go_to_pose('my_pick')
            elif command == '5':
                node.go_to_pose('my_extended')
            elif command == '6':
                node.demo_sequence()
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
