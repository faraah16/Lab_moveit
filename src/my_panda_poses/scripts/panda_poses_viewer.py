#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PandaPosesViewer(Node):
    def __init__(self):
        super().__init__('panda_poses_viewer')
        
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
            'my_home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'my_ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            'my_pick': [0.0, 0.5, 0.0, -1.5, 0.0, 2.0, 0.785],
            'my_extended': [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785],
            'my_transport': [0.0, -1.0, 0.0, -2.0, 0.0, 1.0, 0.0],
        }
        
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
            'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        self.get_logger().info('✅ Panda Poses Viewer initialisé!')
        self.get_logger().info(f'📋 Poses disponibles : {list(self.custom_poses.keys())}')
    
    def joint_state_callback(self, msg):
        """Recevoir l'état actuel des joints"""
        self.current_joint_state = msg
    
    def print_current_pose(self):
        """Afficher la pose actuelle du robot"""
        if self.current_joint_state is None:
            self.get_logger().warn('⚠️  En attente des données du robot...')
            return
        
        print('\n' + '='*60)
        print('📍 POSITION ACTUELLE DU ROBOT')
        print('='*60)
        
        # Extraire seulement les joints du Panda
        panda_joints = {}
        for name, position in zip(self.current_joint_state.name, self.current_joint_state.position):
            if 'panda_joint' in name and name in self.joint_names:
                panda_joints[name] = position
        
        # Afficher dans l'ordre
        for joint_name in self.joint_names:
            if joint_name in panda_joints:
                print(f'  {joint_name}: {panda_joints[joint_name]:.4f} rad')
        print('='*60 + '\n')
    
    def list_poses(self):
        """Afficher toutes les poses disponibles"""
        print('\n' + '='*60)
        print('📋 VOS POSES PERSONNALISÉES')
        print('='*60)
        
        for pose_name, joint_values in self.custom_poses.items():
            print(f'\n🎯 Pose: {pose_name}')
            print('-' * 40)
            for i, joint_name in enumerate(self.joint_names):
                print(f'  {joint_name}: {joint_values[i]:.4f} rad')
        
        print('\n' + '='*60)
        print('💡 Pour tester ces poses dans RViz:')
        print('   1. Ouvrez l\'onglet "Joints" dans MotionPlanning')
        print('   2. Entrez les valeurs ci-dessus')
        print('   3. Cliquez sur "Plan" puis "Execute"')
        print('='*60 + '\n')
    
    def compare_with_pose(self, pose_name):
        """Comparer la position actuelle avec une pose"""
        if pose_name not in self.custom_poses:
            self.get_logger().error(f'❌ Pose "{pose_name}" non trouvée!')
            return
        
        if self.current_joint_state is None:
            self.get_logger().warn('⚠️  En attente des données du robot...')
            return
        
        # Extraire les positions actuelles
        current_positions = {}
        for name, position in zip(self.current_joint_state.name, self.current_joint_state.position):
            if 'panda_joint' in name and name in self.joint_names:
                current_positions[name] = position
        
        target_positions = self.custom_poses[pose_name]
        
        print('\n' + '='*60)
        print(f'📊 COMPARAISON AVEC LA POSE: {pose_name}')
        print('='*60)
        print(f'{"Joint":<15} {"Actuel":<12} {"Cible":<12} {"Différence":<12}')
        print('-' * 60)
        
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in current_positions:
                current = current_positions[joint_name]
                target = target_positions[i]
                diff = abs(current - target)
                status = '✅' if diff < 0.01 else '⚠️ '
                print(f'{status} {joint_name:<13} {current:>10.4f}  {target:>10.4f}  {diff:>10.4f}')
        
        print('='*60 + '\n')


def main():
    rclpy.init()
    
    node = PandaPosesViewer()
    
    # Attendre un peu pour recevoir les données
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    
    # Menu interactif
    print("\n" + "="*60)
    print("🤖 VISUALISEUR DE POSES PANDA")
    print("="*60)
    print("\nCommandes disponibles :")
    print("  1 - Lister toutes vos poses personnalisées")
    print("  2 - Afficher la position actuelle du robot")
    print("  3 - Comparer avec 'my_home'")
    print("  4 - Comparer avec 'my_ready'")
    print("  5 - Comparer avec 'my_pick'")
    print("  q - Quitter")
    print("="*60 + "\n")
    
    try:
        while rclpy.ok():
            command = input("Commande : ").strip()
            
            if command == '1':
                node.list_poses()
            elif command == '2':
                node.print_current_pose()
            elif command == '3':
                node.compare_with_pose('my_home')
            elif command == '4':
                node.compare_with_pose('my_ready')
            elif command == '5':
                node.compare_with_pose('my_pick')
            elif command == 'q':
                print("👋 Au revoir!")
                break
            else:
                print(f"❌ Commande inconnue: {command}")
            
            # Spin pour recevoir les messages
            for _ in range(5):
                rclpy.spin_once(node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        print("\n👋 Arrêt du programme...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
