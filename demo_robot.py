#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration
import time

class RobotDemo(Node):
    def __init__(self):
        super().__init__('robot_demo')
        
        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            '/gripper_controller/commands',
            10
        )
        
        self.get_logger().info('🤖 Robot Demo Node démarré !')
        time.sleep(2)
        
    def move_arm(self, joint1_pos, joint2_pos, duration_sec=3.0):
        """Déplace le bras vers une position"""
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2']
        
        point = JointTrajectoryPoint()
        point.positions = [joint1_pos, joint2_pos]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)
        
        msg.points = [point]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'📍 Bras → joint1={joint1_pos:.2f}, joint2={joint2_pos:.2f}')
        
    def move_gripper(self, position):
        """
        Contrôle le gripper
        position > 0 : ouvert
        position = 0 : fermé
        """
        msg = Float64MultiArray()
        msg.data = [position, -position]  # Doigts opposés
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'✋ Gripper → {position:.3f}')
        
    def run_demo(self):
        """Séquence de démonstration complète"""
        self.get_logger().info('🎬 DÉBUT DE LA DÉMONSTRATION')
        
        # Position initiale
        self.get_logger().info('\n=== 1. Position HOME ===')
        self.move_arm(0.0, 0.0)
        self.move_gripper(0.0)  # Fermé
        time.sleep(4)
        
        # Rotation base
        self.get_logger().info('\n=== 2. Rotation base (joint1) ===')
        self.move_arm(1.57, 0.0)  # 90° rotation
        time.sleep(4)
        
        self.move_arm(-1.57, 0.0)  # -90° rotation
        time.sleep(4)
        
        self.move_arm(0.0, 0.0)  # Retour centre
        time.sleep(4)
        
        # Mouvement coude
        self.get_logger().info('\n=== 3. Mouvement coude (joint2) ===')
        self.move_arm(0.0, 1.0)  # Coude plié
        time.sleep(4)
        
        self.move_arm(0.0, -1.0)  # Coude étendu
        time.sleep(4)
        
        self.move_arm(0.0, 0.0)  # Retour neutre
        time.sleep(4)
        
        # Ouverture/fermeture gripper
        self.get_logger().info('\n=== 4. Test Gripper ===')
        for i in range(3):
            self.move_gripper(0.04)  # Ouvert
            time.sleep(2)
            self.move_gripper(0.0)   # Fermé
            time.sleep(2)
        
        # Séquence combinée
        self.get_logger().info('\n=== 5. Séquence pick & place simulée ===')
        
        # Position de saisie
        self.move_arm(0.8, -0.5)
        self.move_gripper(0.04)  # Ouvrir
        time.sleep(4)
        
        # Saisir
        self.move_gripper(0.0)  # Fermer
        time.sleep(2)
        
        # Lever
        self.move_arm(0.8, 0.5)
        time.sleep(4)
        
        # Rotation
        self.move_arm(-0.8, 0.5)
        time.sleep(4)
        
        # Déposer
        self.move_arm(-0.8, -0.3)
        time.sleep(4)
        
        self.move_gripper(0.04)  # Ouvrir
        time.sleep(2)
        
        # Retour home
        self.get_logger().info('\n=== 6. Retour HOME ===')
        self.move_arm(0.0, 0.0)
        self.move_gripper(0.0)
        time.sleep(4)
        
        self.get_logger().info('\n✅ DÉMONSTRATION TERMINÉE !')

def main():
    rclpy.init()
    node = RobotDemo()
    
    try:
        node.run_demo()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

