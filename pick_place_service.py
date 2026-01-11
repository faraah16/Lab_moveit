#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import time
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
import sys

class PickPlaceService(Node):
    def __init__(self):
        super().__init__('pick_place_service')
        
        self.get_logger().info('🦾 Pick & Place Service démarrage...')
        
        # Initialiser MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        
        # Groupes MoveIt
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")
        
        # Configuration
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)
        self.arm_group.set_planning_time(5.0)
        
        # Subscriber pour commandes
        self.command_sub = self.create_subscription(
            String,
            '/pick_place/command',
            self.command_callback,
            10
        )
        
        # Publisher pour status
        self.status_pub = self.create_publisher(
            String,
            '/pick_place/status',
            10
        )
        
        self.get_logger().info('✅ Pick & Place Service prêt !')
    
    def command_callback(self, msg):
        """Reçoit commande pick/place"""
        try:
            command = yaml.safe_load(msg.data)
            action = command.get('action', '')
            
            if action == 'pick':
                color = command.get('color', 'unknown')
                self.execute_pick(color)
                
            elif action == 'place':
                destination = command.get('destination', 'unknown')
                self.execute_place(destination)
                
            elif action == 'home':
                self.go_home()
                
        except Exception as e:
            self.get_logger().error(f'❌ Erreur: {e}')
            self.publish_status('error', str(e))
    
    def publish_status(self, status, message=''):
        """Publie le status"""
        data = {
            'status': status,
            'message': message,
            'timestamp': time.time()
        }
        msg = String()
        msg.data = yaml.dump(data)
        self.status_pub.publish(msg)
    
    def go_home(self):
        """Position de repos"""
        self.get_logger().info('🏠 Retour position home...')
        
        # Position home (bras vertical)
        self.arm_group.set_named_target("home")
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        
        if success:
            self.get_logger().info('✅ Home atteint')
            self.publish_status('home_reached')
        else:
            self.get_logger().error('❌ Échec home')
            self.publish_status('error', 'Failed to reach home')
    
    def open_gripper(self):
        """Ouvre le gripper"""
        self.get_logger().info('   ✋ Ouverture gripper...')
        self.gripper_group.set_joint_value_target([0.04, -0.04])
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        time.sleep(0.5)
    
    def close_gripper(self):
        """Ferme le gripper"""
        self.get_logger().info('   ✊ Fermeture gripper...')
        self.gripper_group.set_joint_value_target([0.0, 0.0])
        self.gripper_group.go(wait=True)
        self.gripper_group.stop()
        time.sleep(0.5)
    
    def execute_pick(self, color):
        """Exécute un pick sur une table colorée"""
        self.get_logger().info(f'')
        self.get_logger().info(f'🦾 PICK: Table {color}')
        self.get_logger().info(f'')
        
        self.publish_status('picking', f'Picking from {color} table')
        
        try:
            # 1. Ouvrir gripper
            self.open_gripper()
            
            # 2. Position au-dessus de la table (ajustée selon couleur)
            # Les tables sont devant le robot à ~0.4m
            if color == 'red':
                x, y = 0.35, 0.0
            elif color == 'blue':
                x, y = 0.35, 0.0
            elif color == 'yellow':
                x, y = 0.35, 0.0
            else:
                x, y = 0.35, 0.0
            
            self.get_logger().info(f'   📍 Approche table...')
            
            # Position approche (au-dessus)
            target_pose = Pose()
            target_pose.position.x = x
            target_pose.position.y = y
            target_pose.position.z = 0.15  # 15cm au-dessus de la table
            target_pose.orientation.w = 1.0
            
            self.arm_group.set_pose_target(target_pose)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            if not success:
                raise Exception("Failed to reach approach position")
            
            self.get_logger().info(f'   ✅ Position approche atteinte')
            time.sleep(1.0)
            
            # 3. Descendre pour pick
            self.get_logger().info(f'   ⬇️  Descente vers box...')
            target_pose.position.z = 0.05  # 5cm (juste au-dessus de la table)
            
            self.arm_group.set_pose_target(target_pose)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            if not success:
                raise Exception("Failed to reach pick position")
            
            time.sleep(0.5)
            
            # 4. Fermer gripper
            self.close_gripper()
            
            self.get_logger().info(f'   ✅ Box attrapée !')
            
            # 5. Remonter
            self.get_logger().info(f'   ⬆️  Remontée...')
            target_pose.position.z = 0.20
            
            self.arm_group.set_pose_target(target_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            time.sleep(0.5)
            
            self.get_logger().info(f'✅ PICK TERMINÉ !')
            self.publish_status('pick_complete')
            
        except Exception as e:
            self.get_logger().error(f'❌ Erreur pick: {e}')
            self.publish_status('error', str(e))
    
    def execute_place(self, destination):
        """Exécute un place vers destination"""
        self.get_logger().info(f'')
        self.get_logger().info(f'🦾 PLACE: {destination}')
        self.get_logger().info(f'')
        
        self.publish_status('placing', f'Placing to {destination}')
        
        try:
            # Position au-dessus de la destination
            self.get_logger().info(f'   📍 Approche destination...')
            
            target_pose = Pose()
            target_pose.position.x = 0.35
            target_pose.position.y = 0.0
            target_pose.position.z = 0.20
            target_pose.orientation.w = 1.0
            
            self.arm_group.set_pose_target(target_pose)
            success = self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            if not success:
                raise Exception("Failed to reach place approach")
            
            time.sleep(0.5)
            
            # Descendre
            self.get_logger().info(f'   ⬇️  Descente...')
            target_pose.position.z = 0.08
            
            self.arm_group.set_pose_target(target_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            time.sleep(0.5)
            
            # Ouvrir gripper (déposer)
            self.open_gripper()
            
            self.get_logger().info(f'   ✅ Box déposée !')
            
            # Remonter
            target_pose.position.z = 0.20
            self.arm_group.set_pose_target(target_pose)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            
            self.get_logger().info(f'✅ PLACE TERMINÉ !')
            self.publish_status('place_complete')
            
        except Exception as e:
            self.get_logger().error(f'❌ Erreur place: {e}')
            self.publish_status('error', str(e))

def main(args=None):
    rclpy.init(args=args)
    service = PickPlaceService()
    
    try:
        rclpy.spin(service)
    except KeyboardInterrupt:
        pass
    finally:
        service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
