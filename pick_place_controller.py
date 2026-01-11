#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommand
import json
import time
import yaml

class PickPlaceController(Node):
    def __init__(self):
        super().__init__('pick_place_controller')
        
        self.get_logger().info('🦾 Pick & Place Controller démarré')
        
        # Publishers pour contrôle bras + gripper
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        self.gripper_pub = self.create_publisher(
            GripperCommand,
            '/gripper_controller/commands',
            10
        )
        
        # Subscriber pour boxes détectées
        self.boxes_sub = self.create_subscription(
            String,
            '/detected_boxes',
            self.boxes_callback,
            10
        )
        
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
        
        # Tables de destination (positions depuis warehouse_zones.yaml)
        self.table_positions = {
            'red': {'x': -6.0, 'y': -3.43, 'z': 0.68},
            'blue': {'x': -4.0, 'y': -3.43, 'z': 0.68},
            'yellow': {'x': -2.0, 'y': -3.43, 'z': 0.68}
        }
        
        self.detected_boxes = []
        self.is_processing = False
        
        self.get_logger().info('✅ Pick & Place Controller prêt !')
        self.get_logger().info('   Attente de /detected_boxes...')
    
    def boxes_callback(self, msg):
        """Reçoit les boxes détectées"""
        try:
            self.detected_boxes = json.loads(msg.data)
            self.get_logger().info(f'📦 {len(self.detected_boxes)} boxes détectées')
            
            for box in self.detected_boxes:
                self.get_logger().info(
                    f"   • {box['name']} ({box['color']}) "
                    f"→ ({box['x']:.2f}, {box['y']:.2f}, {box['z']:.2f})"
                )
        except Exception as e:
            self.get_logger().error(f'❌ Erreur parsing boxes: {e}')
    
    def command_callback(self, msg):
        """Reçoit commandes pick/place"""
        if self.is_processing:
            self.get_logger().warn('⚠️  Déjà en train de traiter !')
            return
        
        try:
            command = yaml.safe_load(msg.data)
            action = command.get('action', '')
            
            if action == 'sort_all':
                # Trier toutes les boxes détectées
                self.sort_all_boxes()
                
            elif action == 'pick':
                color = command.get('color', '')
                self.execute_pick_sequence(color)
                
            elif action == 'place':
                color = command.get('color', '')
                self.execute_place_sequence(color)
                
            elif action == 'home':
                self.go_home()
                
        except Exception as e:
            self.get_logger().error(f'❌ Erreur commande: {e}')
    
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
    
    def sort_all_boxes(self):
        """Trie TOUTES les boxes détectées"""
        if not self.detected_boxes:
            self.get_logger().warn('⚠️  Aucune box détectée !')
            self.publish_status('error', 'No boxes detected')
            return
        
        self.is_processing = True
        self.get_logger().info('')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info(f'🔄 TRI DE {len(self.detected_boxes)} BOXES')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('')
        
        try:
            for i, box in enumerate(self.detected_boxes, 1):
                color = box['color']
                name = box['name']
                
                self.get_logger().info(f'📦 Box {i}/{len(self.detected_boxes)}: {name} ({color})')
                
                # 1. PICK
                self.get_logger().info(f'   🦾 Phase 1: PICK')
                success_pick = self.execute_pick_sequence(color)
                
                if not success_pick:
                    self.get_logger().error(f'   ❌ PICK échoué pour {name}')
                    continue
                
                # 2. PLACE
                self.get_logger().info(f'   🦾 Phase 2: PLACE vers table {color}')
                success_place = self.execute_place_sequence(color)
                
                if not success_place:
                    self.get_logger().error(f'   ❌ PLACE échoué pour {name}')
                    continue
                
                self.get_logger().info(f'   ✅ {name} trié avec succès !')
                self.get_logger().info('')
            
            self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self.get_logger().info('✅ TRI TERMINÉ !')
            self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self.publish_status('sort_complete')
            
        except Exception as e:
            self.get_logger().error(f'❌ Erreur tri: {e}')
            self.publish_status('error', str(e))
        
        finally:
            self.is_processing = False
    
    def execute_pick_sequence(self, color):
        """Séquence complète de PICK"""
        try:
            # 1. Ouvrir gripper
            self.get_logger().info('      ✋ Ouverture gripper...')
            self.open_gripper()
            time.sleep(1.0)
            
            # 2. Position approche (au-dessus de la box)
            self.get_logger().info('      📍 Approche box...')
            self.move_arm_to_position(joint1=0.0, joint2=-0.5)
            time.sleep(2.0)
            
            # 3. Descendre pour saisir
            self.get_logger().info('      ⬇️  Descente...')
            self.move_arm_to_position(joint1=0.0, joint2=0.3)
            time.sleep(2.0)
            
            # 4. Fermer gripper (saisir box)
            self.get_logger().info('      ✊ Saisie box...')
            self.close_gripper()
            time.sleep(1.5)
            
            # 5. Remonter avec box
            self.get_logger().info('      ⬆️  Remontée avec box...')
            self.move_arm_to_position(joint1=0.0, joint2=-0.8)
            time.sleep(2.0)
            
            self.get_logger().info('      ✅ PICK réussi')
            return True
            
        except Exception as e:
            self.get_logger().error(f'      ❌ Erreur PICK: {e}')
            return False
    
    def execute_place_sequence(self, color):
        """Séquence complète de PLACE"""
        try:
            # Robot se déplace vers table (fait par Mission Orchestrator)
            # On suppose qu'on est devant la table
            
            # 1. Position approche table
            self.get_logger().info(f'      📍 Approche table {color}...')
            self.move_arm_to_position(joint1=0.0, joint2=-0.5)
            time.sleep(2.0)
            
            # 2. Descendre pour déposer
            self.get_logger().info('      ⬇️  Descente...')
            self.move_arm_to_position(joint1=0.0, joint2=0.2)
            time.sleep(2.0)
            
            # 3. Ouvrir gripper (déposer box)
            self.get_logger().info('      ✋ Dépôt box...')
            self.open_gripper()
            time.sleep(1.5)
            
            # 4. Remonter
            self.get_logger().info('      ⬆️  Remontée...')
            self.move_arm_to_position(joint1=0.0, joint2=-0.8)
            time.sleep(2.0)
            
            self.get_logger().info('      ✅ PLACE réussi')
            return True
            
        except Exception as e:
            self.get_logger().error(f'      ❌ Erreur PLACE: {e}')
            return False
    
    def move_arm_to_position(self, joint1, joint2, duration=2.0):
        """Bouge le bras à une position donnée"""
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2']
        
        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        msg.points.append(point)
        
        self.arm_pub.publish(msg)
    
    def open_gripper(self):
        """Ouvre le gripper"""
        msg = GripperCommand()
        msg.position = 0.04  # Ouvert
        msg.max_effort = 5.0
        
        self.gripper_pub.publish(msg)
    
    def close_gripper(self):
        """Ferme le gripper"""
        msg = GripperCommand()
        msg.position = 0.0  # Fermé
        msg.max_effort = 5.0
        
        self.gripper_pub.publish(msg)
    
    def go_home(self):
        """Position de repos"""
        self.get_logger().info('🏠 Retour home...')
        self.move_arm_to_position(joint1=0.0, joint2=-1.2)
        time.sleep(2.0)
        self.open_gripper()
        self.get_logger().info('✅ Home')
        self.publish_status('home_reached')

def main(args=None):
    rclpy.init(args=args)
    controller = PickPlaceController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
