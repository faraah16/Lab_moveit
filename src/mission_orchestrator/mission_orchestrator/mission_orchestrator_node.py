#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import yaml
import os
from ament_index_python.packages import get_package_share_directory
import math
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
import time

# Importer le Navigation Client
from mission_orchestrator.navigation_client import NavigationClient


class MissionOrchestrator(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        
        self.get_logger().info('🤖 Mission Orchestrator démarré')
        
        # Charger les zones depuis le fichier YAML
        self.zones = {}
        self.load_zones()
        
        # Initialiser le Navigation Client
        self.get_logger().info('🧭 Initialisation du Navigation Client...')
        self.nav_client = NavigationClient()
        
        self.get_logger().info('✅ Mission Orchestrator prêt !')
    
    def load_zones(self):
        """Charge le fichier YAML des zones"""
        try:
            # Chemin vers le fichier de config
            config_path = os.path.join(
                get_package_share_directory('location_manager'),
                'config',
                'warehouse_zones.yaml'
            )
            
            # Lire le fichier YAML
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            # Charger les zones
            if 'zones' in config:
                self.zones = config['zones']
                self.get_logger().info(f'📍 {len(self.zones)} zones chargées')
            else:
                self.get_logger().error('❌ Aucune zone trouvée dans le fichier YAML')
                
        except FileNotFoundError:
            self.get_logger().error(f'❌ Fichier de configuration non trouvé : {config_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur lors du chargement : {str(e)}')
    
    def get_zone_info(self, zone_name):
        """Récupère les informations d'une zone"""
        if zone_name not in self.zones:
            self.get_logger().warn(f'⚠️  Zone inconnue : {zone_name}')
            return None
        
        zone_data = self.zones[zone_name]
        
        # Créer un dictionnaire avec les infos
        zone_info = {
            'name': zone_data.get('name', zone_name),
            'position': zone_data['position'],
            'orientation': zone_data['orientation'],
            'marker_id': zone_data.get('marker_id', -1),
            'function': zone_data.get('function', 'unknown'),
            'description': zone_data.get('description', ''),
            'color': zone_data.get('color', 'none')
        }
        
        return zone_info
    def rotate_towards_goal(self, goal_x, goal_y):
        """
        Fait tourner le robot sur place pour s'orienter vers le goal
        avant de commencer la navigation
        
        Args:
            goal_x, goal_y: Coordonnées du goal
        """
        try:
            # Créer TF buffer pour obtenir position actuelle
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, self)
            
            # Attendre que TF soit prêt
            time.sleep(0.5)
            
            # Obtenir position actuelle du robot
            transform = tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time()
            )
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # Calculer angle vers le goal
            dx = goal_x - current_x
            dy = goal_y - current_y
            target_angle = math.atan2(dy, dx)
            
            # Obtenir orientation actuelle du robot
            q = transform.transform.rotation
            
            # Convertir quaternion en angle (yaw)
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Calculer différence d'angle
            angle_diff = target_angle - current_yaw
            
            # Normaliser entre -pi et pi
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self.get_logger().info(f'🧭 Orientation actuelle: {math.degrees(current_yaw):.1f}°')
            self.get_logger().info(f'🎯 Orientation cible: {math.degrees(target_angle):.1f}°')
            self.get_logger().info(f'🔄 Rotation nécessaire: {math.degrees(angle_diff):.1f}°')
            
            # Si rotation > 20°, faire rotation sur place
            if abs(angle_diff) > 0.35:  # ~20 degrés
                self.get_logger().info(f'   ↻ ROTATION SUR PLACE en cours...')
                
                # Créer publisher pour cmd_vel
                cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
                time.sleep(0.2)  # Laisser le publisher s'initialiser
                
                # Calculer vitesse de rotation (max 0.6 rad/s)
                rotation_speed = 0.4 if angle_diff > 0 else -0.4
                
                # Publier commandes de rotation
                twist = Twist()
                rate = self.create_rate(10)  # 10 Hz
                
                # Rotation progressive
                total_rotated = 0.0
                dt = 0.1  # 10 Hz
                
                while abs(total_rotated) < abs(angle_diff) - 0.1:  # Marge de 0.1 rad
                    twist.linear.x = 0.0
                    twist.angular.z = rotation_speed
                    cmd_vel_pub.publish(twist)
                    
                    total_rotated += abs(rotation_speed * dt)
                    time.sleep(dt)
                
                # Arrêter la rotation
                twist.angular.z = 0.0
                cmd_vel_pub.publish(twist)
                
                self.get_logger().info(f'   ✅ Rotation terminée !')
                time.sleep(0.5)  # Pause pour stabiliser
                
            else:
                self.get_logger().info(f'   ℹ️  Rotation minime, pas de rotation sur place')
            
        except Exception as e:
            self.get_logger().warn(f'⚠️  Erreur lors de la rotation: {e}')
            self.get_logger().warn(f'   → Navigation directe sans rotation préalable')


    def go_to_zone(self, zone_name):
        """
        Navigue vers une zone nommée
        
        Args:
            zone_name: Nom de la zone (ex: 'blue_table', 'charging_zone')
            
        Returns:
            bool: True si succès, False sinon
        """
        self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info(f'🎯 MISSION: Aller à {zone_name}')
        self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        
        # 1. Récupérer les infos de la zone
        self.get_logger().info(f'📍 Récupération des infos de la zone...')
        zone_info = self.get_zone_info(zone_name)
        
        if zone_info is None:
            self.get_logger().error(f'❌ Zone "{zone_name}" inconnue !')
            return False
        
        # NOUVEAU : Appliquer un offset pour les tables
        x = zone_info["position"]["x"]
        y = zone_info["position"]["y"]
        
        # APRÈS (nouveau code)
        # Offset RÉDUIT pour permettre la manipulation
        if zone_info["function"] in ["pick_colored_box", "place_boxes"]:
            # Tables: reculer de 0.2m (20cm) pour approche proche
            y += 0.2
            self.get_logger().info(f'   🔧 Offset navigation: +0.2m sur Y')
            self.get_logger().info(f'   📏 Distance robot→table: ~0.23m')
            
        elif zone_info["function"] == "storage":
            # Caisses: reculer de 0.3m vers l'ouest
            x -= 0.3
            self.get_logger().info(f'   🔧 Offset navigation: -0.3m sur X')


        self.get_logger().info(f'   ✅ Zone trouvée: {zone_info["name"]}')
        self.get_logger().info(f'   📍 Position marqueur: ({zone_info["position"]["x"]:.2f}, {zone_info["position"]["y"]:.2f})')
        self.get_logger().info(f'   📍 Position navigation: ({x:.2f}, {y:.2f})')
        self.get_logger().info(f'   🎯 Marker ID: {zone_info["marker_id"]}')
        self.get_logger().info(f'   ⚙️  Fonction: {zone_info["function"]}')
        # ═══════════════════════════════════════════════════════
        # NOUVEAU : ROTATION SUR PLACE AVANT NAVIGATION
        # ═══════════════════════════════════════════════════════
        self.get_logger().info(f'')
        self.get_logger().info(f'🔄 PHASE 1: Orientation vers le goal')
        self.rotate_towards_goal(x, y)
        # ═══════════════════════════════════════════════════════
        # PHASE 2 : NAVIGATION
        # ═══════════════════════════════════════════════════════
        self.get_logger().info(f'')
        self.get_logger().info(f'🚀 PHASE 2: Navigation vers la position')
        success = self.nav_client.navigate_to_pose(
            x=x,
            y=y,
            z=zone_info["position"]["z"],
            orientation_w=zone_info["orientation"]["w"]
        )
        # 3. Retour du résultat
        if success:
            self.get_logger().info(f'✅ Arrivé à {zone_name} !')
            self.get_logger().info(f'📸 Robot positionné à ~0.2m du marqueur')
            self.get_logger().info(f'💡 Prêt pour manipulation')
            self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            return True
        else:
            self.get_logger().error(f'❌ Échec de la navigation vers {zone_name}')
            self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            return False
        
    def pick_and_place_mission(self, pick_zone, place_zone):
        """
        Mission complète: Pick d'une zone et Place dans une autre
        
        Args:
            pick_zone: Zone où picker (ex: 'blue_table')
            place_zone: Zone où placer (ex: 'depot_table')
            
        Returns:
            bool: True si succès, False sinon
        """
        self.get_logger().info(f'')
        self.get_logger().info(f'╔═══════════════════════════════════════════════════╗')
        self.get_logger().info(f'║     MISSION PICK & PLACE                          ║')
        self.get_logger().info(f'║  Pick from: {pick_zone:20s}               ║')
        self.get_logger().info(f'║  Place to:  {place_zone:20s}               ║')
        self.get_logger().info(f'╚═══════════════════════════════════════════════════╝')
        self.get_logger().info(f'')
        
        # ÉTAPE 1: Aller à la zone de pick
        self.get_logger().info(f'📦 ÉTAPE 1/5: Navigation vers zone de pick')
        if not self.go_to_zone(pick_zone):
            self.get_logger().error(f'❌ MISSION ÉCHOUÉE: Navigation vers {pick_zone} impossible')
            return False
        
        # ÉTAPE 2: Alignement avec ArUco (à implémenter plus tard)
        self.get_logger().info(f'🎯 ÉTAPE 2/5: Alignement précis avec ArUco')
        self.get_logger().info(f'   ⚠️  Non implémenté - Skip pour l\'instant')
        # TODO: align_with_marker(zone_info["marker_id"])
        
        # ÉTAPE 3: Pick avec MoveIt (à implémenter plus tard)
        self.get_logger().info(f'🦾 ÉTAPE 3/5: Pick de l\'objet')
        self.get_logger().info(f'   ⚠️  Non implémenté - Simulation du pick (2s)')
        import time
        time.sleep(2)
        self.get_logger().info(f'   ✅ Objet "attrapé" (simulé)')
        
        # ÉTAPE 4: Aller à la zone de place
        self.get_logger().info(f'🚚 ÉTAPE 4/5: Navigation vers zone de dépôt')
        if not self.go_to_zone(place_zone):
            self.get_logger().error(f'❌ MISSION ÉCHOUÉE: Navigation vers {place_zone} impossible')
            return False
        
        # ÉTAPE 5: Place avec MoveIt (à implémenter plus tard)
        self.get_logger().info(f'🦾 ÉTAPE 5/5: Place de l\'objet')
        self.get_logger().info(f'   ⚠️  Non implémenté - Simulation du place (2s)')
        time.sleep(2)
        self.get_logger().info(f'   ✅ Objet "déposé" (simulé)')
        
        # SUCCESS !
        self.get_logger().info(f'')
        self.get_logger().info(f'╔═══════════════════════════════════════════════════╗')
        self.get_logger().info(f'║          ✅ MISSION RÉUSSIE ! 🎉                  ║')
        self.get_logger().info(f'╚═══════════════════════════════════════════════════╝')
        self.get_logger().info(f'')
        
        return True
    
    def patrol_mission(self, zone_list):
        """
        Mission de patrouille: visite une liste de zones
        
        Args:
            zone_list: Liste de noms de zones
            
        Returns:
            bool: True si succès, False sinon
        """
        self.get_logger().info(f'')
        self.get_logger().info(f'╔═══════════════════════════════════════════════════╗')
        self.get_logger().info(f'║     MISSION PATROUILLE                            ║')
        self.get_logger().info(f'║  Zones: {len(zone_list)} points                          ║')
        self.get_logger().info(f'╚═══════════════════════════════════════════════════╝')
        self.get_logger().info(f'')
        
        for i, zone_name in enumerate(zone_list):
            self.get_logger().info(f'🚶 Point {i+1}/{len(zone_list)}: {zone_name}')
            
            if not self.go_to_zone(zone_name):
                self.get_logger().error(f'❌ PATROUILLE ÉCHOUÉE au point {i+1}')
                return False
            
            # Pause de 3 secondes à chaque point
            self.get_logger().info(f'⏸️  Pause 3 secondes...')
            import time
            time.sleep(3)
        
        self.get_logger().info(f'')
        self.get_logger().info(f'╔═══════════════════════════════════════════════════╗')
        self.get_logger().info(f'║     ✅ PATROUILLE TERMINÉE ! 🎉                   ║')
        self.get_logger().info(f'╚═══════════════════════════════════════════════════╝')
        self.get_logger().info(f'')
        
        return True


def main(args=None):
    rclpy.init(args=args)
    orchestrator = MissionOrchestrator()
    
    try:
        # Garder le node actif
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()