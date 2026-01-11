#!/usr/bin/env python3

import yaml
from datetime import datetime
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
import subprocess
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
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
        self.nav_client = NavigationClient(orchestrator=self)

        # ═══════════════════════════════════════════════════════
        # PICK & PLACE INTERFACE
        # ═══════════════════════════════════════════════════════
        self.pick_place_command_pub = self.create_publisher(
            String,
            '/pick_place/command',
            10
        )
        
        self.pick_place_status = 'idle'
        self.pick_place_status_sub = self.create_subscription(
            String,
            '/pick_place/status',
            self.pick_place_status_callback,
            10
        )
        
        self.detected_boxes = []
        self.detected_boxes_sub = self.create_subscription(
            String,
            '/detected_boxes',
            self.detected_boxes_callback,
            10
        )
        
        self.get_logger().info('🦾 Pick & Place interface initialisée')
        # ═══════════════════════════════════════════════════════
        # NOUVEAU : TF Buffer PERSISTANT
        # ═══════════════════════════════════════════════════════
        self.get_logger().info('🔧 Initialisation du TF Buffer...')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # ═══════════════════════════════════════════════════════
        # GESTION BATTERIE
        # ═══════════════════════════════════════════════════════
        self.battery_level = 100.0
        self.current_zone = 'start_stop_zone'
        self.low_battery_mode = False
        self.charging_complete = False
        self.navigation_in_progress = False  # ← NOUVEAU : pour surveiller navigation
        
        # Publisher pour zone actuelle
        self.zone_pub = self.create_publisher(
            String,
            '/current_zone',
            10
        )
        
        # Subscriber batterie
        self.battery_sub = self.create_subscription(
            Float32,
            '/battery_status',
            self.battery_callback,
            10
        )
        
        self.battery_alert_sub = self.create_subscription(
            String,
            '/battery_alert',
            self.battery_alert_callback,
            10
        )
        # ═══════════════════════════════════════════════════════
        # SUBSCRIBER POUR MISSIONS WAREHOUSE
        # ═══════════════════════════════════════════════════════
        self.mission_sub = self.create_subscription(
            String,
            '/warehouse/execute_mission',
            self.mission_callback,
            10
        )
        
        # Publisher pour feedback missions
        self.mission_feedback_pub = self.create_publisher(
            String,
            '/warehouse/mission_feedback',
            10
        )
        
        self.get_logger().info('📦 Abonnement aux missions warehouse activé')
        # ═══════════════════════════════════════════════════════
        # TIMER DE SURVEILLANCE BATTERIE (toutes les 1 seconde)
        # ═══════════════════════════════════════════════════════
        self.battery_check_timer = self.create_timer(
            1.0,  # Vérification toutes les 1 seconde
            self.periodic_battery_check
)
        self.get_logger().info('🔋 Gestion batterie activée + surveillance 1Hz')
        self.get_logger().info('✅ TF Buffer initialisé')
        


        self.get_logger().info('✅ Mission Orchestrator prêt !')
    def battery_callback(self, msg):
        """Mise à jour niveau batterie"""
        self.battery_level = msg.data
    
    def battery_alert_callback(self, msg):
        """Gestion alertes batterie"""
        if msg.data == 'LOW_BATTERY':
            self.get_logger().warn('⚠️  BATTERIE FAIBLE ! Mode LOW_BATTERY activé')
            self.low_battery_mode = True
            self.charging_complete = False
            
        elif msg.data == 'CHARGED':
            self.get_logger().info('✅ BATTERIE CHARGÉE ! Prêt à reprendre')
            self.charging_complete = True
            self.low_battery_mode = False
    
    def periodic_battery_check(self):
        """
        Vérification périodique de la batterie (appelée toutes les 1 seconde)
        Annule la navigation en cours si batterie faible
        """
        # ⭐⭐⭐ LOG DE DEBUG PERMANENT (même si pas en navigation)
        self.get_logger().info(
            f'🔍 [BATTERY_CHECK] '
            f'level={self.battery_level:.1f}% | '
            f'low_mode={self.low_battery_mode} | '
            f'navigating={self.navigation_in_progress} | '
            f'zone={self.current_zone}'
        )
    
        # Vérifier si batterie faible ET navigation en cours ET pas déjà vers charging
        if (self.low_battery_mode and 
            self.navigation_in_progress and 
            self.current_zone != 'charging_zone'):
            
            self.get_logger().warn('')
            self.get_logger().warn('🚨🚨🚨 BATTERIE FAIBLE PENDANT NAVIGATION ! 🚨🚨🚨')
            self.get_logger().warn(f'   Niveau: {self.battery_level:.1f}%')
            self.get_logger().warn(f'   Zone: {self.current_zone}')
            self.get_logger().warn('   → INTERRUPTION navigation en cours')
            self.get_logger().warn('')
            
            # Annuler le goal de navigation
            try:
                self.nav_client.cancel_goal()
                self.get_logger().info('   ✅ Goal annulé')
            except Exception as e:
                self.get_logger().warn(f'   ⚠️  Erreur annulation: {e}')
            
            # Marquer navigation terminée
            self.navigation_in_progress = False
            
            # Attendre stabilisation
            time.sleep(2.0)
            
            # Aller charger IMMÉDIATEMENT
            self.get_logger().warn('   🔌 Navigation PRIORITAIRE vers charging_zone...')
            self.go_to_zone('charging_zone')
    
    def pick_place_status_callback(self, msg):
        """Reçoit status du pick & place"""
        try:
            status_data = yaml.safe_load(msg.data)
            self.pick_place_status = status_data.get('status', 'unknown')
            
            if self.pick_place_status == 'error':
                self.get_logger().error(f"❌ Pick&Place error: {status_data.get('message', '')}")
        except:
            pass
    
    def detected_boxes_callback(self, msg):
        """Reçoit boxes détectées"""
        try:
            import json
            self.detected_boxes = json.loads(msg.data)
            self.get_logger().info(f'📦 {len(self.detected_boxes)} boxes reçues par orchestrator')
        except:
            pass
    
    def send_pick_command(self, color):
        """Envoie commande PICK et attend complétion"""
        self.get_logger().info(f'   📤 Commande PICK ({color})...')
        
        command = {
            'action': 'pick',
            'color': color
        }
        
        msg = String()
        msg.data = yaml.dump(command)
        self.pick_place_command_pub.publish(msg)
        
        # Attendre complétion (max 30s)
        self.pick_place_status = 'picking'
        timeout = 30
        start_time = time.time()
        
        while self.pick_place_status == 'picking':
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
            if time.time() - start_time > timeout:
                self.get_logger().error('   ❌ Timeout PICK')
                return False
        
        if self.pick_place_status == 'pick_complete':
            self.get_logger().info('   ✅ PICK réussi')
            return True
        else:
            self.get_logger().error(f'   ❌ PICK échoué: {self.pick_place_status}')
            return False
    
    def send_place_command(self, color):
        """Envoie commande PLACE et attend complétion"""
        self.get_logger().info(f'   📤 Commande PLACE ({color})...')
        
        command = {
            'action': 'place',
            'color': color
        }
        
        msg = String()
        msg.data = yaml.dump(command)
        self.pick_place_command_pub.publish(msg)
        
        # Attendre complétion (max 30s)
        self.pick_place_status = 'placing'
        timeout = 30
        start_time = time.time()
        
        while self.pick_place_status == 'placing':
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
            if time.time() - start_time > timeout:
                self.get_logger().error('   ❌ Timeout PLACE')
                return False
        
        if self.pick_place_status == 'place_complete':
            self.get_logger().info('   ✅ PLACE réussi')
            return True
        else:
            self.get_logger().error(f'   ❌ PLACE échoué: {self.pick_place_status}')
            return False
    def publish_current_zone(self, zone_name):
        """Publie la zone actuelle"""
        self.current_zone = zone_name
        msg = String()
        msg.data = zone_name
        self.zone_pub.publish(msg)
        self.get_logger().info(f'📍 Zone actuelle: {zone_name}')
    
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
        Fait tourner le robot sur place TRÈS PRÉCISÉMENT pour s'orienter vers le goal
        AMÉLIORATION: Rotation en boucle fermée avec feedback TF en temps réel
        + Attente robuste de la disponibilité de TF
        
        Args:
            goal_x, goal_y: Coordonnées du goal
        """
        try:
            # Créer TF buffer PERSISTENT pour feedback en temps réel
            tf_buffer = self.tf_buffer
            
            # ═══════════════════════════════════════════════════════
            # ATTENDRE QUE TF SOIT PRÊT (CRITIQUE!)
            # ═══════════════════════════════════════════════════════
            self.get_logger().info(f'   ⏳ Attente initialisation TF...')
            time.sleep(2.0)  # Délai initial augmenté
            
            # Vérifier que la frame "map" existe avec plusieurs tentatives
            max_attempts = 20
            tf_ready = False
            
            for attempt in range(max_attempts):
                try:
                    # Test de disponibilité de la frame map -> base_link
                    test_transform = tf_buffer.lookup_transform(
                        'map', 
                        'base_link', 
                        rclpy.time.Time(seconds=0),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    # Si on arrive ici, TF est OK
                    self.get_logger().info(f'   ✅ TF prêt! (frame map→base_link disponible)')
                    tf_ready = True
                    break
                    
                except Exception as e:
                    if attempt < max_attempts - 1:
                        self.get_logger().info(
                            f'   ⏳ TF pas encore prêt, attente... '
                            f'({attempt+1}/{max_attempts}) - {str(e)[:50]}'
                        )
                        time.sleep(1.0)
                    else:
                        # Dernier essai échoué
                        raise Exception(
                            f"TF map→base_link non disponible après {max_attempts}s. "
                            f"AMCL n'est probablement pas encore localisé."
                        )
            
            if not tf_ready:
                raise Exception("TF non disponible")
            
            # ═══════════════════════════════════════════════════════
            # FONCTION HELPER: Obtenir l'angle actuel du robot
            # ═══════════════════════════════════════════════════════
            def get_current_yaw():
                """Récupère l'orientation actuelle du robot"""
                try:
                    transform = tf_buffer.lookup_transform(
                        'map', 
                        'base_link', 
                        rclpy.time.Time(seconds=0)
                    )
                    q = transform.transform.rotation
                    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                    return math.atan2(siny_cosp, cosy_cosp)
                except:
                    return None
            
            # ═══════════════════════════════════════════════════════
            # ÉTAPE 1: Calculer angle cible
            # ═══════════════════════════════════════════════════════
            transform = tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(seconds=0)
            )
            
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            # Calculer angle vers le goal
            dx = goal_x - current_x
            dy = goal_y - current_y
            target_angle = math.atan2(dy, dx)
            
            # Orientation actuelle
            current_yaw = get_current_yaw()
            
            # Différence d'angle
            angle_diff = target_angle - current_yaw
            
            # Normaliser entre -pi et pi
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            self.get_logger().info(f'🧭 Orientation actuelle: {math.degrees(current_yaw):.1f}°')
            self.get_logger().info(f'🎯 Orientation cible: {math.degrees(target_angle):.1f}°')
            self.get_logger().info(f'🔄 Rotation nécessaire: {math.degrees(angle_diff):.1f}°')
            
            # ═══════════════════════════════════════════════════════
            # ÉTAPE 2: Rotation RIGIDE sur place (si > 10°)
            # ═══════════════════════════════════════════════════════
            if abs(angle_diff) > 0.175:  # ~10 degrés
                self.get_logger().info(f'   ↻ ROTATION RIGIDE SUR PLACE...')
                
                # ═══════════════════════════════════════════════════════
                # DÉSACTIVER LE CONTROLLER NAV2
                # ═══════════════════════════════════════════════════════
                self.get_logger().info(f'   ⏸️  Pause du controller Nav2...')
                try:
                    result = subprocess.run(
                        ['ros2', 'lifecycle', 'set', '/controller_server', 'deactivate'],
                        capture_output=True,
                        timeout=5
                    )
                    if result.returncode == 0:
                        self.get_logger().info(f'   ✅ Controller Nav2 désactivé')
                except Exception as e:
                    self.get_logger().warn(f'   ⚠️  Erreur désactivation: {e}')
                
                time.sleep(0.5)
                
                # ═══════════════════════════════════════════════════════
                # CRÉER PUBLISHER AVEC QoS EXACT
                # ═══════════════════════════════════════════════════════
                from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
                
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1
                )
                
                cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', qos)
                self.get_logger().info(f'   📡 Publisher créé (RELIABLE/VOLATILE/depth=1)')
                
                time.sleep(1.5)
                
                subscriber_count = cmd_vel_pub.get_subscription_count()
                self.get_logger().info(f'   📊 Subscribers: {subscriber_count}')
                                
                # PHASE 1: Rotation avec feedback
                self.get_logger().info(f'   📍 Phase 1: Rotation avec feedback temps réel')

                tolerance = 0.05
                max_iterations = 2000
                iteration = 0

                while iteration < max_iterations:
                    # ⭐ CRITIQUE : Spinner pour recevoir messages TF
                    rclpy.spin_once(self, timeout_sec=0.01)
                    # ═══════════════════════════════════════════════════════
                    # VÉRIFICATION BATTERIE PENDANT ROTATION
                    # ═══════════════════════════════════════════════════════
                    if iteration % 50 == 0:  # Vérifier toutes les 50 itérations (~2s)
                        if self.low_battery_mode:
                            self.get_logger().warn('🚨 BATTERIE FAIBLE PENDANT ROTATION !')
                            self.get_logger().warn('   → Arrêt rotation et navigation vers charging')
                            # Arrêt immédiat
                            twist = Twist()
                            for _ in range(5):
                                cmd_vel_pub.publish(twist)
                                time.sleep(0.05)
                            # Réactiver Nav2
                            try:
                                subprocess.run(
                                    ['ros2', 'lifecycle', 'set', '/controller_server', 'activate'],
                                    capture_output=True, timeout=5
                                )
                            except:
                                pass
                            # Sortir de la fonction rotate
                            return
                    current_yaw = get_current_yaw()
                    if current_yaw is None:
                        break
                    
                    angle_error = target_angle - current_yaw
                    while angle_error > math.pi:
                        angle_error -= 2 * math.pi
                    while angle_error < -math.pi:
                        angle_error += 2 * math.pi
                    
                    if abs(angle_error) < tolerance:
                        self.get_logger().info(f'   ✅ Cible atteinte! Erreur: {math.degrees(angle_error):.2f}°')
                        break
                                        
                    speed_factor = max(0.4, min(1.0, abs(angle_error) / 0.5))
                    rotation_speed = 1.2 * speed_factor  # 1.2 rad/s max (50% plus rapide!)
                    
                    if angle_error < 0:
                        rotation_speed = -rotation_speed
                    
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = rotation_speed
                    cmd_vel_pub.publish(twist)
                    # Log détaillé
                    if iteration % 20 == 0:
                        self.get_logger().info(
                            f'      Itération {iteration}: '
                            f'Angle: {math.degrees(current_yaw):+.1f}° | '
                            f'Erreur: {math.degrees(angle_error):+.1f}° | '
                            f'Cmd: {rotation_speed:+.2f} rad/s')
                    
                    iteration += 1
                    time.sleep(0.04)  # Réduit à 40ms pour compenser le spin_once    
                # PHASE 2: Arrêt
                self.get_logger().info(f'   🛑 Arrêt')
                twist = Twist()
                for _ in range(10):
                    cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                
                self.get_logger().info(f'   ✅ Rotation terminée!')

                # ═══════════════════════════════════════════════════════
                # VÉRIFICATION FINALE ET CORRECTION SI NÉCESSAIRE
                # ═══════════════════════════════════════════════════════
                time.sleep(0.5)
                final_yaw = get_current_yaw()

                if final_yaw is not None:
                    final_error = target_angle - final_yaw
                    while final_error > math.pi:
                        final_error -= 2 * math.pi
                    while final_error < -math.pi:
                        final_error += 2 * math.pi
                    
                    self.get_logger().info(f'   📊 Vérification finale:')
                    self.get_logger().info(f'      Orientation cible: {math.degrees(target_angle):.1f}°')
                    self.get_logger().info(f'      Orientation finale: {math.degrees(final_yaw):.1f}°')
                    self.get_logger().info(f'      Erreur: {math.degrees(final_error):.1f}°')
                    
                    # ⭐ TOLERANCE STRICTE : 3° maximum
                    strict_tolerance = 0.05  # 3°
                    
                    if abs(final_error) > strict_tolerance:
                        # ❌ ERREUR TROP GRANDE → CORRECTION !
                        self.get_logger().warn(f'   ⚠️  ERREUR > 3° ! Rotation corrective...')
                        
                        # ROTATION CORRECTIVE (même mécanisme mais plus court)
                        max_correction_iterations = 800
                        correction_iteration = 0
                        
                        while correction_iteration < max_correction_iterations:
                            rclpy.spin_once(self, timeout_sec=0.01)
                            
                            current_yaw = get_current_yaw()
                            if current_yaw is None:
                                break
                            
                            angle_error = target_angle - current_yaw
                            while angle_error > math.pi:
                                angle_error -= 2 * math.pi
                            while angle_error < -math.pi:
                                angle_error += 2 * math.pi
                            
                            # ✅ ARRÊT si < 3°
                            if abs(angle_error) < strict_tolerance:
                                self.get_logger().info(f'   ✅ Correction réussie! Erreur: {math.degrees(angle_error):.2f}°')
                                break
                            
                            # Rotation douce
                            speed_factor = max(0.3, min(1.0, abs(angle_error) / 0.3))
                            rotation_speed = 0.6 * speed_factor  # Plus doux que la rotation principale
                            
                            if angle_error < 0:
                                rotation_speed = -rotation_speed
                            
                            twist = Twist()
                            twist.angular.z = rotation_speed
                            cmd_vel_pub.publish(twist)
                            
                            if correction_iteration % 20 == 0:
                                self.get_logger().info(
                                    f'      [Correction {correction_iteration}] Erreur: {math.degrees(angle_error):+.1f}°'
                                )
                            
                            correction_iteration += 1
                            time.sleep(0.04)
                        
                        # Arrêt après correction
                        twist = Twist()
                        for _ in range(10):
                            cmd_vel_pub.publish(twist)
                            time.sleep(0.05)
                        
                        # Vérification APRÈS correction
                        time.sleep(0.3)
                        final_yaw = get_current_yaw()
                        if final_yaw is not None:
                            final_error = target_angle - final_yaw
                            while final_error > math.pi:
                                final_error -= 2 * math.pi
                            while final_error < -math.pi:
                                final_error += 2 * math.pi
                            
                            self.get_logger().info(f'   📊 Après correction:')
                            self.get_logger().info(f'      Erreur finale: {math.degrees(final_error):.1f}°')
                            
                            if abs(final_error) < 0.08:  # 4.5°
                                self.get_logger().info(f'   ✅ Rotation PRÉCISE atteinte !')
                            else:
                                self.get_logger().warn(f'   ⚠️  Rotation imprécise malgré correction')
                    else:
                        # ✅ ERREUR < 3° dès le premier coup
                        self.get_logger().info(f'   ✅ Rotation PRÉCISE ! (erreur < 3°)')

                # ═══════════════════════════════════════════════════════
                # RÉACTIVER NAV2
                # ═══════════════════════════════════════════════════════
                self.get_logger().info(f'   ▶️  Réactivation Nav2...')
                try:
                    subprocess.run(
                        ['ros2', 'lifecycle', 'set', '/controller_server', 'activate'],
                        capture_output=True,
                        timeout=5
                    )
                    self.get_logger().info(f'   ✅ Nav2 réactivé')
                except:
                    pass
                
                time.sleep(2.0)
                try:
                    result = subprocess.run(
                        ['ros2', 'lifecycle', 'get', '/controller_server'],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    state = result.stdout.strip()
                    self.get_logger().info(f'   📊 État controller_server: {state}')
                except:
                    pass

            else:
                self.get_logger().info(f'   ℹ️  Rotation < 10°, skip')
                        
        except Exception as e:
            self.get_logger().warn(f'⚠️  Erreur lors de la rotation: {e}')
            self.get_logger().warn(f'   → Navigation directe sans rotation préalable')


            
    def go_to_zone_with_waypoint(self, zone_name):
        """
        Navigation avec waypoint intermédiaire pour chemin en L
        VERSION CORRIGÉE : Waypoint avec marge de sécurité
        """
        self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info(f'🎯 MISSION WAYPOINT: Aller à {zone_name}')
        self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        
        # Récupérer infos zone
        zone_info = self.get_zone_info(zone_name)
        if zone_info is None:
            self.get_logger().error(f'❌ Zone "{zone_name}" inconnue !')
            return False
        
        # Appliquer offset
        goal_x = zone_info["position"]["x"]
        goal_y = zone_info["position"]["y"]
        
        if zone_info["function"] in ["pick_colored_box", "place_boxes"]:
            goal_y += 0.2
            self.get_logger().info(f'   🔧 Offset: +0.2m sur Y')
        elif zone_info["function"] == "storage":
            goal_x -= 0.3
            self.get_logger().info(f'   🔧 Offset: -0.3m sur X')
        
        # Position actuelle
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time()
            )
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
            
            self.get_logger().info(f'📍 Position actuelle: ({current_x:.2f}, {current_y:.2f})')
            self.get_logger().info(f'🎯 Goal final: ({goal_x:.2f}, {goal_y:.2f})')
            
        except Exception as e:
            self.get_logger().warn(f'⚠️  Erreur TF, navigation directe: {e}')
            return self.go_to_zone(zone_name)
        
        # ═══════════════════════════════════════════════════════
        # CALCULER WAYPOINT AVEC MARGE DE SÉCURITÉ
        # ═══════════════════════════════════════════════════════
        import math
        
        safety_margin = 1.0  # 1 mètre de marge
        
        # Calculer direction verticale
        dy = goal_y - current_y
        
        if abs(dy) < 0.5:
            # Si déplacement vertical trop petit, navigation directe
            self.get_logger().info('ℹ️  Déplacement vertical < 0.5m, navigation directe')
            return self.go_to_zone(zone_name)
        
        # Waypoint : 1m AVANT le goal sur l'axe Y
        waypoint_x = current_x
        
        if dy > 0:
            # Goal au nord : waypoint 1m avant (plus au sud)
            waypoint_y = goal_y - safety_margin
        else:
            # Goal au sud : waypoint 1m avant (plus au nord)
            waypoint_y = goal_y + safety_margin
        
        # Vérifier que waypoint n'est pas trop proche de la position actuelle
        distance_to_waypoint = math.sqrt(
            (waypoint_x - current_x)**2 + (waypoint_y - current_y)**2
        )
        
        if distance_to_waypoint < 0.5:
            self.get_logger().info('ℹ️  Waypoint trop proche, navigation directe')
            return self.go_to_zone(zone_name)
        
        self.get_logger().info(f'')
        self.get_logger().info(f'🔄 NAVIGATION EN 2 ÉTAPES (chemin en L)')
        self.get_logger().info(f'   🛡️  Marge sécurité: {safety_margin}m')
        self.get_logger().info(f'   📍 Waypoint sécurisé: ({waypoint_x:.2f}, {waypoint_y:.2f})')
        self.get_logger().info(f'   🎯 Goal final: ({goal_x:.2f}, {goal_y:.2f})')
        self.get_logger().info(f'')
        
        # ═══════════════════════════════════════════════════════
        # ÉTAPE 1 : Navigation vers waypoint (vertical)
        # ═══════════════════════════════════════════════════════
        self.get_logger().info(f'🎯 ÉTAPE 1/2: Navigation vers waypoint (vertical)')
        
        self.rotate_towards_goal(waypoint_x, waypoint_y)
        
        # Clear costmaps
        self.get_logger().info(f'   🧹 Nettoyage costmaps...')
        try:
            import subprocess
            subprocess.run(
                ['ros2', 'service', 'call', '/local_costmap/clear_entirely_local_costmap',
                'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True, timeout=2
            )
            subprocess.run(
                ['ros2', 'service', 'call', '/global_costmap/clear_entirely_global_costmap',
                'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True, timeout=2
            )
        except:
            pass
        
        time.sleep(0.5)
        
        success1 = self.nav_client.navigate_to_pose(
            x=waypoint_x,
            y=waypoint_y,
            z=0.0,
            orientation_w=1.0,
            is_charging_mission=False  # ← AJOUTER
        )
        
        if not success1:
            self.get_logger().warn('⚠️  Échec waypoint, tentative navigation directe')
            return self.go_to_zone(zone_name)
        
        self.get_logger().info('✅ Waypoint atteint !')
        time.sleep(3)  # ← Augmenter à 3s pour stabilisation
        
        # ═══════════════════════════════════════════════════════
        # ÉTAPE 2 : Navigation vers goal (horizontal)
        # ═══════════════════════════════════════════════════════
        self.get_logger().info(f'')
        self.get_logger().info(f'🎯 ÉTAPE 2/2: Navigation vers goal (horizontal)')
        
        self.rotate_towards_goal(goal_x, goal_y)
        
        # Clear costmaps
        self.get_logger().info(f'   🧹 Nettoyage costmaps...')
        try:
            subprocess.run(
                ['ros2', 'service', 'call', '/local_costmap/clear_entirely_local_costmap',
                'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True, timeout=2
            )
            subprocess.run(
                ['ros2', 'service', 'call', '/global_costmap/clear_entirely_global_costmap',
                'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True, timeout=2
            )
        except:
            pass
        
        time.sleep(0.5)
        self.navigation_in_progress = True
        success2 = self.nav_client.navigate_to_pose(
            x=goal_x,
            y=goal_y,
            z=zone_info["position"]["z"],
            orientation_w=zone_info["orientation"]["w"],
            is_charging_mission=False  # ← AJOUTER
        )
        self.navigation_in_progress = False
        if success2:
            self.get_logger().info(f'✅ Arrivé à {zone_name} via waypoint !')
            self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            return True
        else:
            self.get_logger().error(f'❌ Échec navigation vers {zone_name}')
            self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            return False


    def check_battery_and_charge(self):
        """
        Vérifie batterie et va charger si nécessaire
        Returns:
            bool: True si besoin de charger, False sinon
        """
        if self.low_battery_mode and not self.charging_complete:
            self.get_logger().warn('')
            self.get_logger().warn('🔋🔋🔋 BATTERIE FAIBLE DÉTECTÉE 🔋🔋🔋')
            self.get_logger().warn(f'   Niveau actuel: {self.battery_level:.1f}%')
            self.get_logger().warn('   → Navigation PRIORITAIRE vers charging_zone')
            self.get_logger().warn('')
            
            # Aller charger
            success = self.go_to_zone('charging_zone')
            
            if success:
                self.get_logger().info('🔌 Arrivé à la charging_zone')
                self.get_logger().info('⏳ Attente charge complète...')
                
                # Attendre que batterie soit chargée
                while not self.charging_complete:
                    rclpy.spin_once(self, timeout_sec=1.0)
                    if self.battery_level >= 95.0:
                        break
                    time.sleep(5)
                
                self.get_logger().info('✅ Charge terminée ! Reprise des missions.')
                return True
            else:
                self.get_logger().error('❌ Échec navigation vers charging_zone !')
                return False
        
        return False        
    
    def mission_callback(self, msg):
        """Reçoit et exécute les missions du warehouse"""
        try:
            mission = yaml.safe_load(msg.data)
            mission_type = mission.get('type', 'unknown')
            description = mission.get('description', 'Mission inconnue')
            
            self.get_logger().info('')
            self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self.get_logger().info(f'📦 MISSION REÇUE: {description}')
            self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self.get_logger().info('')
            
            success = False
            
            # ═══════════════════════════════════════════════════════
            # EXÉCUTION SELON TYPE DE MISSION
            # ═══════════════════════════════════════════════════════
            
            if mission_type == 'arrival' or mission_type == 'arrival_for_order':
                # Mission de tri d'arrivage
                success = self.execute_arrival_mission(mission)
            
            elif mission_type == 'order':
                # Mission de commande client
                success = self.execute_order_mission(mission)
            
            elif mission_type == 'stop':
                # Arrêt soft
                success = self.execute_stop_mission(mission)
            
            elif mission_type == 'emergency_stop':
                # Arrêt d'urgence
                success = self.execute_emergency_stop(mission)
            
            else:
                self.get_logger().warn(f'⚠️  Type de mission inconnu: {mission_type}')
                success = False
            
            # ═══════════════════════════════════════════════════════
            # ENVOYER FEEDBACK
            # ═══════════════════════════════════════════════════════
            feedback = {
                'mission_type': mission_type,
                'description': description,
                'success': success,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            
            feedback_msg = String()
            feedback_msg.data = yaml.dump(feedback)
            self.mission_feedback_pub.publish(feedback_msg)
            
            if success:
                self.get_logger().info('')
                self.get_logger().info('✅ MISSION TERMINÉE AVEC SUCCÈS')
                self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                self.get_logger().info('')
            else:
                self.get_logger().error('')
                self.get_logger().error('❌ MISSION ÉCHOUÉE')
                self.get_logger().error('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
                self.get_logger().error('')
        
        except Exception as e:
            self.get_logger().error(f'❌ Erreur traitement mission: {e}')
    
    def execute_arrival_mission(self, mission):
        """Exécute une mission de tri d'arrivage AVEC PICK & PLACE RÉEL"""
        self.get_logger().info('')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('📦 MISSION: Tri d\'arrivage RÉEL')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('')
        
        # 1. Navigation vers depot_table
        self.get_logger().info('🚗 ÉTAPE 1: Navigation vers depot_table')
        success = self.go_to_zone('depot_table')
        
        if not success:
            self.get_logger().error('❌ Échec navigation vers depot_table')
            return False
        
        self.get_logger().info('✅ Arrivé au depot_table')
        time.sleep(2.0)
        
        # 2. Attendre détection des boxes
        self.get_logger().info('🔍 ÉTAPE 2: Attente détection boxes...')
        timeout = 10
        start = time.time()
        
        while len(self.detected_boxes) == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
            
            if time.time() - start > timeout:
                self.get_logger().error('❌ Timeout détection boxes')
                return False
        
        self.get_logger().info(f'✅ {len(self.detected_boxes)} boxes détectées')
        
        # 3. Trier chaque box
        self.get_logger().info('')
        self.get_logger().info('🦾 ÉTAPE 3: Tri des boxes')
        self.get_logger().info('')
        
        for i, box in enumerate(self.detected_boxes, 1):
            color = box['color']
            name = box['name']
            
            self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self.get_logger().info(f'📦 Box {i}/{len(self.detected_boxes)}: {name} ({color})')
            self.get_logger().info(f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            
            # a) PICK
            self.get_logger().info('🦾 Phase A: PICK')
            success_pick = self.send_pick_command(color)
            
            if not success_pick:
                self.get_logger().error(f'❌ PICK échoué pour {name}')
                continue
            
            # b) NAVIGATION vers table couleur
            table_name = f'{color}_table'
            self.get_logger().info(f'🚗 Phase B: Navigation vers {table_name}')
            success_nav = self.go_to_zone(table_name)
            
            if not success_nav:
                self.get_logger().error(f'❌ Navigation échouée vers {table_name}')
                continue
            
            time.sleep(1.0)
            
            # c) PLACE
            self.get_logger().info('🦾 Phase C: PLACE')
            success_place = self.send_place_command(color)
            
            if not success_place:
                self.get_logger().error(f'❌ PLACE échoué pour {name}')
                continue
            
            self.get_logger().info(f'✅ {name} trié avec succès !')
            self.get_logger().info('')
            
            # d) RETOUR vers depot_table pour prochaine box
            if i < len(self.detected_boxes):
                self.get_logger().info('🔄 Retour vers depot_table...')
                self.go_to_zone('depot_table')
                time.sleep(1.0)
        
        self.get_logger().info('')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('✅ TRI D\'ARRIVAGE TERMINÉ !')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('')
        
        return True
    
    def execute_order_mission(self, mission):
        """Exécute une mission de commande client"""
        order = mission.get('data', {})
        order_number = order.get('number', 'N/A')
        items = order.get('items', {})
        destination = order.get('destination', 'yellow_crate_left')
        
        self.get_logger().info(f'🎯 MISSION: Commande {order_number}')
        self.get_logger().info(f'   Destination: {destination}')
        
        # Pour chaque item à récupérer
        for color, qty in items.items():
            if qty > 0:
                # Déterminer la table source
                if 'red' in color:
                    table = 'red_table'
                elif 'blue' in color:
                    table = 'blue_table'
                elif 'yellow' in color:
                    table = 'yellow_table'
                else:
                    continue
                
                color_name = color.replace('_boxes', '').title()
                
                self.get_logger().info(f'')
                self.get_logger().info(f'📦 Récupération: {qty}x {color_name}')
                self.get_logger().info(f'   → Navigation vers {table}')
                
                # Aller à la table
                success = self.go_to_zone(table)
                
                if not success:
                    self.get_logger().error(f'❌ Échec navigation vers {table}')
                    return False
                
                self.get_logger().info(f'✅ Arrivé à {table}')
                
                # Simuler pick
                self.get_logger().info(f'🦾 Pick de {qty} box(es) en cours...')
                import time
                time.sleep(3)
                self.get_logger().info(f'✅ {qty} box(es) récupérée(s)')
        
        # Aller à la destination
        self.get_logger().info('')
        self.get_logger().info(f'🚚 Transport vers destination')
        self.get_logger().info(f'   → Navigation vers {destination}')
        
        success = self.go_to_zone(destination)
        
        if not success:
            self.get_logger().error(f'❌ Échec navigation vers {destination}')
            return False
        
        self.get_logger().info(f'✅ Arrivé à {destination}')
        
        # Simuler place
        self.get_logger().info('🦾 Dépôt des boxes en cours...')
        import time
        time.sleep(3)
        
        self.get_logger().info(f'✅ Commande {order_number} préparée !')
        
        return True
    
    def execute_stop_mission(self, mission):
        """Exécute un arrêt soft (retour start/stop)"""
        self.get_logger().info('🛑 MISSION: Arrêt SOFT')
        self.get_logger().info('   → Retour à start_stop_zone')
        
        success = self.go_to_zone('start_stop_zone')
        
        if success:
            self.get_logger().info('✅ Robot arrêté à start_stop_zone')
            self.get_logger().info('💡 Prêt pour la prochaine mission')
        else:
            self.get_logger().error('❌ Échec retour à start_stop_zone')
        
        return success
    
    def execute_emergency_stop(self, mission):
        """Exécute un arrêt d'urgence"""
        self.get_logger().error('🚨 MISSION: ARRÊT D\'URGENCE')
        self.get_logger().error('   → Retour IMMÉDIAT à start_stop_zone')
        
        # Annuler toute navigation en cours
        try:
            self.nav_client.cancel_goal()
            self.get_logger().warn('   ⚠️  Navigation annulée')
        except:
            pass
        
        # Navigation immédiate
        success = self.go_to_zone('start_stop_zone')
        
        if success:
            self.get_logger().info('✅ Robot arrêté (urgence)')
        else:
            self.get_logger().error('❌ Échec arrêt d\'urgence')
        
        return success

    def go_to_zone(self, zone_name):
        """
        Navigue vers une zone nommée
        
        Args:
            zone_name: Nom de la zone (ex: 'blue_table', 'charging_zone')
            
        Returns:
            bool: True si succès, False sinon
        """
        # ═══════════════════════════════════════════════════════
        # VÉRIFICATION BATTERIE (sauf si on va déjà charger !)
        # ═══════════════════════════════════════════════════════
        if zone_name != 'charging_zone':  # ← CRITIQUE : éviter récursion
            if self.check_battery_and_charge():
                self.get_logger().info('✅ Batterie rechargée, reprise de la mission')
        
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
        if zone_name == 'depot_table':
            # ⭐ DEPOT: Approche modérée (compromis distance/nav)
            y -= 1.2  # 30cm plus proche
            self.get_logger().info(f'   🔧 Offset DEPOT: -0.3m sur Y')
            self.get_logger().info(f'   📏 Position: ({x:.2f}, {y:.2f})')
            
        elif zone_info["function"] in ["pick_colored_box", "place_boxes"]:
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

        # NOUVEAU : Clear costmaps avant navigation
        self.get_logger().info(f'   🧹 Nettoyage des costmaps...')
        try:
            subprocess.run(
                ['ros2', 'service', 'call', '/local_costmap/clear_entirely_local_costmap', 
                'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True,
                timeout=2
            )
            subprocess.run(
                ['ros2', 'service', 'call', '/global_costmap/clear_entirely_global_costmap',
                'nav2_msgs/srv/ClearEntireCostmap', '{}'],
                capture_output=True,
                timeout=2
            )
            self.get_logger().info(f'   ✅ Costmaps nettoyées')
        except:
            pass

        time.sleep(0.5)
        
        # ═══════════════════════════════════════════════════════
        # VÉRIFICATION BATTERIE AVANT NAVIGATION NAV2
        # ═══════════════════════════════════════════════════════
        if self.low_battery_mode and zone_name != 'charging_zone':
            self.get_logger().warn('🚨 BATTERIE FAIBLE DÉTECTÉE AVANT NAV2 !')
            self.get_logger().warn('   → Annulation et navigation vers charging_zone')
            return self.go_to_zone('charging_zone')
        
        # Marquer que navigation commence
        self.navigation_in_progress = True
        # ⭐ NOUVEAU : Indiquer si c'est une mission de charge
        is_charging = (zone_name == 'charging_zone')

        success = self.nav_client.navigate_to_pose(
            x=x,
            y=y,
            z=zone_info["position"]["z"],
            orientation_w=zone_info["orientation"]["w"],
            is_charging_mission=is_charging  
        )
        # Marquer que navigation est terminée
        self.navigation_in_progress = False
        # 3. Retour du résultat
        if success:
                    self.get_logger().info(f'✅ Arrivé à {zone_name} !')
                    self.get_logger().info(f'📸 Robot positionné à ~0.2m du marqueur')
                    self.get_logger().info(f'💡 Prêt pour manipulation')
                    
                    # ═══════════════════════════════════════════════════════
                    # PUBLIER ZONE ACTUELLE (pour battery_manager)
                    # ═══════════════════════════════════════════════════════
                    self.publish_current_zone(zone_name)
                    
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