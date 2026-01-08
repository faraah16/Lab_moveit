#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import yaml
from datetime import datetime


class MissionQueueManager(Node):
    def __init__(self):
        super().__init__('mission_queue_manager')
        
        # File de missions avec priorités
        self.mission_queue = []
        
        # État du système
        self.battery_level = 100.0
        self.low_battery_mode = False
        self.current_mission = None
        self.robot_busy = False
        
        # Subscribers
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
        
        self.command_sub = self.create_subscription(
            String,
            '/warehouse/command',
            self.command_callback,
            10
        )
        
        # Publishers
        self.mission_pub = self.create_publisher(
            String,
            '/warehouse/execute_mission',
            10
        )
        
        # Timer pour traiter la file
        self.timer = self.create_timer(2.0, self.process_queue)
        
        self.get_logger().info('⚡ Mission Queue Manager démarré')
        self.get_logger().info('📋 Priorités: Batterie > Commandes > Tri arrivage')
    
    def battery_callback(self, msg):
        """Mise à jour niveau batterie"""
        self.battery_level = msg.data
    
    def battery_alert_callback(self, msg):
        """Gestion alertes batterie"""
        if msg.data == 'LOW_BATTERY':
            self.low_battery_mode = True
            self.get_logger().warn('⚠️  BATTERIE FAIBLE - Priorité CHARGE')
        elif msg.data == 'CHARGED':
            self.low_battery_mode = False
            self.get_logger().info('✅ BATTERIE CHARGÉE - Reprise missions')
    
    def command_callback(self, msg):
        """Reçoit les commandes et les ajoute à la file"""
        try:
            command = yaml.safe_load(msg.data)
            action = command.get('action', '')
            
            if action == 'new_arrival':
                self.add_arrival_mission(command)
            
            elif action == 'new_order':
                self.add_order_mission(command)
            
            elif action == 'stop_robot':
                self.handle_stop_robot(command)
        
        except Exception as e:
            self.get_logger().error(f'❌ Erreur traitement commande: {e}')
    
    def add_arrival_mission(self, command):
        """Ajoute une mission de tri d'arrivage"""
        arrival = command.get('arrival', {})
        
        # Calculer total boxes
        total_boxes = sum(arrival.values())
        
        mission = {
            'type': 'arrival',
            'priority': 3,  # Priorité basse
            'data': arrival,
            'total_boxes': total_boxes,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'description': f"Tri arrivage ({total_boxes} boxes)"
        }
        
        self.mission_queue.append(mission)
        self.sort_queue()
        
        self.get_logger().info(f'📦 Mission ajoutée: Tri arrivage ({total_boxes} boxes)')
        self.display_queue()
    
    def add_order_mission(self, command):
        """Ajoute une mission de commande client"""
        order = command.get('order', {})
        
        # Déterminer priorité numérique
        priority_str = order.get('priority', 'normale')
        if priority_str == 'haute':
            priority = 1
        elif priority_str == 'normale':
            priority = 2
        else:  # basse
            priority = 3
        
        # Vérifier stock disponible
        stock_available = order.get('stock_available', True)
        
        mission = {
            'type': 'order',
            'priority': priority,
            'data': order,
            'stock_available': stock_available,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'description': f"Commande {order.get('number', 'N/A')} - {order.get('client', 'N/A')}"
        }
        
        # Si stock insuffisant, ajouter aussi une mission de tri AVANT
        if not stock_available:
            self.get_logger().warn('⚠️  Stock insuffisant pour cette commande')
            self.get_logger().info('💡 Ajout d\'une mission de tri prioritaire')
            
            # Créer mission de tri avec priorité légèrement supérieure
            arrival_mission = {
                'type': 'arrival_for_order',
                'priority': priority,  # Même priorité que la commande
                'data': {},  # Sera complété par l'arrivage
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'description': f"Tri urgent pour {order.get('number', 'N/A')}"
            }
            
            self.mission_queue.append(arrival_mission)
        
        self.mission_queue.append(mission)
        self.sort_queue()
        
        priority_emoji = '🔴' if priority == 1 else '🟡' if priority == 2 else '🟢'
        self.get_logger().info(f'{priority_emoji} Mission ajoutée: {mission["description"]}')
        self.display_queue()
    
    def sort_queue(self):
        """Trie la file par priorité (1 = haute, 3 = basse)"""
        self.mission_queue.sort(key=lambda m: (m['priority'], m['timestamp']))
    
    def display_queue(self):
        """Affiche la file des missions"""
        if not self.mission_queue:
            self.get_logger().info('📋 File de missions: VIDE')
            return
        
        self.get_logger().info('')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('📋 FILE DES MISSIONS')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        
        for i, mission in enumerate(self.mission_queue, 1):
            priority = mission['priority']
            priority_emoji = '🔴' if priority == 1 else '🟡' if priority == 2 else '🟢'
            
            self.get_logger().info(
                f'  {i}. {priority_emoji} {mission["description"]}'
            )
        
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('')
    
    def handle_stop_robot(self, command):
        """Gère l'arrêt du robot"""
        mode = command.get('mode', 'soft')
        
        if mode == 'soft':
            self.get_logger().warn('')
            self.get_logger().warn('🟡 ARRÊT SOFT DEMANDÉ')
            self.get_logger().warn('   → Robot va terminer mission en cours')
            self.get_logger().warn('   → Puis aller à start/stop zone')
            self.get_logger().warn('')
            
            # Créer mission d'arrêt avec haute priorité (mais après mission en cours)
            stop_mission = {
                'type': 'stop',
                'priority': 0,  # Priorité maximale (après mission en cours)
                'data': {'zone': 'start_stop_zone'},
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'description': "Retour à start/stop zone (arrêt soft)"
            }
            
            # Ajouter en début de file (sera exécuté après mission actuelle)
            self.mission_queue.insert(0, stop_mission)
            
            self.get_logger().info('✅ Mission d\'arrêt ajoutée')
            self.display_queue()
        
        elif mode == 'immediate':
            self.get_logger().error('')
            self.get_logger().error('🔴 ARRÊT IMMÉDIAT DEMANDÉ')
            self.get_logger().error('   → ANNULATION de toutes les missions')
            self.get_logger().error('   → Navigation IMMÉDIATE vers start/stop')
            self.get_logger().error('')
            
            # Vider la file
            cancelled_count = len(self.mission_queue)
            self.mission_queue.clear()
            
            if cancelled_count > 0:
                self.get_logger().warn(f'   ⚠️  {cancelled_count} mission(s) annulée(s)')
            
            # Créer mission d'arrêt immédiat
            stop_mission = {
                'type': 'emergency_stop',
                'priority': -1,  # Priorité absolue
                'data': {'zone': 'start_stop_zone'},
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'description': "ARRÊT D'URGENCE - Retour start/stop"
            }
            
            # Mission immédiate
            self.mission_queue = [stop_mission]
            
            # Forcer le robot à ne plus être busy
            self.robot_busy = False
            
            self.get_logger().error('🚨 ARRÊT D\'URGENCE ACTIVÉ')
            self.display_queue()
            
    def process_queue(self):
        """Traite la file de missions"""
        # Si batterie faible, priorité absolue à la charge
        if self.low_battery_mode:
            if not self.robot_busy:
                self.get_logger().warn('🔋 BATTERIE FAIBLE - Robot va charger')
                self.robot_busy = True  # Le robot gère ça automatiquement
            return
        
        # Si robot occupé, attendre
        if self.robot_busy:
            return
        
        # Si file vide, rien à faire
        if not self.mission_queue:
            return
        
        # Prendre la mission la plus prioritaire
        mission = self.mission_queue.pop(0)
        self.current_mission = mission
        
        self.get_logger().info('')
        self.get_logger().info('🚀 EXÉCUTION PROCHAINE MISSION')
        self.get_logger().info(f'   {mission["description"]}')
        self.get_logger().info('')
        
        # Publier pour exécution
        msg = String()
        msg.data = yaml.dump(mission)
        self.mission_pub.publish(msg)
        
        self.robot_busy = True
        
        # Simuler fin de mission (à remplacer par feedback réel plus tard)
        # Pour l'instant, on suppose que la mission prend 10s
        import time
        time.sleep(10)
        self.robot_busy = False
        
        self.get_logger().info('✅ Mission terminée (simulé)')
        
        # Afficher file restante
        if self.mission_queue:
            self.display_queue()


def main(args=None):
    rclpy.init(args=args)
    manager = MissionQueueManager()
    
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
