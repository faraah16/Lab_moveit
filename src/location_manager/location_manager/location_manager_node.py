#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class LocationManagerNode(Node):
    def __init__(self):
        super().__init__('location_manager')
        
        # Charger le fichier de configuration
        self.zones = {}
        self.zone_groups = {}
        self.load_zones_config()
        
        self.get_logger().info('Location Manager démarré avec succès ✅')
        self.get_logger().info(f'Zones chargées : {len(self.zones)}')
        
        # Publier la liste des zones au démarrage
        self.print_available_zones()
    
    def load_zones_config(self):
        """Charge le fichier YAML de configuration des zones"""
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
                self.get_logger().info(f'✅ {len(self.zones)} zones chargées depuis {config_path}')
            
            # Charger les groupes de zones
            if 'zone_groups' in config:
                self.zone_groups = config['zone_groups']
                self.get_logger().info(f'✅ {len(self.zone_groups)} groupes de zones chargés')
                
        except FileNotFoundError:
            self.get_logger().error(f'❌ Fichier de configuration non trouvé : {config_path}')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur lors du chargement de la config : {str(e)}')
    
    def get_zone_info(self, zone_name):
        """Récupère les informations d'une zone"""
        if zone_name not in self.zones:
            self.get_logger().warn(f'⚠️  Zone inconnue : {zone_name}')
            return None
        
        zone_data = self.zones[zone_name]
        
        # Créer un objet Pose
        pose = Pose()
        pose.position = Point(
            x=float(zone_data['position']['x']),
            y=float(zone_data['position']['y']),
            z=float(zone_data['position']['z'])
        )
        pose.orientation = Quaternion(
            x=float(zone_data['orientation']['x']),
            y=float(zone_data['orientation']['y']),
            z=float(zone_data['orientation']['z']),
            w=float(zone_data['orientation']['w'])
        )
        
        # Créer un dictionnaire avec toutes les infos
        zone_info = {
            'name': zone_data.get('name', zone_name),
            'pose': pose,
            'marker_id': zone_data.get('marker_id', -1),
            'function': zone_data.get('function', 'unknown'),
            'description': zone_data.get('description', ''),
            'color': zone_data.get('color', 'none')
        }
        
        return zone_info
    
    def get_zones_by_function(self, function):
        """Récupère toutes les zones ayant une fonction spécifique"""
        matching_zones = []
        for zone_name, zone_data in self.zones.items():
            if zone_data.get('function') == function:
                matching_zones.append(zone_name)
        return matching_zones
    
    def get_zones_by_color(self, color):
        """Récupère toutes les zones ayant une couleur spécifique"""
        matching_zones = []
        for zone_name, zone_data in self.zones.items():
            if zone_data.get('color') == color:
                matching_zones.append(zone_name)
        return matching_zones
    
    def get_zone_group(self, group_name):
        """Récupère un groupe de zones"""
        if group_name not in self.zone_groups:
            self.get_logger().warn(f'⚠️  Groupe inconnu : {group_name}')
            return []
        return self.zone_groups[group_name]
    
    def print_available_zones(self):
        """Affiche la liste des zones disponibles"""
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('📍 ZONES DISPONIBLES :')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        
        for zone_name, zone_data in self.zones.items():
            marker = f"[ID {zone_data.get('marker_id')}]" if zone_data.get('marker_id') is not None else "[No marker]"
            self.get_logger().info(f"  • {zone_name:25} {marker:12} - {zone_data.get('description', 'N/A')}")
        
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')


def main(args=None):
    rclpy.init(args=args)
    node = LocationManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
