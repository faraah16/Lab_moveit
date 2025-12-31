#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from location_manager.location_manager_node import LocationManagerNode


class LocationManagerTest(Node):
    def __init__(self):
        super().__init__('location_manager_test')
        
        # Créer une instance du Location Manager
        self.location_manager = LocationManagerNode()
        
        # Tester différentes requêtes
        self.run_tests()
    
    def run_tests(self):
        """Exécute une série de tests"""
        self.get_logger().info('🧪 DÉBUT DES TESTS')
        self.get_logger().info('=' * 60)
        
        # Test 1: Récupérer info d'une zone
        self.test_get_zone('blue_table')
        self.test_get_zone('charging_zone')
        self.test_get_zone('depot_table')
        
        # Test 2: Zone inexistante
        self.test_get_zone('zone_inexistante')
        
        # Test 3: Zones par fonction
        self.test_zones_by_function('pick_colored_box')
        self.test_zones_by_function('battery_charging')
        
        # Test 4: Zones par couleur
        self.test_zones_by_color('red')
        self.test_zones_by_color('blue')
        
        # Test 5: Groupe de zones
        self.test_zone_group('colored_tables')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎉 TESTS TERMINÉS')
    
    def test_get_zone(self, zone_name):
        """Test de récupération d'une zone"""
        self.get_logger().info(f'\n📍 Test: get_zone("{zone_name}")')
        zone_info = self.location_manager.get_zone_info(zone_name)
        
        if zone_info:
            self.get_logger().info(f'  ✅ Zone trouvée:')
            self.get_logger().info(f'     Nom: {zone_info["name"]}')
            self.get_logger().info(f'     Position: ({zone_info["pose"].position.x:.2f}, {zone_info["pose"].position.y:.2f})')
            self.get_logger().info(f'     Marker ID: {zone_info["marker_id"]}')
            self.get_logger().info(f'     Fonction: {zone_info["function"]}')
        else:
            self.get_logger().info(f'  ❌ Zone non trouvée')
    
    def test_zones_by_function(self, function):
        """Test de récupération par fonction"""
        self.get_logger().info(f'\n🔍 Test: zones avec fonction "{function}"')
        zones = self.location_manager.get_zones_by_function(function)
        self.get_logger().info(f'  ✅ Trouvé {len(zones)} zone(s): {zones}')
    
    def test_zones_by_color(self, color):
        """Test de récupération par couleur"""
        self.get_logger().info(f'\n🎨 Test: zones de couleur "{color}"')
        zones = self.location_manager.get_zones_by_color(color)
        self.get_logger().info(f'  ✅ Trouvé {len(zones)} zone(s): {zones}')
    
    def test_zone_group(self, group_name):
        """Test de récupération d'un groupe"""
        self.get_logger().info(f'\n📦 Test: groupe "{group_name}"')
        zones = self.location_manager.get_zone_group(group_name)
        self.get_logger().info(f'  ✅ Zones du groupe: {zones}')


def main(args=None):
    rclpy.init(args=args)
    test_node = LocationManagerTest()
    
    try:
        rclpy.spin_once(test_node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
