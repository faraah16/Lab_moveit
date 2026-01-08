#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class BatteryManager(Node):
    def __init__(self):
        super().__init__('battery_manager')
        
        # État batterie
        self.battery_level = 100.0
        self.discharge_rate = 0.5  # % par seconde
        self.low_battery_threshold = 20.0
        self.charged_threshold = 95.0
        self.is_charging = False
        self.charge_rate = 2.0  # % par seconde
        
        # ⭐ NOUVEAU : Mode override pour tests
        self.override_mode = False
        self.override_level = 100.0
        self.override_timer = None  # ← CORRECTION
        
        # Publishers
        self.battery_pub = self.create_publisher(
            Float32,
            '/battery_status',
            10
        )
        
        self.alert_pub = self.create_publisher(
            String,
            '/battery_alert',
            10
        )
        
        # Subscriber zone actuelle
        self.zone_sub = self.create_subscription(
            String,
            '/current_zone',
            self.zone_callback,
            10
        )
        
        # ⭐ NOUVEAU : Subscriber pour forcer niveau batterie
        self.override_sub = self.create_subscription(
            Float32,
            '/battery_override',
            self.override_callback,
            10
        )
        
        # Timer mise à jour batterie
        self.timer = self.create_timer(1.0, self.update_battery)
        
        self.get_logger().info('🔋 Battery Manager started')
        self.get_logger().info('💡 Pour forcer batterie: ros2 topic pub /battery_override std_msgs/msg/Float32 "data: 15.0" --once')
    
    def override_callback(self, msg):
        """Forcer le niveau de batterie (pour tests)"""
        self.override_mode = True
        self.override_level = msg.data
        self.battery_level = msg.data
        self.get_logger().warn(f'🔧 OVERRIDE MODE ACTIVÉ: Batterie forcée à {msg.data:.1f}%')
        
        # ⭐ CORRECTION : Annuler ancien timer si existant
        if self.override_timer is not None:
            self.override_timer.cancel()
        
        # ⭐ CORRECTION : Créer timer avec callback lambda
        self.override_timer = self.create_timer(30.0, self._disable_override_timer)
        self.get_logger().info('⏰ Override actif pendant 30 secondes')
    
    def _disable_override_timer(self):
        """Timer callback pour désactiver override"""
        self.disable_override()
        if self.override_timer is not None:
            self.override_timer.cancel()
            self.override_timer = None
    
    def disable_override(self):
        """Désactiver mode override"""
        if self.override_mode:
            self.override_mode = False
            self.get_logger().info('🔧 Override mode désactivé - retour mode normal')
    
    def zone_callback(self, msg):
        """Détection zone de charge"""
        if msg.data == 'charging_zone':
            if not self.is_charging:
                self.get_logger().info('🔌 Entrée dans charging_zone - CHARGE EN COURS')
                self.is_charging = True
        else:
            if self.is_charging:
                self.get_logger().info('🚶 Sortie de charging_zone - DÉCHARGE EN COURS')
                self.is_charging = False
    
    def update_battery(self):
        """Mise à jour niveau batterie"""
        
        # ⭐ Si mode override, garder niveau fixe
        if self.override_mode:
            self.battery_level = self.override_level
        else:
            # Logique normale
            if self.is_charging:
                self.battery_level += self.charge_rate
                if self.battery_level > 100.0:
                    self.battery_level = 100.0
            else:
                self.battery_level -= self.discharge_rate
                if self.battery_level < 0.0:
                    self.battery_level = 0.0
        
        # Publier niveau
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)
        
        # Publier alertes EN CONTINU pendant override
        if self.battery_level < self.low_battery_threshold:
            alert = String()
            alert.data = 'LOW_BATTERY'
            self.alert_pub.publish(alert)
            
            # Log seulement de temps en temps pour pas spam
            if int(self.battery_level * 10) % 10 == 0:  # Toutes les 10 itérations
                self.get_logger().warn(f'⚠️ LOW BATTERY: {self.battery_level:.1f}%')
        
        elif self.battery_level >= self.charged_threshold and self.is_charging:
            alert = String()
            alert.data = 'CHARGED'
            self.alert_pub.publish(alert)
            self.get_logger().info(f'✅ CHARGED: {self.battery_level:.1f}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()