#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory


class StockManager(Node):
    def __init__(self):
        super().__init__('stock_manager')
        
        # Chemin vers le fichier stock
        self.stock_file = os.path.join(
            get_package_share_directory('warehouse_manager'),
            'config',
            'warehouse_stock.yaml'
        )
        
        # Charger le stock initial
        self.stock = {}
        self.load_stock()
        
        # Publisher pour broadcast stock
        self.stock_pub = self.create_publisher(
            String,
            '/warehouse/stock_status',
            10
        )
        
        # Timer pour publier stock toutes les 5s
        self.timer = self.create_timer(5.0, self.publish_stock)
        # Subscriber pour commandes de l'interface
        self.command_sub = self.create_subscription(
            String,
            '/warehouse/command',
            self.command_callback,
            10
        )
        self.get_logger().info('📦 Stock Manager démarré')
        self.get_logger().info(f'📁 Fichier stock: {self.stock_file}')
        self.display_stock()
    
    def load_stock(self):
        """Charge le stock depuis le fichier YAML"""
        try:
            with open(self.stock_file, 'r') as f:
                data = yaml.safe_load(f)
                self.stock = data.get('stock', {})
                self.capacites = data.get('capacites', {})
                self.get_logger().info('✅ Stock chargé depuis YAML')
        except FileNotFoundError:
            self.get_logger().error(f'❌ Fichier stock non trouvé: {self.stock_file}')
            self.stock = {}
        except Exception as e:
            self.get_logger().error(f'❌ Erreur lecture stock: {e}')
            self.stock = {}
    
    def save_stock(self):
        """Sauvegarde le stock dans le fichier YAML"""
        try:
            data = {
                'stock': self.stock,
                'capacites': self.capacites,
                'last_update': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            
            with open(self.stock_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
            
            self.get_logger().info('💾 Stock sauvegardé dans YAML')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur sauvegarde stock: {e}')
    
    def display_stock(self):
        """Affiche le stock dans les logs"""
        self.get_logger().info('')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('📦 ÉTAT DU STOCK')
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        
        for color, data in self.stock.items():
            total = data.get('total', 0)
            depot = data.get('dans_depot', 0)
            table = data.get('dans_table', 0)
            
            self.get_logger().info(
                f'  🔴 {color:15s}: {total:2d} total '
                f'(Depot: {depot:2d} | Table: {table:2d})'
            )
        
        self.get_logger().info('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
        self.get_logger().info('')
    
    def publish_stock(self):
        """Publie le stock sur le topic"""
        msg = String()
        msg.data = yaml.dump(self.stock)
        self.stock_pub.publish(msg)
    
    def command_callback(self, msg):
        """Traite les commandes de l'interface employé"""
        try:
            command = yaml.safe_load(msg.data)
            action = command.get('action', '')
            
            if action == 'update_stock':
                self.get_logger().info('📝 Réception commande: Mise à jour stock')
                
                # Mettre à jour le stock
                new_stock = command.get('stock', {})
                self.stock = new_stock
                
                # Sauvegarder dans YAML
                self.save_stock()
                
                # Afficher nouveau stock
                self.display_stock()
                
                self.get_logger().info('✅ Stock mis à jour et sauvegardé')
            
            elif action == 'new_arrival':
                self.get_logger().info('📦 Réception commande: Nouveau arrivage')
                
                arrival = command.get('arrival', {})
                
                # Mettre à jour le stock (ajouter dans depot)
                for color, qty in arrival.items():
                    if qty > 0:
                        if color in self.stock:
                            self.stock[color]['dans_depot'] += qty
                            self.stock[color]['total'] += qty
                        else:
                            # Si couleur pas dans stock, l'ajouter
                            self.stock[color] = {
                                'total': qty,
                                'dans_depot': qty,
                                'dans_table': 0
                            }
                        
                        color_name = color.replace('_boxes', '').title()
                        self.get_logger().info(f'   + {qty} {color_name}')
                
                # Sauvegarder
                self.save_stock()
                self.display_stock()
                
                self.get_logger().info('✅ Arrivage enregistré et stock mis à jour')
                self.get_logger().info('🤖 Mission de tri ajoutée à la file')
            elif action == 'new_order':
                self.get_logger().info('🎯 Réception commande: Nouvelle commande client')
                
                order = command.get('order', {})
                order_number = order.get('number', 'N/A')
                client = order.get('client', 'N/A')
                items = order.get('items', {})
                priority = order.get('priority', 'normale')
                stock_available = order.get('stock_available', True)
                
                self.get_logger().info(f'   📋 {order_number} - Client: {client}')
                self.get_logger().info(f'   ⚡ Priorité: {priority}')
                
                # Afficher items
                for color, qty in items.items():
                    color_name = color.replace('_boxes', '').title()
                    self.get_logger().info(f'      - {qty}x {color_name}')
                
                if stock_available:
                    # Décrémenter le stock (retirer des tables)
                    self.get_logger().info('   📦 Mise à jour stock...')
                    
                    for color, qty in items.items():
                        if color in self.stock:
                            # Retirer des tables
                            self.stock[color]['dans_table'] -= qty
                            self.stock[color]['total'] -= qty
                            
                            color_name = color.replace('_boxes', '').title()
                            self.get_logger().info(f'      ✓ Retiré {qty} {color_name} de la table')
                    
                    # Sauvegarder
                    self.save_stock()
                    self.display_stock()
                    
                    self.get_logger().info('✅ Commande enregistrée, stock mis à jour')
                    self.get_logger().info('🤖 Mission de préparation ajoutée à la file')
                
                else:
                    self.get_logger().warn('⚠️  Stock insuffisant pour cette commande')
                    self.get_logger().info('💡 La commande sera traitée après le tri de l\'arrivage')
                    self.get_logger().info('🤖 Mission en attente')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur traitement commande: {e}')


    def get_stock_summary(self):
        """Retourne un résumé du stock"""
        summary = {}
        for color, data in self.stock.items():
            summary[color] = {
                'total': data.get('total', 0),
                'depot': data.get('dans_depot', 0),
                'table': data.get('dans_table', 0)
            }
        return summary


def main(args=None):
    rclpy.init(args=args)
    stock_manager = StockManager()
    
    try:
        rclpy.spin(stock_manager)
    except KeyboardInterrupt:
        pass
    finally:
        stock_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
