#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import yaml
import os
import sys
from datetime import datetime


class EmployeeInterface(Node):
    def __init__(self):
        super().__init__('employee_interface')
        
        # État du système
        self.battery_level = 100.0
        self.robot_zone = 'start_stop_zone'
        self.current_mission = 'Aucune'
        self.stock = {}
        self.mission_history = []
        self.order_counter = 1  # Compteur pour numéros de commande
        
        # Subscribers
        self.battery_sub = self.create_subscription(
            Float32,
            '/battery_status',
            self.battery_callback,
            10
        )
        
        self.zone_sub = self.create_subscription(
            String,
            '/current_zone',
            self.zone_callback,
            10
        )
        
        self.stock_sub = self.create_subscription(
            String,
            '/warehouse/stock_status',
            self.stock_callback,
            10
        )
        
        # Publisher pour commandes
        self.command_pub = self.create_publisher(
            String,
            '/warehouse/command',
            10
        )
        
        self.get_logger().info('👷 Employee Interface démarrée')
        
        # Attendre 2s que les topics se connectent
        import time
        time.sleep(2.0)
    
    def battery_callback(self, msg):
        """Mise à jour niveau batterie"""
        self.battery_level = msg.data
    
    def zone_callback(self, msg):
        """Mise à jour position robot"""
        self.robot_zone = msg.data
    
    def stock_callback(self, msg):
        """Mise à jour stock"""
        try:
            self.stock = yaml.safe_load(msg.data)
                # ⭐ DEBUG : Log pour vérifier réception
                # self.get_logger().info(f'📦 Stock mis à jour: {len(self.stock)} couleurs')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur parsing stock: {e}')
        
    def clear_screen(self):
        """Efface l'écran"""
        os.system('clear' if os.name == 'posix' else 'cls')
    
    def display_header(self):
        """Affiche l'en-tête"""
        print()
        print("╔═══════════════════════════════════════════════════════════╗")
        print("║          🏭 WAREHOUSE CONTROL SYSTEM 🏭                   ║")
        print("╚═══════════════════════════════════════════════════════════╝")
        print()
    
    def display_status(self):
        """Affiche l'état du système"""
        # Batterie avec barre
        battery_bars = int(self.battery_level / 10)
        battery_str = '█' * battery_bars + '░' * (10 - battery_bars)
        battery_color = '🟢' if self.battery_level > 50 else '🟡' if self.battery_level > 20 else '🔴'
        
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  ÉTAT DU SYSTÈME                                          │")
        print("├───────────────────────────────────────────────────────────┤")
        print(f"│  {battery_color} Batterie: {self.battery_level:5.1f}% [{battery_str}]           │")
        print(f"│  📍 Position: {self.robot_zone:30s}          │")
        print(f"│  🎯 Mission:  {self.current_mission:30s}          │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
    
    def display_stock(self):
        """Affiche le stock"""
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  📦 STOCK ACTUEL                                          │")
        print("├───────────────────────────────────────────────────────────┤")
        
        if not self.stock:
            print("│  ⚠️  Stock non disponible (en attente...)                │")
        else:
            for color, data in self.stock.items():
                total = data.get('total', 0)
                depot = data.get('dans_depot', 0)
                table = data.get('dans_table', 0)
                
                # Emoji selon la couleur
                emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
                color_name = color.replace('_boxes', '').replace('_', ' ').title()
                
                print(f"│  {emoji} {color_name:12s}: {total:2d} total  "
                      f"(Depot: {depot:2d} | Table: {table:2d})    │")
        
        print("└───────────────────────────────────────────────────────────┘")
        print()
    
    def display_menu(self):
        """Affiche le menu principal"""
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  COMMANDES DISPONIBLES                                    │")
        print("├───────────────────────────────────────────────────────────┤")
        print("│  [1] 📝 Modifier stock initial                            │")
        print("│  [2] 📦 Nouveau arrivage                                  │")
        print("│  [3] 🎯 Créer commande client                             │")
        print("│  [4] 🛑 Arrêter robot                                     │")
        print("│  [5] 📊 Afficher historique                               │")
        print("│  [6] 🔄 Rafraîchir affichage                              │")
        print("│  [0] ❌ Quitter                                            │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
    
   
    def display_history(self):
        """Affiche l'historique détaillé des missions"""
        self.clear_screen()
        self.display_header()
        
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  📊 HISTORIQUE DES MISSIONS                               │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
        
        if not self.mission_history:
            print("  ℹ️  Aucune mission dans l'historique")
            print()
        else:
            # Statistiques
            total_missions = len(self.mission_history)
            
            # Compter types de missions
            arrivals = sum(1 for m in self.mission_history if 'Arrivage' in m)
            orders = sum(1 for m in self.mission_history if 'CMD' in m)
            stops = sum(1 for m in self.mission_history if 'Arrêt' in m or 'ARRÊT' in m)
            stock_updates = sum(1 for m in self.mission_history if 'Stock modifié' in m)
            
            print("📈 STATISTIQUES:")
            print("─" * 61)
            print(f"  📦 Total missions: {total_missions}")
            print(f"  📥 Arrivages: {arrivals}")
            print(f"  🎯 Commandes clients: {orders}")
            print(f"  🛑 Arrêts: {stops}")
            print(f"  📝 Modifications stock: {stock_updates}")
            print()
            print("━" * 61)
            print()
            
            # Options d'affichage
            print("FILTRES:")
            print("  [1] Tout afficher")
            print("  [2] Arrivages uniquement")
            print("  [3] Commandes uniquement")
            print("  [4] Arrêts uniquement")
            print("  [0] Retour")
            print()
            
            filter_choice = input("Votre choix: ").strip()
            
            print()
            print("━" * 61)
            print()
            
            # Filtrer selon choix
            filtered = []
            
            if filter_choice == '1':
                filtered = self.mission_history
                print("📋 HISTORIQUE COMPLET:")
            elif filter_choice == '2':
                filtered = [m for m in self.mission_history if 'Arrivage' in m]
                print("📥 ARRIVAGES:")
            elif filter_choice == '3':
                filtered = [m for m in self.mission_history if 'CMD' in m]
                print("🎯 COMMANDES:")
            elif filter_choice == '4':
                filtered = [m for m in self.mission_history if 'Arrêt' in m or 'ARRÊT' in m]
                print("🛑 ARRÊTS:")
            elif filter_choice == '0':
                return
            else:
                print("❌ Choix invalide")
                input("\nAppuyez sur Entrée pour continuer...")
                return
            
            print("─" * 61)
            print()
            
            if not filtered:
                print("  ℹ️  Aucune mission de ce type")
            else:
                # Afficher les 20 dernières
                display_count = min(20, len(filtered))
                
                for i, mission in enumerate(filtered[-display_count:], 1):
                    # Ajouter emoji selon type
                    if 'CMD' in mission:
                        emoji = '🎯'
                    elif 'Arrivage' in mission:
                        emoji = '📦'
                    elif 'ARRÊT IMMÉDIAT' in mission:
                        emoji = '🚨'
                    elif 'Arrêt' in mission:
                        emoji = '🛑'
                    elif 'Stock' in mission:
                        emoji = '📝'
                    else:
                        emoji = '📌'
                    
                    print(f"  {i:2d}. {emoji} {mission}")
                
                if len(filtered) > 20:
                    print()
                    print(f"  💡 Affichage des 20 dernières sur {len(filtered)} missions")
            
            print()
            print("━" * 61)
        
        print()
        
        # Options supplémentaires
        print("OPTIONS:")
        print("  [1] Exporter l'historique")
        print("  [2] Effacer l'historique")
        print("  [0] Retour")
        print()
        
        option = input("Votre choix: ").strip()
        
        if option == '1':
            self.export_history()
        elif option == '2':
            self.clear_history()
        
        input("\nAppuyez sur Entrée pour continuer...")
    
    def export_history(self):
        """Exporte l'historique dans un fichier"""
        print()
        print("━" * 61)
        print("📤 EXPORT DE L'HISTORIQUE")
        print("━" * 61)
        print()
        
        if not self.mission_history:
            print("⚠️  Historique vide, rien à exporter")
            return
        
        # Générer nom de fichier
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"historique_warehouse_{timestamp}.txt"
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("═" * 61 + "\n")
                f.write("  HISTORIQUE DES MISSIONS - WAREHOUSE CONTROL SYSTEM\n")
                f.write("═" * 61 + "\n")
                f.write(f"  Date export: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"  Total missions: {len(self.mission_history)}\n")
                f.write("═" * 61 + "\n\n")
                
                for i, mission in enumerate(self.mission_history, 1):
                    f.write(f"{i:3d}. {mission}\n")
                
                f.write("\n" + "═" * 61 + "\n")
                f.write("  Fin de l'historique\n")
                f.write("═" * 61 + "\n")
            
            print(f"✅ Historique exporté: {filename}")
            print(f"📁 Fichier créé dans le répertoire courant")
        
        except Exception as e:
            print(f"❌ Erreur lors de l'export: {e}")
    
    def clear_history(self):
        """Efface l'historique"""
        print()
        print("━" * 61)
        print("🗑️  EFFACEMENT DE L'HISTORIQUE")
        print("━" * 61)
        print()
        
        if not self.mission_history:
            print("ℹ️  Historique déjà vide")
            return
        
        print(f"⚠️  Vous allez effacer {len(self.mission_history)} mission(s)")
        print("⚠️  Cette action est IRRÉVERSIBLE")
        print()
        
        confirm = input("Confirmer l'effacement ? (OUI en majuscules): ").strip()
        
        if confirm == 'OUI':
            self.mission_history.clear()
            print("\n✅ Historique effacé")
        else:
            print("\n❌ Effacement annulé")
    
    def modify_stock(self):
        """Permet de modifier le stock initial"""
        self.clear_screen()
        self.display_header()
        
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  📝 MODIFICATION DU STOCK INITIAL                         │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
        
        # Afficher stock actuel
        print("📦 Stock actuel:")
        for color, data in self.stock.items():
            total = data.get('total', 0)
            depot = data.get('dans_depot', 0)
            table = data.get('dans_table', 0)
            
            emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
            color_name = color.replace('_boxes', '').replace('_', ' ').title()
            
            print(f"  {emoji} {color_name:12s}: {total:2d} total "
                  f"(Depot: {depot:2d} | Table: {table:2d})")
        
        print()
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print()
        
        # Demander confirmation
        response = input("Voulez-vous modifier le stock ? (o/n): ").strip().lower()
        
        if response != 'o':
            print("\n❌ Modification annulée")
            return
        
        print()
        new_stock = {}
        
        # Pour chaque couleur
        for color in ['red_boxes', 'blue_boxes', 'yellow_boxes']:
            emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
            color_name = color.replace('_boxes', '').replace('_', ' ').title()
            
            print(f"\n{emoji} {color_name}:")
            print("─" * 40)
            
            while True:
                try:
                    # Demander les valeurs
                    depot = int(input(f"  Nombre dans depot_table: ").strip())
                    table = int(input(f"  Nombre dans {color_name.lower()}_table: ").strip())
                    
                    if depot < 0 or table < 0:
                        print("  ❌ Les nombres doivent être positifs !")
                        continue
                    
                    total = depot + table
                    
                    # Confirmer
                    print(f"\n  ✓ Total: {total} boxes ({depot} depot + {table} table)")
                    confirm = input("  Valider ? (o/n): ").strip().lower()
                    
                    if confirm == 'o':
                        new_stock[color] = {
                            'total': total,
                            'dans_depot': depot,
                            'dans_table': table
                        }
                        break
                    else:
                        print("  ⚠️  Recommencez pour cette couleur")
                
                except ValueError:
                    print("  ❌ Entrez un nombre valide !")
                except KeyboardInterrupt:
                    print("\n\n❌ Modification annulée")
                    return
        
        # Récapitulatif
        print()
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("📋 RÉCAPITULATIF DU NOUVEAU STOCK:")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        for color, data in new_stock.items():
            total = data['total']
            depot = data['dans_depot']
            table = data['dans_table']
            
            emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
            color_name = color.replace('_boxes', '').replace('_', ' ').title()
            
            print(f"  {emoji} {color_name:12s}: {total:2d} total "
                  f"(Depot: {depot:2d} | Table: {table:2d})")
        
        print()
        final_confirm = input("Confirmer et sauvegarder ? (o/n): ").strip().lower()
        
        if final_confirm == 'o':
            # Mettre à jour le stock local
            self.stock = new_stock
            
            # Publier la commande de mise à jour
            command = {
                'action': 'update_stock',
                'stock': new_stock,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            }
            
            msg = String()
            msg.data = yaml.dump(command)
            self.command_pub.publish(msg)
            
            # Ajouter à l'historique
            self.mission_history.append(
                f"[{datetime.now().strftime('%H:%M:%S')}] Stock modifié manuellement"
            )
            
            print("\n✅ Stock mis à jour avec succès !")
            print("💾 Sauvegarde en cours...")
            
            # Attendre confirmation (spin pour recevoir mise à jour)
            import time
            for i in range(3):
                rclpy.spin_once(self, timeout_sec=0.5)
                time.sleep(0.5)
            
            print("✅ Stock sauvegardé !")
        else:
            print("\n❌ Modification annulée")
        
        input("\nAppuyez sur Entrée pour continuer...")
    
    def new_arrival(self):
        """Enregistre un nouveau arrivage de boxes"""
        self.clear_screen()
        self.display_header()
        
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  📦 NOUVEAU ARRIVAGE                                      │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
        
        print("Indiquez les quantités arrivées au depot_table:")
        print("(Entrez 0 si aucune box de cette couleur)")
        print()
        
        arrival = {}
        total_boxes = 0
        
        # Pour chaque couleur
        for color in ['red_boxes', 'blue_boxes', 'yellow_boxes']:
            emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
            color_name = color.replace('_boxes', '').replace('_', ' ').title()
            
            while True:
                try:
                    qty = int(input(f"{emoji} {color_name:12s}: ").strip())
                    
                    if qty < 0:
                        print("   ❌ La quantité doit être positive !")
                        continue
                    
                    arrival[color] = qty
                    total_boxes += qty
                    break
                
                except ValueError:
                    print("   ❌ Entrez un nombre valide !")
                except KeyboardInterrupt:
                    print("\n\n❌ Arrivage annulé")
                    return
        
        # Vérifier si arrivage vide
        if total_boxes == 0:
            print("\n⚠️  Aucune box dans l'arrivage !")
            input("\nAppuyez sur Entrée pour continuer...")
            return
        
        # Récapitulatif
        print()
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("📋 RÉCAPITULATIF DE L'ARRIVAGE:")
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        for color, qty in arrival.items():
            if qty > 0:
                emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
                color_name = color.replace('_boxes', '').replace('_', ' ').title()
                print(f"  {emoji} {color_name:12s}: {qty} box(es)")
        
        print(f"\n  📊 Total: {total_boxes} box(es)")
        print()
        
        confirm = input("Confirmer l'arrivage et lancer le tri automatique ? (o/n): ").strip().lower()
        
        if confirm != 'o':
            print("\n❌ Arrivage annulé")
            input("\nAppuyez sur Entrée pour continuer...")
            return
        
        # Publier la commande d'arrivage
        command = {
            'action': 'new_arrival',
            'arrival': arrival,
            'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        }
        
        msg = String()
        msg.data = yaml.dump(command)
        self.command_pub.publish(msg)
        
        # Mettre à jour mission actuelle
        self.current_mission = f"Tri arrivage ({total_boxes} boxes)"
        
        # Ajouter à l'historique
        arrival_str = ', '.join([
            f"{qty} {color.replace('_boxes', '')}" 
            for color, qty in arrival.items() if qty > 0
        ])
        self.mission_history.append(
            f"[{datetime.now().strftime('%H:%M:%S')}] Arrivage: {arrival_str}"
        )
        
        print("\n✅ Arrivage enregistré !")
        print("🤖 Le robot va trier les boxes automatiquement...")
        print("📊 Stock depot mis à jour")
        
        # Attendre confirmation
        import time
        for i in range(3):
            rclpy.spin_once(self, timeout_sec=0.5)
            time.sleep(0.5)
        
        print("\n💡 Le robot commence le tri des boxes")
        print("   (Navigation + Pick & Place sera implémenté dans les prochaines étapes)")
        
        input("\nAppuyez sur Entrée pour continuer...")
    
    def create_order(self):
        """Crée une commande client"""
        self.clear_screen()
        self.display_header()
        
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  🎯 NOUVELLE COMMANDE CLIENT                              │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
        
        # ⭐ CRITICAL : Forcer refresh du stock
        print("⏳ Chargement du stock...")
        for i in range(5):
            rclpy.spin_once(self, timeout_sec=0.5)
        print()
        
        # Vérifier que le stock est chargé
        if not self.stock:
            print("❌ Erreur: Stock non disponible")
            print("💡 Assurez-vous que stock_manager_node est lancé")
            input("\nAppuyez sur Entrée pour continuer...")
            return
        
        # Afficher stock disponible (sur tables uniquement)
        print("📦 Stock disponible (sur tables):")
        available_stock = {}
        
        for color, data in self.stock.items():
            # ⭐ CORRECTION : Vérifier que data est un dict
            if isinstance(data, dict):
                table_qty = data.get('dans_table', 0)
                available_stock[color] = table_qty
                
                emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
                color_name = color.replace('_boxes', '').replace('_', ' ').title()
                
                status = '✅' if table_qty > 0 else '❌'
                print(f"  {status} {emoji} {color_name:12s}: {table_qty} disponible(s)")
            else:
                self.get_logger().warn(f'⚠️  Format incorrect pour {color}: {data}')
        
        print()
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print()
        # Numéro de commande
        order_number = self.order_counter
        print(f"📋 Commande #CMD{order_number:03d}")
        print()
        
        # Nom du client
        try:
            client_name = input("👤 Nom du client: ").strip()
            
            if not client_name:
                print("\n❌ Nom client requis !")
                input("\nAppuyez sur Entrée pour continuer...")
                return
            
            print()
            
            # Items de la commande
            print("📦 Items de la commande:")
            print("(Entrez 0 si vous ne voulez pas de cette couleur)")
            print()
            
            order_items = {}
            total_items = 0
            stock_available = True
            missing_items = []
            
            for color in ['red_boxes', 'blue_boxes', 'yellow_boxes']:
                emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
                color_name = color.replace('_boxes', '').replace('_', ' ').title()
                available = available_stock.get(color, 0)
                
                while True:
                    try:
                        qty = int(input(f"{emoji} {color_name:12s} (dispo: {available}): ").strip())
                        
                        if qty < 0:
                            print("   ❌ La quantité doit être positive !")
                            continue
                        
                        if qty > available:
                            print(f"   ⚠️  Stock insuffisant ! (disponible: {available})")
                            use_anyway = input("   Voulez-vous quand même commander ? (o/n): ").strip().lower()
                            
                            if use_anyway != 'o':
                                continue
                            else:
                                stock_available = False
                                missing_items.append(f"{qty - available} {color_name}")
                        
                        if qty > 0:
                            order_items[color] = qty
                            total_items += qty
                        
                        break
                    
                    except ValueError:
                        print("   ❌ Entrez un nombre valide !")
                    except KeyboardInterrupt:
                        print("\n\n❌ Commande annulée")
                        return
            
            # Vérifier si commande vide
            if total_items == 0:
                print("\n⚠️  Commande vide !")
                input("\nAppuyez sur Entrée pour continuer...")
                return
            
            print()
            
            # Destination
            print("📍 Destination:")
            print("  [1] 📦 Yellow Crate Right - TOP")
            print("  [2] 📦 Yellow Crate Right - MIDDLE")
            print()
            
            while True:
                dest_choice = input("Choisissez (1 ou 2): ").strip()
                
                if dest_choice == '1':
                    destination = 'yellow_crate_right_top'
                    break
                elif dest_choice == '2':
                    destination = 'yellow_crate_right_middle'
                    break
                else:
                    print("❌ Choix invalide !")
            
            print()
            
            # Priorité
            print("⚡ Priorité:")
            print("  [1] 🔴 Haute")
            print("  [2] 🟡 Normale")
            print("  [3] 🟢 Basse")
            print()
            
            while True:
                priority_choice = input("Choisissez (1, 2 ou 3): ").strip()
                
                if priority_choice == '1':
                    priority = 'haute'
                    break
                elif priority_choice == '2':
                    priority = 'normale'
                    break
                elif priority_choice == '3':
                    priority = 'basse'
                    break
                else:
                    print("❌ Choix invalide !")
            
            # Récapitulatif
            print()
            print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            print("📋 RÉCAPITULATIF DE LA COMMANDE:")
            print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
            print(f"  📋 Numéro: CMD{order_number:03d}")
            print(f"  👤 Client: {client_name}")
            print(f"  📦 Items:")
            
            for color, qty in order_items.items():
                emoji = '🔴' if 'red' in color else '🔵' if 'blue' in color else '🟡'
                color_name = color.replace('_boxes', '').replace('_', ' ').title()
                print(f"     - {emoji} {qty}x {color_name}")
            
            if destination == 'yellow_crate_right_top':
                dest_name = "Yellow Crate Right - TOP"
            elif destination == 'yellow_crate_right_middle':
                dest_name = "Yellow Crate Right - MIDDLE"
            else:
                dest_name = destination  # Fallback
            print(f"  📍 Destination: {dest_name}")
            
            priority_emoji = '🔴' if priority == 'haute' else '🟡' if priority == 'normale' else '🟢'
            print(f"  ⚡ Priorité: {priority_emoji} {priority.title()}")
            
            if not stock_available:
                print()
                print("  ⚠️  ATTENTION: Stock insuffisant pour certains items:")
                for item in missing_items:
                    print(f"     - Manque {item}")
                print("  💡 Le robot triera d'abord l'arrivage avant de préparer cette commande")
            
            print()
            
            confirm = input("Confirmer la commande ? (o/n): ").strip().lower()
            
            if confirm != 'o':
                print("\n❌ Commande annulée")
                input("\nAppuyez sur Entrée pour continuer...")
                return
            
            # Publier la commande
            command = {
                'action': 'new_order',
                'order': {
                    'number': f"CMD{order_number:03d}",
                    'client': client_name,
                    'items': order_items,
                    'destination': destination,
                    'priority': priority,
                    'stock_available': stock_available,
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
            }
            
            msg = String()
            msg.data = yaml.dump(command)
            self.command_pub.publish(msg)
            
            # Mettre à jour mission actuelle
            self.current_mission = f"Commande #{order_number} ({client_name})"
            
            # Ajouter à l'historique
            items_str = ', '.join([
                f"{qty}x {color.replace('_boxes', '')}" 
                for color, qty in order_items.items()
            ])
            
            self.mission_history.append(
                f"[{datetime.now().strftime('%H:%M:%S')}] CMD{order_number:03d} - {client_name}: {items_str}"
            )
            
            # Incrémenter compteur
            self.order_counter += 1
            
            print("\n✅ Commande enregistrée !")
            
            if stock_available:
                print("🤖 Le robot va préparer la commande...")
            else:
                print("🤖 Le robot va d'abord trier l'arrivage, puis préparer la commande")
            
            # Attendre confirmation
            import time
            for i in range(3):
                rclpy.spin_once(self, timeout_sec=0.5)
                time.sleep(0.5)
            
            print("\n💡 Commande ajoutée à la file des missions")
            print("   (Navigation + Pick & Place sera implémenté dans les prochaines étapes)")
            
        except KeyboardInterrupt:
            print("\n\n❌ Commande annulée")
        
        input("\nAppuyez sur Entrée pour continuer...")
    
    def stop_robot(self):
        """Arrête le robot"""
        self.clear_screen()
        self.display_header()
        
        print("┌───────────────────────────────────────────────────────────┐")
        print("│  🛑 ARRÊT DU ROBOT                                        │")
        print("└───────────────────────────────────────────────────────────┘")
        print()
        
        # Afficher état actuel
        print("📊 État actuel:")
        print(f"  📍 Position: {self.robot_zone}")
        print(f"  🎯 Mission: {self.current_mission}")
        print(f"  🔋 Batterie: {self.battery_level:.1f}%")
        print()
        
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print()
        
        # Options d'arrêt
        print("🛑 MODES D'ARRÊT:")
        print()
        print("  [1] 🟡 ARRÊT SOFT")
        print("      → Le robot termine sa mission en cours")
        print("      → Puis va à la zone start/stop")
        print("      → Temps estimé: Variable (selon mission)")
        print()
        print("  [2] 🔴 ARRÊT IMMÉDIAT")
        print("      → Annule TOUT immédiatement")
        print("      → Va directement à start/stop")
        print("      → ⚠️  Mission en cours sera perdue !")
        print()
        print("  [0] ❌ Annuler (continuer les opérations)")
        print()
        
        try:
            choice = input("Votre choix (0, 1 ou 2): ").strip()
            
            if choice == '0':
                print("\n✅ Annulé - Robot continue")
                input("\nAppuyez sur Entrée pour continuer...")
                return
            
            elif choice == '1':
                # ARRÊT SOFT
                print()
                print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
                print("⚠️  CONFIRMATION ARRÊT SOFT")
                print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
                print()
                print("Le robot va:")
                print("  ✓ Terminer sa mission en cours")
                print("  ✓ Se diriger vers start/stop zone")
                print("  ✓ Les missions en attente seront conservées")
                print()
                
                confirm = input("Confirmer l'arrêt soft ? (o/n): ").strip().lower()
                
                if confirm != 'o':
                    print("\n❌ Arrêt annulé")
                    input("\nAppuyez sur Entrée pour continuer...")
                    return
                
                # Publier commande arrêt soft
                command = {
                    'action': 'stop_robot',
                    'mode': 'soft',
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
                
                msg = String()
                msg.data = yaml.dump(command)
                self.command_pub.publish(msg)
                
                # Mettre à jour mission
                self.current_mission = "Arrêt en cours (soft)"
                
                # Historique
                self.mission_history.append(
                    f"[{datetime.now().strftime('%H:%M:%S')}] Arrêt SOFT demandé"
                )
                
                print("\n✅ Commande d'arrêt soft envoyée")
                print("🤖 Le robot va terminer sa mission puis s'arrêter")
                
            elif choice == '2':
                # ARRÊT IMMÉDIAT
                print()
                print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
                print("🚨 CONFIRMATION ARRÊT IMMÉDIAT")
                print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
                print()
                print("⚠️  ATTENTION: Le robot va:")
                print("  ✗ ANNULER la mission en cours")
                print("  ✗ VIDER la file des missions")
                print("  ✓ Se diriger IMMÉDIATEMENT vers start/stop")
                print()
                print("🔴 Cette action est IRRÉVERSIBLE !")
                print()
                
                confirm = input("Confirmer l'arrêt IMMÉDIAT ? (OUI en majuscules): ").strip()
                
                if confirm != 'OUI':
                    print("\n❌ Arrêt annulé")
                    input("\nAppuyez sur Entrée pour continuer...")
                    return
                
                # Double confirmation
                print()
                print("⚠️  DERNIÈRE CONFIRMATION")
                final = input("Êtes-vous ABSOLUMENT sûr ? (o/n): ").strip().lower()
                
                if final != 'o':
                    print("\n❌ Arrêt annulé")
                    input("\nAppuyez sur Entrée pour continuer...")
                    return
                
                # Publier commande arrêt immédiat
                command = {
                    'action': 'stop_robot',
                    'mode': 'immediate',
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                }
                
                msg = String()
                msg.data = yaml.dump(command)
                self.command_pub.publish(msg)
                
                # Mettre à jour mission
                self.current_mission = "ARRÊT IMMÉDIAT"
                
                # Historique
                self.mission_history.append(
                    f"[{datetime.now().strftime('%H:%M:%S')}] ARRÊT IMMÉDIAT demandé"
                )
                
                print("\n🚨 Commande d'arrêt IMMÉDIAT envoyée")
                print("🤖 Le robot va s'arrêter MAINTENANT")
            
            else:
                print("\n❌ Choix invalide !")
                input("\nAppuyez sur Entrée pour continuer...")
                return
            
            # Attendre confirmation
            import time
            for i in range(3):
                rclpy.spin_once(self, timeout_sec=0.5)
                time.sleep(0.5)
            
            print("\n💡 Commande transmise au robot")
            
        except KeyboardInterrupt:
            print("\n\n❌ Arrêt annulé")
        
        input("\nAppuyez sur Entrée pour continuer...")

    def run_interface(self):
        """Boucle principale de l'interface"""
        while rclpy.ok():
            # Rafraîchir données (spin une fois)
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Afficher l'interface
            self.clear_screen()
            self.display_header()
            self.display_status()
            self.display_stock()
            self.display_menu()
            
            # Demander choix
            try:
                choice = input("Votre choix: ").strip()
                
                if choice == '0':
                    print("\n👋 Au revoir !\n")
                    break
                
                elif choice == '1':
                    self.modify_stock()
                    
                elif choice == '2':
                    self.new_arrival()

                elif choice == '3':
                    self.create_order()
                    
                elif choice == '4':
                    self.stop_robot()

                elif choice == '5':
                    self.clear_screen()
                    self.display_header()
                    self.display_history()
                    input("\nAppuyez sur Entrée pour continuer...")
                
                elif choice == '6':
                    # Juste rafraîchir (boucle continue)
                    pass
                
                else:
                    print("\n❌ Choix invalide !")
                    input("\nAppuyez sur Entrée pour continuer...")
            
            except KeyboardInterrupt:
                print("\n\n👋 Au revoir !\n")
                break
            except Exception as e:
                print(f"\n❌ Erreur: {e}")
                input("\nAppuyez sur Entrée pour continuer...")


def main(args=None):
    rclpy.init(args=args)
    interface = EmployeeInterface()
    
    try:
        interface.run_interface()
    except KeyboardInterrupt:
        pass
    finally:
        interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
