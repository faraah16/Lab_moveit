#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from mission_orchestrator.mission_orchestrator_node import MissionOrchestrator
import time

def main(args=None):
    # ═══════════════════════════════════════════════════════
    # INITIALISER ROS2
    # ═══════════════════════════════════════════════════════
    rclpy.init(args=args)
    
    # ═══════════════════════════════════════════════════════
    # WARMUP TF
    # ═══════════════════════════════════════════════════════
    temp_node = Node('tf_warmup_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, temp_node)
    
    print("\n" + "="*60)
    print("⏳ Attente 30 secondes que AMCL et TF soient prêts...")
    print("="*60)
    
    for i in range(30, 0, -1):
        print(f"   Démarrage dans {i} secondes...", end='\r')
        rclpy.spin_once(temp_node, timeout_sec=0.1)
        time.sleep(0.9)
    
    print("\n✅ Attente terminée !")
    
    try:
        transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        print(f"✅ TF disponible! Position: x={transform.transform.translation.x:.2f}")
    except:
        print(f"⚠️  TF pas encore disponible")
    
    temp_node.destroy_node()
    
    # ═══════════════════════════════════════════════════════
    # CRÉER ORCHESTRATOR
    # ═══════════════════════════════════════════════════════
    orchestrator = MissionOrchestrator()
    
    print("⏳ Attente 15 secondes pour TF buffer...")
    for i in range(15, 0, -1):
        print(f"   {i} secondes...", end='\r')
        rclpy.spin_once(orchestrator, timeout_sec=0.1)
        time.sleep(0.9)
    print("\n✅ TF buffer prêt !\n")
    
    # ═══════════════════════════════════════════════════════
    # LISTE DES ZONES À TESTER
    # ═══════════════════════════════════════════════════════
    zones_to_test = [
        'green_table',        # Table verte
        'blue_table',         # Table bleue
        'red_table',          # Table rouge
        'depot_table',        # Table de dépôt
        'charging_zone',      # Zone de recharge
        'box_zone_1',         # Caisse 1
    ]
    
    print("\n" + "="*60)
    print("🧪 TEST DE NAVIGATION MULTI-ZONES")
    print(f"   {len(zones_to_test)} zones à tester")
    print("="*60 + "\n")
    
    # ═══════════════════════════════════════════════════════
    # TESTER CHAQUE ZONE
    # ═══════════════════════════════════════════════════════
    results = {}

    for i, zone_name in enumerate(zones_to_test):
        print(f"\n{'─'*60}")
        print(f"🎯 TEST {i+1}/{len(zones_to_test)}: {zone_name}")
        print(f"{'─'*60}\n")
        
        # ⭐ NOUVEAU : Afficher l'heure de début
        import datetime
        start_time = datetime.datetime.now()
        print(f"⏰ Début: {start_time.strftime('%H:%M:%S')}")
        
        # ⭐ NOUVEAU : Info avant navigation
        print(f"📍 Lancement de la navigation vers {zone_name}...")
        print(f"   (Consultez les logs ci-dessous pour détails rotation/navigation)\n")
        
        # Navigation
        success = orchestrator.go_to_zone(zone_name)
        
        # ⭐ NOUVEAU : Afficher l'heure de fin
        end_time = datetime.datetime.now()
        duration = (end_time - start_time).total_seconds()
        
        results[zone_name] = success
        
        if success:
            print(f"\n✅ {zone_name}: SUCCÈS")
            print(f"   ⏱️  Temps écoulé: {duration:.1f}s")
            print(f"   ⏰ Fin: {end_time.strftime('%H:%M:%S')}")
            # Pause 3 secondes à chaque point
            print("   ⏸️  Pause 3 secondes...")
            time.sleep(3)
        else:
            print(f"\n❌ {zone_name}: ÉCHEC")
            print(f"   ⏱️  Temps écoulé: {duration:.1f}s")
            print(f"   ⏰ Fin: {end_time.strftime('%H:%M:%S')}")
            
            # Demander si on continue
            print("\n⚠️  Voulez-vous continuer le test ?")
            print("   Appuyez sur Ctrl+C pour arrêter, ou attendez 5s pour continuer...")
            time.sleep(5)
    
    # ═══════════════════════════════════════════════════════
    # RÉSULTATS FINAUX
    # ═══════════════════════════════════════════════════════
    print("\n" + "="*60)
    print("📊 RÉSULTATS FINAUX")
    print("="*60)
    
    successes = sum(1 for v in results.values() if v)
    total = len(results)
    
    for zone_name, success in results.items():
        status = "✅ SUCCÈS" if success else "❌ ÉCHEC"
        print(f"   {zone_name:20s} : {status}")
    
    print(f"\n📈 Score: {successes}/{total} ({100*successes/total:.0f}%)")
    
    if successes == total:
        print("\n🎉🎉🎉 TOUS LES TESTS RÉUSSIS ! 🎉🎉🎉")
    elif successes > total / 2:
        print("\n👍 Majorité des tests réussis")
    else:
        print("\n⚠️  Plusieurs échecs détectés")
    
    print("="*60 + "\n")
    
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()