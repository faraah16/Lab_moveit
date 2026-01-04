#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from mission_orchestrator.mission_orchestrator_node import MissionOrchestrator
import time

def main(args=None):
    rclpy.init(args=args)
    
    temp_node = Node('tf_warmup_node')
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, temp_node)
    
    print("\n" + "="*60)
    print("⏳ Attente 30 secondes que AMCL et TF soient prêts...")
    print("   (TF buffer se remplit en arrière-plan...)")
    print("="*60)
    
    for i in range(30, 0, -1):
        print(f"   Démarrage dans {i} secondes...", end='\r')
        rclpy.spin_once(temp_node, timeout_sec=0.1)
        time.sleep(0.9)
    
    print("\n✅ Attente terminée !")
    
    try:
        transform = tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        print(f"✅ TF map→base_link disponible!")
        print(f"   Position robot: x={transform.transform.translation.x:.2f}, y={transform.transform.translation.y:.2f}")
    except Exception as e:
        print(f"⚠️  TF map→base_link pas encore disponible : {str(e)[:60]}")
    
    print("\n🚀 Démarrage du test !\n")
    
    temp_node.destroy_node()
    
    # ═══════════════════════════════════════════════════════
    # CRÉER LE MISSION ORCHESTRATOR
    # ═══════════════════════════════════════════════════════
    orchestrator = MissionOrchestrator()
    
    # ═══════════════════════════════════════════════════════
    # ATTENDRE QUE SON TF BUFFER SE REMPLISSE !
    # ═══════════════════════════════════════════════════════
    print("⏳ Attente 15 secondes que le TF buffer de MissionOrchestrator se remplisse...")
    for i in range(15, 0, -1):
        print(f"   Démarrage dans {i} secondes...", end='\r')
        rclpy.spin_once(orchestrator, timeout_sec=0.1)  # Spin pour recevoir TF
        time.sleep(0.9)
    print("\n✅ TF buffer de MissionOrchestrator prêt !\n")
    
    print("\n" + "="*60)
    print("🧪 TEST VERS BLUE_TABLE")
    print("="*60)
    print("\n📍 Navigation vers blue_table")
    
    success = orchestrator.go_to_zone('yellow_crate_right_top')
    
    if success:
        print("\n" + "="*60)
        print("✅ TEST RÉUSSI !")
        print("="*60)
    else:
        print("\n" + "="*60)
        print("❌ TEST ÉCHOUÉ !")
        print("="*60)
    
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()