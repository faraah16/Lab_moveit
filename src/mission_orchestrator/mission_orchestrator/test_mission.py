#!/usr/bin/env python3

import rclpy
from mission_orchestrator.mission_orchestrator_node import MissionOrchestrator


def main(args=None):
    rclpy.init(args=args)
    
    orchestrator = MissionOrchestrator()
    
    print("\n" + "="*60)
    print("🧪 TEST VERS BLUE_TABLE")
    print("="*60 + "\n")
    
    # Test vers blue_table
    print("📍 Navigation vers blue_table")
    success = orchestrator.go_to_zone('blue_table')
    
    if success:
        print("\n✅ TEST RÉUSSI !")
    else:
        print("\n❌ TEST ÉCHOUÉ !")
    
    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()