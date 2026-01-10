#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

rclpy.init()
node = Node('test_mission_publisher')

pub = node.create_publisher(String, '/warehouse/execute_mission', 10)

# Attendre que le publisher soit prêt
import time
time.sleep(2)

# Mission d'arrivage
mission = {
    'type': 'arrival',
    'description': 'Test arrivage',
    'total_boxes': 5,
    'data': {
        'red_boxes': 2,
        'blue_boxes': 2,
        'yellow_boxes': 1
    }
}

msg = String()
msg.data = yaml.dump(mission)

print("📤 Envoi mission:")
print(msg.data)
print()

pub.publish(msg)

time.sleep(1)

print("✅ Mission envoyée !")

node.destroy_node()
rclpy.shutdown()
