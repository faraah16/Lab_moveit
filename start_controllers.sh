#!/bin/bash
sleep 5

# Charger les paramètres du controller manager
ros2 param load /controller_manager ~/Lab_moveit/controller_params.yaml

sleep 2

# Démarrer les controllers
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller arm_controller  
ros2 control load_controller gripper_controller

sleep 1

ros2 control set_controller_state joint_state_broadcaster start
ros2 control set_controller_state arm_controller start
ros2 control set_controller_state gripper_controller start

echo "✅ Controllers démarrés !"
