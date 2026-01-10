#!/usr/bin/env python3
"""
╔════════════════════════════════════════════════════════════════════╗
║  🏭 WAREHOUSE SYSTEM LAUNCH FILE                                  ║
║                                                                    ║
║  Lance:                                                            ║
║  ✅ Navigation (Nav2 + AMCL + EKF)                                ║
║  ✅ Mission Orchestrator                                          ║
║  ✅ Stock Manager                                                 ║
║  ✅ Mission Queue Manager                                         ║
║                                                                    ║
║  ⚠️ PRÉREQUIS: Lancer d'abord warehouse_with_robot.launch.py    ║
╚════════════════════════════════════════════════════════════════════╝
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ════════════════════════════════════════════════════════════
    # INCLURE NAVIGATION LAUNCH
    # ════════════════════════════════════════════════════════════
    my_gazebo_dir = get_package_share_directory('my_gazebo_world')
    
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([my_gazebo_dir, '/launch/navigation.launch.py'])
    )
    
    # ════════════════════════════════════════════════════════════
    # WAREHOUSE SYSTEM NODES
    # ════════════════════════════════════════════════════════════
    
    # Mission Orchestrator
    mission_orchestrator_node = Node(
        package='mission_orchestrator',
        executable='mission_orchestrator',
        name='mission_orchestrator',
        output='screen',
        prefix='printf "\\033[1;96m[ORCHESTR]\\033[0m "'
    )
    
    # Stock Manager
    stock_manager_node = Node(
        package='warehouse_manager',
        executable='stock_manager_node',
        name='stock_manager',
        output='screen',
        prefix='printf "\\033[1;92m[STOCK]\\033[0m "'
    )
    
    # Mission Queue Manager
    mission_queue_node = Node(
        package='warehouse_manager',
        executable='mission_queue_manager',
        name='mission_queue_manager',
        output='screen',
        prefix='printf "\\033[1;95m[QUEUE]\\033[0m "'
    )
    
    # Message de démarrage
    startup_message = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['printf',
                     '\\n\\033[1;32m╔════════════════════════════════════════════════════════════╗\\033[0m\\n'
                     '\\033[1;32m║  ✅ WAREHOUSE SYSTEM PRÊT !                               ║\\033[0m\\n'
                     '\\033[1;32m╠════════════════════════════════════════════════════════════╣\\033[0m\\n'
                     '\\033[1;32m║  🧭 Navigation active                                     ║\\033[0m\\n'
                     '\\033[1;32m║  🎯 Mission Orchestrator prêt                             ║\\033[0m\\n'
                     '\\033[1;32m║  📦 Stock Manager prêt                                    ║\\033[0m\\n'
                     '\\033[1;32m║  ⚡ Mission Queue prêt                                    ║\\033[0m\\n'
                     '\\033[1;32m╠════════════════════════════════════════════════════════════╣\\033[0m\\n'
                     '\\033[1;33m║  ⚠️  LANCER MAINTENANT (Terminal 3):                     ║\\033[0m\\n'
                     '\\033[1;33m║    ros2 run warehouse_manager employee_interface          ║\\033[0m\\n'
                     '\\033[1;32m╚════════════════════════════════════════════════════════════╝\\033[0m\\n'
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Navigation
        navigation,
        
        # Warehouse System
        mission_orchestrator_node,
        stock_manager_node,
        mission_queue_node,
        
        # Message
        startup_message
    ])

