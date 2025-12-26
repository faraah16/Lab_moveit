#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Chemins
    config_dir = get_package_share_directory('mobile_manipulator_sim')
    slam_params_file = os.path.join(config_dir, 'config', 'mapper_params_online_async.yaml')
    
    rviz_dir = get_package_share_directory('my_gazebo_world')
    rviz_config_file = os.path.join(rviz_dir, 'rviz', 'mapping_config.rviz')
    
    return LaunchDescription([
        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),
        
        # RViz avec configuration automatique
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])