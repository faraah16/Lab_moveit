#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import ExecuteProcess
import sys
    

def generate_launch_description():
    # Packages
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    my_gazebo_world_dir = get_package_share_directory('my_gazebo_world')
    mobile_manipulator_sim_dir = get_package_share_directory('mobile_manipulator_sim')
    
    # Fichiers
    world_file = os.path.join(my_gazebo_world_dir, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(mobile_manipulator_sim_dir, 'urdf', 'simple_mobile_robot.urdf')
    
    # Robot description
    with open(xacro_file, 'r') as f:
        robot_description_config_text = f.read()
    robot_description = {'robot_description': robot_description_config_text}
    # Charger les paramètres ros2_control
    controller_config = os.path.join(mobile_manipulator_sim_dir, 'config', 'simple_arm_controllers.yaml')
    # LANCER GAZEBO
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, controller_config]  # <-- Ajoutez controller_config
    )
        
    # Spawn robot - POSITION STABLE
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mobile_simple_arm', 
                   '-topic', 'robot_description',
                   '-x', '-3.5', 
                   '-y', '-2.2', 
                   '-z', '0.2'],  # Sur le sol
        output='screen'
    )
    
    # Controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen'
    )
    
    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen'
    )
    # ========================================
    # NŒUD DE DÉTECTION ARUCO PERSONNALISÉ
    # ========================================
    
 
    
    aruco_detector = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(os.path.expanduser('~'), 'Lab_moveit', 'aruco_detector_node.py')
        ],
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        aruco_detector
    ])