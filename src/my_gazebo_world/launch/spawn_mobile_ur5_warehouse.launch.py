#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    my_gazebo_world_dir = get_package_share_directory('my_gazebo_world')
    mobile_manipulator_sim_dir = get_package_share_directory('mobile_manipulator_sim')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(my_gazebo_world_dir, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(mobile_manipulator_sim_dir, 'urdf', 'mobile_ur5.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    gazebo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')), launch_arguments=[('world', world_file), ('verbose', 'true')])
    robot_state_publisher_node = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen', parameters=[robot_description])
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-entity', 'mobile_ur5', '-topic', 'robot_description',  '-x', '-3.5', '-y', '-2.2', '-z', '-1.75'], output='screen')
    load_joint_state_broadcaster = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    load_arm_controller = Node(package='controller_manager', executable='spawner', arguments=['arm_controller'], output='screen')
    return LaunchDescription([gazebo_launch, robot_state_publisher_node, spawn_entity, load_joint_state_broadcaster, load_arm_controller])
