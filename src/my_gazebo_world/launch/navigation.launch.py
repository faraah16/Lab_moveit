#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Chemins
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    mobile_sim_dir = get_package_share_directory('mobile_manipulator_sim')
    my_gazebo_dir = get_package_share_directory('my_gazebo_world')
    
    # Fichiers de config
    map_file = os.path.join(my_gazebo_dir, 'maps', 'warehouse_map_last_version.yaml')
    nav2_params_file = os.path.join(mobile_sim_dir, 'config', 'nav2_params.yaml')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Map Server (EXPLICITE)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )
    
    # Lifecycle Manager pour map_server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # AMCL pour localisation
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )
    
    # Lifecycle Manager pour AMCL
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl']
        }]
    )
    
    # Nav2 Bringup (controller, planner, bt_navigator, behaviors)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )
    
    # Initialisation automatique AMCL après 8 secondes
    amcl_initial_pose = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '/initialpose',
            'geometry_msgs/PoseWithCovarianceStamped',
            '{header: {frame_id: "map"}, '
            'pose: {pose: {position: {x: -5.0, y: 3.1, z: 0.0}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, '
            'covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.25, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.0, '
            '0.0, 0.0, 0.0, 0.0, 0.0, 0.02]}}'
        ],
        output='screen'
    )
    
    delayed_amcl_init = TimerAction(
        period=8.0,
        actions=[amcl_initial_pose]
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_autostart_cmd,
        
        # 1. Lancer map_server en PREMIER
        map_server_node,
        lifecycle_manager_map,
        
        # 2. Lancer AMCL
        amcl_node,
        lifecycle_manager_amcl,
        
        # 3. Lancer Nav2
        nav2_bringup,
        
        # 4. Initialiser AMCL
        delayed_amcl_init,
    ])