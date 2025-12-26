import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Chemin vers le package
    pkg_share = FindPackageShare('mobile_manipulator_sim').find('mobile_manipulator_sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_base.urdf')
    
    # Lire le fichier URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    return LaunchDescription([
        # ========== LANCER GAZEBO ==========
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
            launch_arguments={'verbose': 'false'}.items()
        ),
        
        # ========== ROBOT STATE PUBLISHER ==========
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),
        
        # ========== SPAWNER DU ROBOT DANS GAZEBO ==========
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_mobile_base',
            output='screen',
            arguments=[
                '-entity', 'mobile_base',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.2'
            ]
        )
    ])
