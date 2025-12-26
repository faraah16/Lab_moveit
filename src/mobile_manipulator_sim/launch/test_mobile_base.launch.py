from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('mobile_manipulator_sim').find('mobile_manipulator_sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile_base.urdf.xacro')
    
    # Lire le fichier URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),
        
        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mobile_base', '-topic', 'robot_description']
        ),
        
        # Teleop keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='xterm -e',
            output='screen'
        )
    ])