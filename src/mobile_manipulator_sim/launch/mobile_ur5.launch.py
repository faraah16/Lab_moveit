import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Chemins
    pkg_share = FindPackageShare('mobile_manipulator_sim').find('mobile_manipulator_sim')
    xacro_file = os.path.join(pkg_share, 'urdf', 'mobile_ur5.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'ur5_controllers.yaml')
    gazebo_ros_share = FindPackageShare('gazebo_ros').find('gazebo_ros')
    
    # Traiter le xacro
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file,
        ' ', 'use_fake_hardware:=true',  # IMPORTANT: false pour Gazebo
        ' ', 'sim_gazebo:=true'
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # ========== 1. LANCER GAZEBO ==========
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'pause': 'false'}.items()
    )
    
    # ========== 2. ROBOT STATE PUBLISHER ==========
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # ========== 3. SPAWN ROBOT ==========
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'mobile_ur5',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1',
            '-timeout', '60.0'
        ],
        output='screen'
    )
    
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])
    
    # ========== 4. CHARGER LES CONTRÔLEURS ==========
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Attendre que le robot soit spawné avant de charger les contrôleurs
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster]
        )
    )
    
    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller]
        )
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        delayed_joint_state_broadcaster,
        delayed_arm_controller
    ])