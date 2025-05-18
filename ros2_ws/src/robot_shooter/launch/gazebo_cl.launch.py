#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    package_name = 'robot_shooter'
    
    # Paths
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'shooter_robot.urdf.xacro')
    robot_description_content = xacro.process_file(urdf_path).toxml()
    
    # Controllers
    controller_config = os.path.join(pkg_share, 'config', 'shooter_controllers.yaml')
    # robot_controllers = os.path.join(pkg_robot, 'config', 'turret.urdf.xacro')
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        remappings=[('robot_description', '/robot_description')],

    )
    
    # Gazebo nodes
    world_file = os.path.join(pkg_share, 'worlds', 'shooter_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros').find('gazebo_ros'), '/launch/gazebo.launch.py']),
        launch_arguments={'world': world_file}.items(),
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot_shooter'],
        output='screen'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
    )

    
    # Load controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    load_pan_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_controller'],
    )
    
    load_tilt_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tilt_controller'],
    )

    joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'

    )
    
    rviz_controller = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_shooter'), 'rviz', 'turret_config.rviz'
            )],
            output='screen'
    )
    
    # Target tracking node
    # target_tracking_node = Node(
    #     package=package_name,
    #     executable='target_tracker',
    #     name='target_tracker',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }]
    # )
    
    # # Target mover node to create motion
    # target_mover_node = Node(
    #     package=package_name,
    #     executable='target_mover',
    #     name='target_mover',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }]
    # )
    
    # # Projectile launcher node
    # projectile_launcher_node = Node(
    #     package=package_name,
    #     executable='projectile_launcher',
    #     name='projectile_launcher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time
    #     }]
    # )
    
    # # Shoot controller node
    # shoot_controller_node = Node(
    #     package=package_name,
    #     executable='shoot_controller',
    #     name='shoot_controller',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'auto_fire': False,      # Set to True for automatic test firing
    #         'fire_rate': 0.5         # Fire rate in Hz when auto_fire is True
    #     }]
    # )
    
    # Define launch sequence
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Nodes to start in parallel
        robot_state_publisher_node,
        gazebo,
        spawn_entity,
        joint_state_publisher_gui,
        rviz_controller,
        control_node,


        
        # Controller nodes - start after spawning
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster]
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_pan_controller]
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_pan_controller,
                on_exit=[load_tilt_controller]
            )
        ),
        
        # Start nodes after controllers are loaded
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_tilt_controller,
        #         on_exit=[
        #             target_tracking_node, 
        #             target_mover_node,
        #             projectile_launcher_node,
        #             shoot_controller_node
        #         ]
        #     )
        # ),

        
        
    ])