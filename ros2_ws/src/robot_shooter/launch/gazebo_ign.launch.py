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
    
    
    # Gazebo nodes
    world_file = os.path.join(pkg_share, 'worlds', 'shooter_world.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments=[('gz_args',' -r -v 3 empty.sdf')],
    )

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output="screen"
    )

    # gz_camera_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=['/shooter/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
    #     output='screen'
    # )
    
    # Spawn robot
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-entity', 'robot_shooter'],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_shooter'), 'rviz', 'turret_config.rviz'
            )],
            output='screen'
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='both',
    )


    
    # Load controllers
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    pan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['pan_controller', '--param-file', controller_config],
    )

    tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tilt_controller', '--param-file', controller_config,],
    )
    
    # joint_state_publisher_gui = Node(
    #         package='joint_state_publisher_gui',
    #         executable='joint_state_publisher_gui',
    #         name='joint_state_publisher_gui',
    #         output='screen'

    # )

    
    # Define launch sequence
    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        # Nodes to start in parallel
        gazebo,
        gazebo_bridge,
        # gz_camera_bridge,
        gz_spawn_entity,
        robot_state_publisher_node,
        rviz_node,
        # joint_state_publisher_gui,
        control_node,
        joint_state_broadcaster,
        pan_controller_spawner,
        tilt_controller_spawner,

        
        # Controller nodes - start after spawning
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[joint_state_broadcaster]
        #     )
        # ),
        
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster,
        #         on_exit=[pan_controller_spawner]
        #     )
        # ),
        
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=pan_controller_spawner,
        #         on_exit=[tilt_controller_spawner]
        #     )
        # ),
        
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