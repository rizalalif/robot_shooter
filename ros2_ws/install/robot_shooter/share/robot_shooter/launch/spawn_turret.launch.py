from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('robot_shooter'),  # ganti dengan nama package kamu
        'urdf',
        'shooter_robot.urdf.xacro'
    )

    # urdf_path = os.path.join(pkg_share, 'urdf', 'shooter_robot.urdf.xacro')
    robot_description_content = xacro.process_file(urdf_file).toxml()



    return LaunchDescription([
        DeclareLaunchArgument("gui",default_value="True",description="This is a flag for joint state publishers"),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'

        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('robot_shooter'), 'rviz', 'turret_config.rviz'
            )],
            output='screen'
        )
    ])
