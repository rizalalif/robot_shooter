from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():
    package_name = 'robot_shooter'
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    
    patok_kiri =  os.path.join('/root/ros2_ws/src/robot_shooter/model', 'patok_anoman/model.sdf')
    patok_kanan =  os.path.join('/root/ros2_ws/src/robot_shooter/model', 'patok_sengkuni/model.sdf')
    patok_depan =  os.path.join('/root/ros2_ws/src/robot_shooter/model', 'patok_petruk/model.sdf')
    patok_belakang =  os.path.join('/root/ros2_ws/src/robot_shooter/model', 'patok_durna/model.sdf')

    
    # Argumen untuk masing-masing patok
    depan_x = LaunchConfiguration('depan_x')
    depan_y = LaunchConfiguration('depan_y')
    
    belakang_x = LaunchConfiguration('belakang_x')
    belakang_y = LaunchConfiguration('belakang_y')
    
    kiri_x = LaunchConfiguration('kiri_x')
    kiri_y = LaunchConfiguration('kiri_y')
    
    kanan_x = LaunchConfiguration('kanan_x')
    kanan_y = LaunchConfiguration('kanan_y')
    
    spawn_patok = [Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'patok_depan',
                '-file', patok_depan,
                '-x', depan_x,
                '-y', depan_y,
                '-z', '-0.1'
            ],
            output='screen'
        ),

        # Spawn patok belakang
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'patok_belakang',
                '-file',patok_belakang ,
                '-x', belakang_x,
                '-y', belakang_y,
                '-z', '-0.1'
            ],
            output='screen'
        ),

        # Spawn patok kiri
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'patok_kiri',
                '-file', patok_kiri,
                '-x', kiri_x,
                '-y', kiri_y,
                '-z', '-0.1'
            ],
            output='screen'
        ),

        # Spawn patok kanan
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'patok_kanan',
                '-file', patok_kanan,
                '-x', kanan_x,
                '-y', kanan_y,
                '-z', '-0.1'
            ],
            output='screen'
        ),]
    
    deleter_target = ExecuteProcess(
            cmd=['ros2', 'run', 'robot_shooter', 'target_deleter'],
            output='screen'
        )

    return LaunchDescription([
        deleter_target,
        
        
        # Deklarasi argumen
        DeclareLaunchArgument('depan_x', default_value='6.0'),
        DeclareLaunchArgument('depan_y', default_value='0.0'),
        
        DeclareLaunchArgument('belakang_x', default_value='-6.0'),
        DeclareLaunchArgument('belakang_y', default_value='0.0'),

        DeclareLaunchArgument('kiri_x', default_value='0.0'),
        DeclareLaunchArgument('kiri_y', default_value='6.0'),

        DeclareLaunchArgument('kanan_x', default_value='0.0'),
        DeclareLaunchArgument('kanan_y', default_value='-6.0'),

        # Spawn patok depan
        RegisterEventHandler(
            OnProcessExit(
                target_action=deleter_target,
                on_exit=spawn_patok
            )
        )
        
        
    ])
