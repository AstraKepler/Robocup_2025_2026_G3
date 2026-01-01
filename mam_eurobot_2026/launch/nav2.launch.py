from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('mam_eurobot_2026')
    
    ekf_config = PathJoinSubstitution([
        pkg,
        'config',
        'local_ekf.yaml'
    ])

    global_ekf_config = PathJoinSubstitution([
        pkg,
        'config',
        'global_ekf.yaml'
    ])

    nav2_params = PathJoinSubstitution([
        pkg,
        'config',
        'nav2_params.yaml'
    ])

    return LaunchDescription([
        # EKF local robot localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': True}]
        ),
        # EKF Global
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_node',
            output='screen',
            parameters=[global_ekf_config, {'use_sim_time': True}]
        ),
        
        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': nav2_params
            }.items()
        ),
    ])