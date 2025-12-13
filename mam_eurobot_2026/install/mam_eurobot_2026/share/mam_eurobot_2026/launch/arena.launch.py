# arena_final.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-rosdev'
    
    pkg_path = FindPackageShare('mam_eurobot_2026')
    
    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('mam_eurobot_2026'),
                 'urdf', 'four_wheel_robot.xacro']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    
    return LaunchDescription([
        # Environment
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', pkg_path),
        SetEnvironmentVariable('XDG_RUNTIME_DIR', '/tmp/runtime-rosdev'),
        
        # Arguments
        DeclareLaunchArgument(
            'world',
            default_value=PathJoinSubstitution([pkg_path, 'worlds', 'arena_world.sdf']),
            description='World file'),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time'),
        
        # 1. Launch Gazebo
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
            output='screen'
        ),
        
        # 2. Start bridge after 3 seconds
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
                    output='screen'
                )
            ]
        ),
        
        # 3. Start robot state publisher after 5 seconds
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                                robot_description]
                )
            ]
        ),
        
        # 4. Spawn robot after 7 seconds
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=['-topic', 'robot_description',
                               '-name', 'mecanum_vehicle',
                               '-x', '0', '-y', '0', '-z', '0.15']
                )
            ]
        ),
        
        # 5. Start controllers after 10 seconds
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['mecanum_drive_controller'],
                    output='screen'
                )
            ]
        ),
    ])