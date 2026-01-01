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
        # 3. Remap IMU topic
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='imu_bridge',
                    output='screen',
                    arguments=[
                        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
                    ],
                )
            ]
        ),
        
        # 4. Start robot state publisher
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
        
        # 5. Spawn robot
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    output='screen',
                    arguments=['-topic', 'robot_description',
                               '-name', 'mecanum_vehicle',
                               '-x', '0.5', '-y', '0.75', '-z', '0.15','-R', '0', '-P', '0', '-Y', '0'] #'-x', '0.75', '-y', '1.15', '-z', '0.15','-R', '0', '-P', '0', '-Y', '3.14'
                )
            ]
        ),
        
        # 6. Start controllers
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
                    remappings=[
                        ('mecanum_drive_controller/odometry', 'odom')     # Remabing the topics for NAV2
                    ],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['gripper_controller'],
                    output='screen'
                )
            ]
        ),
        # 7. Start gripper controllers
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='mam_eurobot_2026',
                    executable='gripper_control_node.py',
                    name='gripper_control',
                    output='screen',
                    parameters=[
                        {'close_distance': 0.01}
                    ]
                )
            ]
        ),
        # 8. Camera bridge
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='camera_image_bridge',
                    output='screen',
                    arguments=[
                        '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                        '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
                    ],
                )
            ]
        ),
        # 9. Camera localization node 
        TimerAction(
            period=16.0,
            actions=[
                Node(
                    package='mam_eurobot_2026',
                    executable='view_camera.py',
                    name='camera_viewer',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),
        # 10. cmd_vel topic remap node
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='mam_eurobot_2026',
                    executable='cmd_vel_bridge.py',
                    name='cmd_vel_bridge',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        ),
    ])