from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node, LifecycleNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Config file paths
    ekf_config = PathJoinSubstitution([pkg_path, 'config', 'local_ekf.yaml'])
    global_ekf_config = PathJoinSubstitution([pkg_path, 'config', 'global_ekf.yaml'])
    nav2_params = PathJoinSubstitution([pkg_path, 'config', 'nav2_params.yaml'])
    
    # Map server node as LifecycleNode - FIXED: Added namespace parameter
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',  # ADDED THIS - empty namespace
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': PathJoinSubstitution(
                [pkg_path, 'config', 'empty_map.yaml']
            )
        }]
    )
    
    # Map server lifecycle manager
    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
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
                               '-x', '0.75', '-y', '1.15', '-z', '0.15','-R', '0', '-P', '0', '-Y', '3.14']
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
                        ('mecanum_drive_controller/odometry', 'odom')
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
        
        
        # 7. Camera bridge
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
        
        # 8. Camera localization node 
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
        
        # 9. cmd_vel topic remap node
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
        
        # 10. Map Server (starts at 20 seconds)
        TimerAction(
            period=20.0,
            actions=[
                map_server_node,
                map_server_lifecycle
            ]
        ),
        
        # 11. EKF Localization (starts at 22 seconds)
        TimerAction(
            period=22.0,
            actions=[
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
            ]
        ),
        
        # 12. Nav2 (starts at 24 seconds - after EKF and map server)
        TimerAction(
            period=24.0,
            actions=[
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
                        'params_file': nav2_params,
                        'map': PathJoinSubstitution([pkg_path, 'config', 'empty_map.yaml'])
                    }.items()
                )
            ]
        ),
    ])