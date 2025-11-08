from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_image_bridge',
            output='screen',
            arguments=['/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image']
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_info_bridge',
            output='screen',
            arguments=['/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']
        ),
    ])
