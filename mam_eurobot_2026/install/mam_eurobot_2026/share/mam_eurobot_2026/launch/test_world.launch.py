# test_world.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    os.environ['XDG_RUNTIME_DIR'] = '/tmp/runtime-rosdev'
    
    return LaunchDescription([
        SetEnvironmentVariable(
            'XDG_RUNTIME_DIR',
            '/tmp/runtime-rosdev'
        ),
        
        # Test 1: Empty world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'
        ),
    ])