from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cyclic_pursuit',
            executable='pose_sub',
            output = 'screen'
        ),        
    ])