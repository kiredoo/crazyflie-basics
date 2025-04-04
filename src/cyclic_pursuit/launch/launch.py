from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('cyclic_pursuit'),
        'config',
        'cyclic.yaml'
    )

    node =  Node(
            package='cyclic_pursuit',
            executable='pose_sub',
            output = 'screen',
            parameters = [config]
        )       

    ld.add_action(node)
    return ld 
  