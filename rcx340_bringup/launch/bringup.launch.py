from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rcx340_bringup')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    controller_node = Node(
        package='rcx340_bringup',
        executable='controller_communication',
        name='controller_communication_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([controller_node])