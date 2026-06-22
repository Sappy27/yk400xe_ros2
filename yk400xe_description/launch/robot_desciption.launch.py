import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = os.path.join(
        get_package_share_directory('yk400xe_description'))

    xacro_file = os.path.join(pkg_path,
                              'urdf',
                              'yk400xe.urdf.xacro')
    
    urdf_path = os.path.join(pkg_path, 'urdf', 'yk400xe.urdf')

    doc = xacro.process_file(xacro_file)

    robot_desc = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()

    params = {'robot_description': robot_desc}
        
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    rviz_node = Node(
        package="rviz2",
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_path, "rviz/rviz_config.rviz")],
    )

    return LaunchDescription([
        node_robot_state_publisher,
        rviz_node
    ])
