import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'arm.rviz')
    robot_description_content = xacro.process_file(urdf_file).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joy',
            executable='joy_node',
        ),
        Node(
            package='rover_arm',
            executable='ik_visualizer',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file]
        )
    ])
