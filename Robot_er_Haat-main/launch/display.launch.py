import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'arm.rviz')
    
    # Process xacro
    robot_description_content = xacro.process_file(urdf_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher GUI (shows sliders for joints)
    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
