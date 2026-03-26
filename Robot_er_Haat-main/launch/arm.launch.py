import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm.urdf')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'arm.rviz')
    controllers_file = os.path.join(pkg_share, 'config', 'arm_controllers.yaml')

    # Process the URDF/xacro file
    robot_description_content = xacro.process_file(urdf_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Joy Node (reads from physical joystick)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )

    # Arm Joy Controller (our custom node)
    arm_joy_controller_node = Node(
        package='rover_arm',
        executable='arm_joy_controller',
        name='arm_joy_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
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
        use_sim_time,
        robot_state_publisher_node,
        joy_node,
        arm_joy_controller_node,
        rviz_node,
    ])
