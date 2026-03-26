import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_arm')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm.urdf')

    # Process xacro
    robot_description_content = xacro.process_file(urdf_file).toxml()
    robot_description = {'robot_description': robot_description_content}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Gazebo Sim (Harmonic)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty.sdf --render-engine ogre'}.items()
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rover_arm',
            '-topic', 'robot_description',
            '-z', '0.5',
        ],
    )

    # Controller Manager — load and activate controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen',
    )
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
    ])
