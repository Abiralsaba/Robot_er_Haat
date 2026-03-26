from setuptools import setup
import os
from glob import glob

package_name = 'rover_arm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # RViz configs
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # Controller configs
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='ROS 2 package for controlling the rover arm with a joystick.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_joy_controller = rover_arm.arm_joy_controller:main',
            'ik_visualizer = rover_arm.ik_visualizer:main',
        ],
    },
)
