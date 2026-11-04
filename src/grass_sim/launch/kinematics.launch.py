"""
Kinematics validation: stick model on sliders + FK/IK overlays from grass_controller.kinematics.

    ros2 launch grass_sim kinematics.launch.py
    ros2 launch grass_sim kinematics.launch.py robot_config:=/path/to/robot.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from grass_controller.robot_config import default_robot_config_path, load_robot_config
from grass_sim.stick_urdf import stick_urdf


def _setup(context):
    config = LaunchConfiguration('robot_config').perform(context) or str(default_robot_config_path())
    rviz = os.path.join(get_package_share_directory('grass_sim'), 'config', 'kinematics.rviz')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': stick_urdf(load_robot_config(config))}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             on_exit=Shutdown()),
        Node(package='grass_sim', executable='kin_check', output='screen',
             parameters=[{'robot_config': config}], on_exit=Shutdown()),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz], on_exit=Shutdown()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_config', default_value='',
                              description="robot.yaml (default: grass_controller's)"),
        OpaqueFunction(function=_setup),
    ])
