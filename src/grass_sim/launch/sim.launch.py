"""
Full GRASS stack against a simulated Arduino, visualised in RViz.

    grass_web (controller + phone UI) -> grass_serial -> pty -> sim_arduino (real firmware) -> RViz
                     \\-> /camera/capture -> grass_camera (mock) -> /camera/status

The controller, serial bridge and camera node are the unmodified GRASS packages; only the Arduino
and the arm are simulated. Simulation state (plants copy, logs, web login) lives in ~/.grass_sim
so it never touches the real robot's ~/.grass or the repository's data/plants.yaml.

    ros2 launch grass_sim sim.launch.py
    ros2 launch grass_sim sim.launch.py time_scale:=3.0 web_port:=8081
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SIM_HOME = os.path.expanduser('~/.grass_sim')
SIM_USER, SIM_PASSWORD = 'sim', 'grass'


def _default_repo():
    # <ws>/install/grass_sim/share/grass_sim -> <ws>/project ref/GRASS-main
    share = get_package_share_directory('grass_sim')
    ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    return os.path.join(ws, 'project ref', 'GRASS-main')


def _setup(context):
    repo = LaunchConfiguration('grass_repo').perform(context)
    plants = LaunchConfiguration('plants_file').perform(context)
    link = LaunchConfiguration('link_path').perform(context)

    os.makedirs(SIM_HOME, exist_ok=True)
    if not os.path.exists(plants):
        shutil.copyfile(os.path.join(repo, 'data', 'plants.yaml'), plants)
    auth = os.path.join(SIM_HOME, 'web_auth.json')
    if not os.path.exists(auth):
        from grass_controller.web_auth import set_password
        set_password(SIM_USER, SIM_PASSWORD, auth)

    desc_share = get_package_share_directory('grass_description')
    with open(os.path.join(desc_share, 'urdf', 'grass.urdf')) as f:
        robot_desc = f.read()
    rviz_config = os.path.join(get_package_share_directory('grass_sim'), 'config', 'grass_sim.rviz')

    return [
        SetEnvironmentVariable('GRASS_LOG_DIR', os.path.join(SIM_HOME, 'logs')),
        SetEnvironmentVariable('GRASS_WEB_AUTH_FILE', auth),
        Node(package='grass_sim', executable='sim_arduino', output='screen', parameters=[{
            'grass_repo': repo,
            'link_path': link,
            'time_scale': float(LaunchConfiguration('time_scale').perform(context)),
        }]),
        Node(package='grass_serial', executable='serial_node', output='screen', parameters=[{
            'serial_port': link, 'boot_delay': 0.0, 'reconnect_interval': 1.0}]),
        Node(package='grass_camera', executable='camera_node', output='screen', parameters=[{
            'camera': 'mock',
            'mock_delay_s': float(LaunchConfiguration('camera_delay_s').perform(context))}]),
        Node(package='grass_controller', executable='grass_web', output='screen', arguments=[
            '--host', LaunchConfiguration('web_host').perform(context),
            '--port', LaunchConfiguration('web_port').perform(context),
            '--plants-file', plants]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_desc}]),
        Node(package='grass_sim', executable='scene', parameters=[{'plants_file': plants}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('grass_repo', default_value=_default_repo(),
                              description='GRASS repository (for firmware/host_test)'),
        DeclareLaunchArgument('plants_file', default_value=os.path.join(SIM_HOME, 'plants.yaml'),
                              description='plants/routines (seeded from the repo on first run)'),
        DeclareLaunchArgument('link_path', default_value='/tmp/grass_sim_arduino'),
        DeclareLaunchArgument('time_scale', default_value='1.0',
                              description='>1 runs the simulated firmware faster than real time'),
        DeclareLaunchArgument('camera_delay_s', default_value='1.5'),
        DeclareLaunchArgument('web_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
