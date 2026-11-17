"""Nav2 (map server + AMCL + planner + controller + behaviours) with RViz and room labels.

    ros2 launch tb3_point_to_point navigation.launch.py                        # project map, 3D follow view
    ros2 launch tb3_point_to_point navigation.launch.py rviz_view:=overview    # top-down view of the floor
    ros2 launch tb3_point_to_point navigation.launch.py map:=$HOME/maps/my_map.yaml

The Nav2 parameter file is chosen for the running ROS distro
(config/nav2_params_humble.yaml or config/nav2_params_jazzy.yaml) unless params_file is given.
AMCL starts already localised at the Reception spawn pose (set_initial_pose in the params).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb3_point_to_point'


def default_params():
    pkg = get_package_share_directory(PKG)
    distro = os.environ.get('ROS_DISTRO', 'humble')
    name = 'nav2_params_humble.yaml' if distro in ('humble', 'iron') else 'nav2_params_jazzy.yaml'
    return os.path.join(pkg, 'config', name)


def _setup(context):
    pkg = get_package_share_directory(PKG)
    params = LaunchConfiguration('params_file').perform(context) or default_params()
    map_yaml = LaunchConfiguration('map').perform(context)
    view = LaunchConfiguration('rviz_view').perform(context)
    rviz_cfg = os.path.join(pkg, 'rviz', 'overview.rviz' if view == 'overview' else 'navigation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    actions = [
        LogInfo(msg=f'[{PKG}] map: {map_yaml}'),
        LogInfo(msg=f'[{PKG}] Nav2 params: {params}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'map': map_yaml, 'use_sim_time': use_sim_time, 'params_file': params,
                              'autostart': 'true', 'slam': 'False'}.items()),
        Node(package=PKG, executable='room_labels', name='room_labels', output='log',
             parameters=[{'use_sim_time': True}]),
    ]
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes'):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'indoor_office.yaml'),
                              description='map YAML to navigate in'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Nav2 parameters (default: chosen for the ROS distro)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_view', default_value='follow', description='follow | overview'),
        OpaqueFunction(function=_setup),
    ])
