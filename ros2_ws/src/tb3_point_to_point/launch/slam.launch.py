"""Build a map with SLAM Toolbox (run the simulation first).

    ros2 launch tb3_point_to_point slam.launch.py                 # drive with teleop yourself
    ros2 launch tb3_point_to_point slam.launch.py explore:=true   # + Nav2 so explore_and_map can drive

Save the map afterwards with:
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map --ros-args -p use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb3_point_to_point'


def _setup(context):
    pkg = get_package_share_directory(PKG)
    use_sim_time = LaunchConfiguration('use_sim_time')
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time,
                              'slam_params_file': os.path.join(pkg, 'config', 'slam_toolbox.yaml')}.items()),
        Node(package=PKG, executable='room_labels', name='room_labels', output='log',
             parameters=[{'use_sim_time': True}]),
    ]
    if LaunchConfiguration('explore').perform(context).lower() in ('true', '1', 'yes'):
        distro = os.environ.get('ROS_DISTRO', 'humble')
        params = os.path.join(pkg, 'config', 'nav2_params_humble.yaml' if distro in ('humble', 'iron')
                              else 'nav2_params_jazzy.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': params,
                              'autostart': 'true'}.items()))
    if LaunchConfiguration('rviz').perform(context).lower() in ('true', '1', 'yes'):
        view = LaunchConfiguration('rviz_view').perform(context)
        cfg = os.path.join(pkg, 'rviz', 'navigation.rviz' if view == 'follow' else 'overview.rviz')
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', cfg], parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('explore', default_value='false',
                              description='also start Nav2 (no AMCL) for autonomous exploration'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_view', default_value='overview', description='overview | follow'),
        OpaqueFunction(function=_setup),
    ])
