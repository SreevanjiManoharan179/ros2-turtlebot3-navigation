"""Everything for a demo in one command: Gazebo + TurtleBot3 + Nav2 + RViz (+ room labels).

    ros2 launch tb3_point_to_point bringup.launch.py
    ros2 launch tb3_point_to_point bringup.launch.py gui:=false rviz_view:=overview

Then, in a second terminal, send goals:
    ros2 run tb3_point_to_point go_to kitchen
    ros2 run tb3_point_to_point mission_runner --scenario tour
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

PKG = 'tb3_point_to_point'


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory(PKG), 'launch')
    pkg = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_view', default_value='follow', description='follow | overview'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'indoor_office.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'simulation.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        # give Gazebo a head start so the robot exists before AMCL needs its first scan
        TimerAction(period=6.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'), 'rviz': LaunchConfiguration('rviz'),
                              'rviz_view': LaunchConfiguration('rviz_view')}.items())]),
    ])
