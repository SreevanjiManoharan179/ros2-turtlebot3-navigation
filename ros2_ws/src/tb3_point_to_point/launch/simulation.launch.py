"""Start Gazebo with the indoor office world and spawn a TurtleBot3.

    ros2 launch tb3_point_to_point simulation.launch.py                 # Gazebo GUI + indoor office
    ros2 launch tb3_point_to_point simulation.launch.py gui:=false       # headless (faster)
    ros2 launch tb3_point_to_point simulation.launch.py world:=turtlebot3_world x_pose:=-2.0 y_pose:=-0.5

ROS 2 Humble  -> Gazebo Classic 11 through gazebo_ros   (worlds/indoor_office.world)
ROS 2 Jazzy+  -> Gazebo Sim (Harmonic) through ros_gz    (worlds/indoor_office_gz.sdf)
Set TB3_SIMULATOR=classic or TB3_SIMULATOR=gz to force one of them.
"""
import math
import os

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb3_point_to_point'


def _share(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def simulator():
    forced = os.environ.get('TB3_SIMULATOR', '').strip().lower()
    if forced in ('classic', 'gz'):
        return forced
    if os.environ.get('ROS_DISTRO', '') in ('humble', 'iron') and _share('gazebo_ros'):
        return 'classic'
    if _share('ros_gz_sim'):
        return 'gz'
    return 'classic'


def _spawn_pose():
    with open(os.path.join(_share(PKG), 'config', 'locations.yaml')) as f:
        data = yaml.safe_load(f)
    loc = data['locations'][data['spawn']]
    return str(loc['x']), str(loc['y']), f"{math.radians(loc.get('yaw_deg', 0.0)):.4f}"


def _setup(context):
    os.environ.setdefault('TURTLEBOT3_MODEL', 'burger')
    model = os.environ['TURTLEBOT3_MODEL']
    sim = simulator()
    pkg, tb3 = _share(PKG), _share('turtlebot3_gazebo')

    world = LaunchConfiguration('world').perform(context)
    if world in ('', 'indoor_office'):
        world = os.path.join(pkg, 'worlds', 'indoor_office.world' if sim == 'classic' else 'indoor_office_gz.sdf')
    elif not os.path.isabs(world):
        world = os.path.join(tb3, 'worlds', world if world.endswith('.world') else world + '.world')
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    x, y, yaw = (LaunchConfiguration(k).perform(context) for k in ('x_pose', 'y_pose', 'yaw'))
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_sdf = os.path.join(tb3, 'models', f'turtlebot3_{model}', 'model.sdf')

    actions = [LogInfo(msg=f'[{PKG}] simulator: {"Gazebo Classic" if sim == "classic" else "Gazebo Sim"}, '
                           f'world: {world}, robot: {model} at ({x}, {y}, yaw {yaw})')]
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())

    if sim == 'classic':
        gazebo_ros = _share('gazebo_ros')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()))
        if gui:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'))))
        actions.append(rsp)
        actions.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', output='screen',
            arguments=['-entity', model, '-file', robot_sdf, '-x', x, '-y', y, '-z', '0.01', '-Y', yaw,
                       '-timeout', '120']))
    else:
        ros_gz_sim = _share('ros_gz_sim')
        actions.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}.items()))
        if gui:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
                launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()))
        actions.append(rsp)
        actions.append(Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-name', model, '-file', robot_sdf, '-x', x, '-y', y, '-z', '0.01', '-Y', yaw]))
        bridge = os.path.join(tb3, 'params', f'turtlebot3_{model}_bridge.yaml')
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', output='screen',
            arguments=['--ros-args', '-p', f'config_file:={bridge}']))
    return actions


def generate_launch_description():
    x, y, yaw = _spawn_pose()
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='indoor_office',
                              description='indoor_office (default), turtlebot3_world, turtlebot3_house '
                                          'or an absolute path to a world file'),
        DeclareLaunchArgument('gui', default_value='true', description='start the Gazebo GUI'),
        DeclareLaunchArgument('x_pose', default_value=x, description='spawn x [m]'),
        DeclareLaunchArgument('y_pose', default_value=y, description='spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value=yaw, description='spawn yaw [rad]'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_setup),
    ])
