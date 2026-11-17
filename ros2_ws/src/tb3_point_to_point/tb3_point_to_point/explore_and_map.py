"""Autonomous mapping: drive through every room with Nav2 while SLAM Toolbox builds the map.

    ros2 run tb3_point_to_point explore_and_map --save ~/my_maps/indoor_office

Start the simulation and 'slam.launch.py explore:=true' first (scripts/04_mapping.sh
--auto does both).  The robot visits the exploration waypoints from
config/scenarios.yaml, then the map is saved with nav2_map_server's map_saver_cli.

The saved map is anchored to the world frame: if the SLAM map started at the robot
(e.g. Gazebo Sim odometry starts at zero), the map origin is shifted by the spawn
pose so that every named location in config/locations.yaml stays valid.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import rclpy

from . import ui
from .locations import LocationBook, load_scenarios
from .map_tools import OccupancyMap, align_to_reference
from .navigator_core import P2PNavigator, print_leg_result


def save_map(path, use_sim_time=True):
    os.makedirs(os.path.dirname(os.path.abspath(path)) or '.', exist_ok=True)
    # --free 0.196: grey (205) pixels stay UNKNOWN when the map is loaded again
    # (with Nav2's default 0.25 they would be read back as free space)
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path, '--occ', '0.65', '--free', '0.196',
           '--ros-args', '-p', f'use_sim_time:={"true" if use_sim_time else "false"}',
           '-p', 'save_map_timeout:=10.0']
    ui.say(f'  saving map: {" ".join(cmd)}', 'grey')
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
    if out.returncode != 0 or not os.path.exists(path + '.yaml'):
        ui.say(out.stdout[-800:] + out.stderr[-800:], 'red')
        return False
    return True


def reanchor_map(yaml_path, dx, dy):
    """Shift the map origin by (dx, dy) so that map coordinates equal world coordinates."""
    with open(yaml_path) as f:
        lines = f.readlines()
    out = []
    for line in lines:
        if line.strip().startswith('origin:'):
            vals = [float(v) for v in line.split(':', 1)[1].strip().strip('[]').split(',')]
            vals[0] += dx
            vals[1] += dy
            line = f'origin: [{vals[0]:.3f}, {vals[1]:.3f}, {vals[2]:.3f}]\n'
        out.append(line)
    with open(yaml_path, 'w') as f:
        f.writelines(out)


def world_to_map_transform(spawn, start_map):
    """(x, y, yaw) in the world frame -> the SLAM map frame, from the robot's spawn pose (world)
    and its first pose in the map frame. Identity if either is unknown."""
    if spawn is None or start_map is None:
        return lambda x, y, yaw: (x, y, yaw)
    dth = start_map[2] - spawn.yaw
    c, s = math.cos(dth), math.sin(dth)

    def to_map(x, y, yaw):
        wx, wy = x - spawn.x, y - spawn.y
        return start_map[0] + c * wx - s * wy, start_map[1] + s * wx + c * wy, yaw + dth
    return to_map


def anchor_to_reference(yaml_path, reference_yaml):
    """Shift the new map onto a floor plan of the building and report the wall accuracy."""
    try:
        slam, ref = OccupancyMap.from_yaml(yaml_path), OccupancyMap.from_yaml(reference_yaml)
    except (OSError, ValueError) as exc:
        ui.say(f'  floor plan not used ({exc})', 'yellow')
        return
    dx, dy, before, after = align_to_reference(slam, ref)
    if not (math.hypot(dx, dy) > 0.005 and after['mean_m'] < before['mean_m']):
        dx = dy = 0.0
        after = before
    # AMCL places every map cell half a cell lower-left of where map_server / the costmaps put it;
    # shifting the origin by half a cell removes that localisation bias (see maps/indoor_office.yaml)
    half = slam.resolution / 2.0
    reanchor_map(yaml_path, dx + half, dy + half)
    ui.say(f'  anchored to the floor plan: origin shifted by ({dx:+.3f}, {dy:+.3f}) m '
           f'(+ half a cell, {half:.3f} m, for AMCL)', 'grey')
    ui.say(f'  map accuracy vs the floor plan: walls on average {100 * after["mean_m"]:.1f} cm from the plan, '
           f'{after["within_5cm_pct"]:.1f} % within 5 cm, {after["within_10cm_pct"]:.1f} % within 10 cm', 'green')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--save', required=True, help='output map path WITHOUT extension (writes .yaml + .pgm)')
    ap.add_argument('--scenarios', default=None, help='scenarios YAML with exploration.waypoints')
    ap.add_argument('--locations', default=None, help='locations YAML (for the spawn pose)')
    ap.add_argument('--timeout', type=float, default=180.0, help='timeout per waypoint (simulated s)')
    ap.add_argument('--reference', default=None,
                    help='floor-plan map (.yaml) of the same building: anchor the new map to it and '
                         'report how far the mapped walls are from the plan')
    ap.add_argument('--wall-time', action='store_true')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:])

    scenarios, _ = load_scenarios(args.scenarios)
    book = LocationBook(args.locations)
    waypoints = (scenarios.get('exploration') or {}).get('waypoints') or []
    if not waypoints:
        ui.say('no exploration.waypoints in the scenarios file', 'red')
        return 2

    rclpy.init()
    nav = P2PNavigator('explore_and_map', use_sim_time=not args.wall_time)
    code = 0
    try:
        ui.say('Waiting for Nav2 (SLAM mode, no AMCL) ...', 'grey')
        nav.wait_until_active(localizer='controller_server')
        nav.wait_for_odom(20.0)
        if nav.get_map(30.0) is None:
            ui.say('no /map from SLAM Toolbox - is slam.launch.py running?', 'red')
            return 3
        # where does the robot start in the SLAM map frame?  (used to anchor the map)
        start_map = None
        end = time.monotonic() + 10.0
        while start_map is None and time.monotonic() < end:
            start_map = nav.estimated_pose()
            nav.spin_for(0.2)

        # SLAM's map frame starts where the robot's odometry starts: at the world origin in Gazebo
        # Classic (odometry = world pose), at the spawn pose in Gazebo Sim. The waypoints are world
        # coordinates, so convert them with the robot's spawn pose (world) <-> start pose (map).
        spawn = book.spawn
        to_map = world_to_map_transform(spawn, start_map)

        ui.banner('Autonomous mapping', f'{len(waypoints)} waypoints through every room')
        nav.reset_mission_trail()
        for i, wp in enumerate(waypoints, 1):
            loc = book.resolve(wp, name=f'waypoint {i}')
            room = book.room_of(loc.x, loc.y) or ''
            ui.say(f'[{i}/{len(waypoints)}] -> ({loc.x:.2f}, {loc.y:.2f}) {room}', 'cyan')
            mx, my, myaw = to_map(loc.x, loc.y, loc.yaw)
            res = nav.navigate(mx, my, myaw, label=room or loc.name, timeout_s=args.timeout,
                               status_period_s=4.0)
            print_leg_result(res, i, len(waypoints))
            if not res['success']:
                nav.clear_costmaps()

        nav.spin_for(2.0)  # let SLAM Toolbox integrate the last scans
        if not save_map(args.save, use_sim_time=not args.wall_time):
            return 4
        if spawn is not None and start_map is not None:
            dx, dy = spawn.x - start_map[0], spawn.y - start_map[1]
            if math.hypot(dx, dy) > 0.05:
                reanchor_map(args.save + '.yaml', dx, dy)
                ui.say(f'  map origin shifted by ({dx:.2f}, {dy:.2f}) m to match the world frame', 'grey')
        if args.reference:
            anchor_to_reference(args.save + '.yaml', args.reference)
        ui.say(f'Map saved: {args.save}.yaml / .pgm', 'green', 'bold')
    except TimeoutError as exc:
        ui.say(f'Nav2 is not ready: {exc}', 'red')
        code = 3
    except KeyboardInterrupt:
        nav.cancel()
        code = 130
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
