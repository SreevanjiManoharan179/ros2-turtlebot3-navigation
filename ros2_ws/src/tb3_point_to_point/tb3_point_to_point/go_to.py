"""Point-to-point navigation to a named room (or x,y,yaw) with performance metrics.

    ros2 run tb3_point_to_point go_to kitchen
    ros2 run tb3_point_to_point go_to 1.9,-2.7,-90          # x,y[,yaw_deg] in the map frame
    ros2 run tb3_point_to_point go_to lab --init reception   # (re)localise at Reception first
    ros2 run tb3_point_to_point go_to --list                 # show the named locations

The goal is checked against the live map before it is sent (inside the map, on free
space, enough clearance for the robot); then the robot drives there with Nav2 and the
run is measured: time, driven vs planned path length, final position/yaw error,
recoveries and closest obstacle distance.
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy

from . import ui
from .locations import LocationBook
from .navigator_core import P2PNavigator, make_pose, print_leg_result

CSV_FIELDS = ['label', 'result', 'success', 'navigation_time_s', 'path_length_m',
              'planned_path_length_m', 'straight_line_m', 'path_efficiency', 'average_speed_mps',
              'goal_x', 'goal_y', 'goal_yaw_deg', 'final_x', 'final_y', 'final_yaw_deg',
              'final_position_error_m', 'final_yaw_error_deg', 'localization_error_m',
              'recoveries', 'replans', 'min_obstacle_distance_m', 'min_clearance_m', 'ground_truth']


def prepare(nav, book, init_spec=None):
    """Make sure AMCL has a pose and Nav2 is active (without resetting a good pose)."""
    nav.spin_for(1.5)  # receive the latched /amcl_pose, /map and /odom if they exist
    localised = nav.initial_pose_received
    end = time.monotonic() + 5.0
    while not localised and time.monotonic() < end:
        # the latched /amcl_pose can arrive late on a busy PC; a live map -> base_footprint
        # transform proves just as well that AMCL already has a pose - never overwrite that
        localised = nav.estimated_pose() is not None
        if not localised:
            nav.spin_for(0.25)
    if init_spec:
        loc = book.resolve(init_spec)
        ui.say(f'Setting initial pose at {loc.name} ({loc.x:.2f}, {loc.y:.2f})', 'cyan')
        nav.setInitialPose(make_pose(loc.x, loc.y, loc.yaw, stamp=nav.get_clock().now().to_msg()))
    elif not localised and book.spawn is not None:
        loc = book.spawn
        ui.say(f'AMCL has no pose yet - using the spawn pose ({loc.name}) as the initial pose', 'yellow')
        nav.setInitialPose(make_pose(loc.x, loc.y, loc.yaw, stamp=nav.get_clock().now().to_msg()))
    ui.say('Waiting for Nav2 to become active ...', 'grey')
    nav.wait_until_active()


def append_csv(path, row):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction='ignore')
        if new:
            w.writeheader()
        w.writerow(row)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('target', nargs='?', help='location name or "x,y[,yaw_deg]"')
    ap.add_argument('--list', action='store_true', help='list the named locations and exit')
    ap.add_argument('--init', default=None, help='set the AMCL initial pose first (name or x,y,yaw)')
    ap.add_argument('--locations', default=None, help='locations YAML (default: package config)')
    ap.add_argument('--timeout', type=float, default=300.0, help='goal timeout in (simulated) seconds')
    ap.add_argument('--csv', default=None, help='append the measured result to this CSV file')
    ap.add_argument('--no-check', action='store_true', help='skip the map clearance check')
    ap.add_argument('--wall-time', action='store_true', help='use wall-clock time (real robot)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:])

    book = LocationBook(args.locations)
    if args.list or not args.target:
        rows = [(n, f'{l.x:6.2f}', f'{l.y:6.2f}', f'{math.degrees(l.yaw):6.1f}', book.room_of(l.x, l.y) or '')
                for n, l in book.locations.items()]
        for line in ui.table(['name', 'x [m]', 'y [m]', 'yaw [deg]', 'room'], rows):
            print(line)
        if not args.target:
            print('\nusage: ros2 run tb3_point_to_point go_to <name | x,y[,yaw_deg]>')
        return 0

    try:
        goal = book.resolve(args.target)
    except KeyError as exc:
        ui.say(str(exc), 'red')
        return 2

    rclpy.init()
    nav = P2PNavigator('go_to', use_sim_time=not args.wall_time)
    try:
        prepare(nav, book, args.init)
        if not args.no_check:
            grid = nav.get_map(10.0)
            if grid is None:
                ui.say('No /map received - skipping the goal check', 'yellow')
            else:
                ok, reason, clearance = grid.check_goal(goal.x, goal.y, robot_radius=nav.robot_radius)
                if not ok:
                    ui.say(f'Goal rejected before sending: {reason}', 'red', 'bold')
                    return 2
        room = book.room_of(goal.x, goal.y)
        ui.banner(f'Point-to-point navigation -> {goal.name}',
                  f'goal x={goal.x:.2f} m, y={goal.y:.2f} m, yaw={math.degrees(goal.yaw):.0f} deg'
                  + (f'  ({room})' if room else ''))
        res = nav.navigate(goal.x, goal.y, goal.yaw, label=goal.name, timeout_s=args.timeout)
        print_leg_result(res)
        if args.csv:
            append_csv(args.csv, res)
            ui.say(f'  logged to {args.csv}', 'grey')
        return 0 if res['success'] else 1
    except TimeoutError as exc:
        ui.say(f'Nav2 is not ready: {exc}', 'red')
        return 3
    except KeyboardInterrupt:
        nav.cancel()
        return 130
    finally:
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
