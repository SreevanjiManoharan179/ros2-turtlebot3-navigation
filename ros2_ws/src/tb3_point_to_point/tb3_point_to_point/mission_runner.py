"""Run a multi-goal mission or the evaluation test table and log every metric.

    ros2 run tb3_point_to_point mission_runner --scenario tour        # visit every room
    ros2 run tb3_point_to_point mission_runner --scenario evaluation  # test table T1..T8
    ros2 run tb3_point_to_point mission_runner --scenario evaluation --repeat 3 --out results/eval

Scenarios live in config/scenarios.yaml.  Each evaluation trial has its own start
pose: the robot is teleported there through Gazebo's /gazebo/set_entity_state
service and AMCL is re-initialised (falls back to driving there if the service is
missing).  Trials can add an obstacle that is NOT in the map; it is spawned in
Gazebo just before the trial and removed afterwards, to test detection and
re-planning.  Results go to <out>/trials.csv, run.json (with trajectories), a
copy of the map, summary.md and plots.
"""
import argparse
import csv
import datetime
import json
import math
import os
import shutil
import subprocess
import sys
import time

import rclpy

try:  # Gazebo Classic (Humble). Gazebo Sim (Jazzy) has no gazebo_msgs: the gz CLI is used instead.
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import DeleteEntity, SetEntityState, SpawnEntity
except ImportError:
    EntityState = DeleteEntity = SetEntityState = SpawnEntity = None

from . import metrics as M
from . import ui
from .go_to import CSV_FIELDS, prepare
from .locations import LocationBook, default_path, load_scenarios
from .navigator_core import P2PNavigator, make_pose, print_leg_result

ROBOT_ENTITY = os.environ.get('TURTLEBOT3_MODEL', 'burger')

OBSTACLE_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <pose>0 0 {hz:.3f} 0 0 0</pose>
      <collision name='collision'><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
      <visual name='visual'><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        <material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>"""


class GazeboTools:
    """Teleport the robot and spawn/remove obstacles in Gazebo Classic (gazebo_ros
    services) or Gazebo Sim (gz transport services through the gz / ros_gz_sim CLIs)."""

    def __init__(self, node):
        self.node = node
        self.set_state = self.spawn = self.delete = None
        if SpawnEntity is not None:
            self.set_state = node.create_client(SetEntityState, '/gazebo/set_entity_state')
            self.spawn = node.create_client(SpawnEntity, '/spawn_entity')
            self.delete = node.create_client(DeleteEntity, '/delete_entity')
        self._backend = None

    def backend(self):
        if self._backend is None:
            if self.spawn is not None and self.spawn.wait_for_service(timeout_sec=3.0):
                self._backend = 'classic'
            elif shutil.which('gz'):
                self._backend = 'gz'
            else:
                self._backend = 'none'
        return self._backend

    def _call(self, client, request, timeout_s=10.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        future = client.call_async(request)
        end = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.result() if future.done() else None

    @staticmethod
    def _gz_service(service, reqtype, req, timeout_ms=5000):
        cmd = ['gz', 'service', '-s', service, '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
               '--timeout', str(timeout_ms), '--req', req]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_ms / 1000.0 + 5)
        except (OSError, subprocess.TimeoutExpired):
            return False
        return out.returncode == 0 and 'true' in out.stdout.lower()

    def can_teleport(self):
        b = self.backend()
        if b == 'classic':
            return self.set_state.wait_for_service(timeout_sec=3.0)
        return b == 'gz'

    def teleport(self, x, y, yaw):
        b = self.backend()
        if b == 'classic':
            req = SetEntityState.Request()
            req.state = EntityState()
            req.state.name = ROBOT_ENTITY
            req.state.pose = make_pose(x, y, yaw).pose
            req.state.pose.position.z = 0.01
            req.state.reference_frame = 'world'
            res = self._call(self.set_state, req)
            return bool(res and res.success)
        if b == 'gz':
            q = M.quaternion_from_yaw(yaw)
            return self._gz_service('/world/default/set_pose', 'gz.msgs.Pose',
                                    f'name: "{ROBOT_ENTITY}" position: {{x: {x}, y: {y}, z: 0.01}} '
                                    f'orientation: {{x: 0, y: 0, z: {q[2]}, w: {q[3]}}}')
        return False

    def spawn_box(self, name, x, y, yaw, size, color):
        sx, sy, sz = [float(v) for v in size]
        r, g, b = [float(v) for v in color]
        sdf = OBSTACLE_SDF.format(name=name, sx=sx, sy=sy, sz=sz, hz=sz / 2.0, r=r, g=g, b=b)
        backend = self.backend()
        if backend == 'classic':
            req = SpawnEntity.Request()
            req.name = name
            req.xml = sdf
            req.initial_pose = make_pose(x, y, yaw).pose
            req.reference_frame = 'world'
            res = self._call(self.spawn, req)
            return bool(res and res.success)
        if backend == 'gz':
            cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'default', '-name', name,
                   '-string', sdf, '-x', str(x), '-y', str(y), '-z', '0.0', '-Y', str(yaw)]
            try:
                return subprocess.run(cmd, capture_output=True, text=True, timeout=60).returncode == 0
            except (OSError, subprocess.TimeoutExpired):
                return False
        return False

    def delete_entity(self, name):
        backend = self.backend()
        if backend == 'classic':
            req = DeleteEntity.Request()
            req.name = name
            res = self._call(self.delete, req)
            return bool(res and res.success)
        if backend == 'gz':
            return self._gz_service('/world/default/remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')
        return False


def save_map_copy(grid_msg, out_dir):
    """Write the live /map as PGM + YAML next to the results (used for the plots)."""
    if grid_msg is None:
        return None
    info = grid_msg.info
    w, h = info.width, info.height
    rows = []
    for r in range(h - 1, -1, -1):
        row = bytearray(w)
        base = r * w
        for c in range(w):
            v = grid_msg.data[base + c]
            row[c] = 205 if v < 0 else (0 if v >= 65 else 254)
        rows.append(bytes(row))
    with open(os.path.join(out_dir, 'map.pgm'), 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(b''.join(rows))
    with open(os.path.join(out_dir, 'map.yaml'), 'w') as f:
        f.write(f'image: map.pgm\nmode: trinary\nresolution: {info.resolution:.3f}\n'
                f'origin: [{info.origin.position.x:.3f}, {info.origin.position.y:.3f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return os.path.join(out_dir, 'map.yaml')


class MissionRunner:

    def __init__(self, nav, book, scenarios, args):
        self.nav = nav
        self.book = book
        self.scen = scenarios
        self.args = args
        self.gz = GazeboTools(nav)
        self.grid = None
        self.teleport_ok = None
        self.results = []

    # ----------------------------------------------------------------- setup
    def reposition(self, start):
        """Put the robot at the trial's start pose (teleport + relocalise, or drive)."""
        cur = self.nav.estimated_pose() or self.nav.odom_pose
        if cur and math.hypot(cur[0] - start.x, cur[1] - start.y) < 0.35:
            return 'already there'
        if self.teleport_ok is None:
            self.teleport_ok = (not self.args.no_teleport) and self.gz.can_teleport()
        if self.teleport_ok and self.gz.teleport(start.x, start.y, start.yaw):
            self.nav.spin_for(0.5)
            self.nav.set_initial_pose_and_wait(start.x, start.y, start.yaw)
            self.nav.clear_costmaps()
            self.nav.spin_for(2.5)   # let AMCL, TF and the costmaps settle at the new pose
            return 'teleported'
        ui.say(f'    driving to the start pose ({start.name}) first - not measured', 'grey')
        quiet, self.nav.quiet = self.nav.quiet, True
        self.nav.navigate(start.x, start.y, start.yaw, label=f'start:{start.name}',
                          timeout_s=self.args.timeout)
        self.nav.quiet = quiet
        return 'driven'

    def check_goal(self, goal):
        if self.grid is None or self.args.no_check:
            return True, 'not checked'
        ok, reason, _ = self.grid.check_goal(goal.x, goal.y, robot_radius=self.nav.robot_radius)
        return ok, reason

    # ------------------------------------------------------------------- run
    def run_leg(self, trial, index, total, repeat):
        goal = self.book.resolve(trial['goal'])
        trial_id = str(trial.get('id', f'L{index}'))
        start = self.book.resolve(trial['start']) if trial.get('start') else None
        start_room = self.book.room_of(start.x, start.y) if start else None
        goal_room = self.book.room_of(goal.x, goal.y)
        route = f'{start.name if start else "current pose"} -> {goal.name}'
        ui.say('')
        ui.say(f'[{index}/{total}] {trial_id}  {route}', 'cyan', 'bold')
        if trial.get('note'):
            ui.say(f'    {trial["note"]}', 'grey')
        if start is not None:
            how = self.reposition(start)
            ui.say(f'    start pose: {start.name} ({how})', 'grey')

        ok, reason = self.check_goal(goal)
        if not ok:
            ui.say(f'    goal rejected before sending: {reason}', 'red')
            res = {'label': goal.name, 'result': 'INVALID_GOAL', 'success': False,
                   'goal_x': goal.x, 'goal_y': goal.y, 'goal_yaw_deg': math.degrees(goal.yaw),
                   'trajectory': []}
        else:
            obstacle_name = trial.get('obstacle')
            spawned = None
            if obstacle_name:
                cfg = (self.scen.get('obstacles') or {})[obstacle_name]
                entity = f'unexpected_{obstacle_name}'
                if self.gz.spawn_box(entity, cfg['x'], cfg['y'], math.radians(cfg.get('yaw_deg', 0.0)),
                                     cfg.get('size', [0.5, 0.5, 0.5]), cfg.get('color', [1.0, 0.45, 0.0])):
                    spawned = entity
                    ui.say(f'    !! unexpected obstacle placed at ({cfg["x"]:.1f}, {cfg["y"]:.1f}) '
                           '- it is NOT in the map', 'yellow', 'bold')
                else:
                    ui.say('    could not spawn the obstacle (is Gazebo running?)', 'yellow')
            res = self.nav.navigate(goal.x, goal.y, goal.yaw, label=goal.name, timeout_s=self.args.timeout)
            if spawned:
                self.gz.delete_entity(spawned)
                self.nav.clear_costmaps()
                self.nav.spin_for(0.5)
            res['obstacle'] = obstacle_name or ''
            if spawned:   # geometry for the trajectory plot
                res['obstacle_box'] = [cfg['x'], cfg['y'], cfg.get('yaw_deg', 0.0)] + \
                    list(cfg.get('size', [0.5, 0.5, 0.5]))[:2]
            print_leg_result(res, index, total)
            if self.args.pause > 0:
                self.nav.spin_for(self.args.pause)

        res.update({
            'trial_id': trial_id,
            'repeat': repeat,
            'start': start.name if start else '',
            'goal': goal.name,
            'start_room': start_room or '',
            'goal_room': goal_room or '',
            'note': trial.get('note', ''),
        })
        self.results.append(res)
        return res

    def build_plan(self):
        name = self.args.scenario
        scen = self.scen.get(name)
        if scen is None:
            raise KeyError(f'scenario {name!r} not found in the scenarios file '
                           f'(available: {", ".join(k for k in self.scen if isinstance(self.scen[k], dict))})')
        repeat = int(self.args.repeat or scen.get('repeat', 1))
        plan = []
        if 'legs' in scen:  # sequential tour: each leg starts where the previous ended
            for r in range(1, repeat + 1):
                for i, goal in enumerate(scen['legs'], 1):
                    plan.append(({'id': f'L{i}', 'goal': goal}, r))
        else:
            for r in range(1, repeat + 1):
                for trial in scen['trials']:
                    plan.append((trial, r))
        return scen, plan

    def run(self):
        scen, plan = self.build_plan()
        ui.banner(f'Mission: {self.args.scenario}', scen.get('description', ''))
        self.nav.reset_mission_trail()
        self.grid = self.nav.get_map(10.0)
        # A mission whose first leg starts at "the current pose" (the tour) begins at the scenario's
        # initial pose. Put the robot there for real (teleport + relocalise, or drive) instead of
        # only telling AMCL: after another mission the robot can be anywhere on the floor.
        start_name = self.scen.get('initial_pose')
        if plan and not plan[0][0].get('start') and start_name and not self.args.init:
            loc = self.book.resolve(start_name)
            ui.say(f'Mission start: {loc.name} ({self.reposition(loc)})', 'grey')
        total = len(plan)
        t0 = time.monotonic()
        for i, (trial, repeat) in enumerate(plan, 1):
            self.run_leg(trial, i, total, repeat)
        self.print_summary(time.monotonic() - t0)

    # --------------------------------------------------------------- output
    def print_summary(self, wall):
        ui.say('')
        ui.banner('Results', f'{len(self.results)} goals, wall-clock {wall / 60.0:.1f} min')
        rows = []
        for r in self.results:
            rows.append([r['trial_id'], f'{r["start"] or "-"} -> {r["goal"]}', r['result'],
                         ui.fmt(r.get('navigation_time_s'), 1), ui.fmt(r.get('path_length_m')),
                         ui.fmt(r.get('planned_path_length_m')), ui.fmt(r.get('final_position_error_m'), 3),
                         ui.fmt(r.get('final_yaw_error_deg'), 1), str(r.get('recoveries', '-'))])
        for line in ui.table(['id', 'route', 'result', 'time s', 'path m', 'plan m', 'err m', 'yaw err', 'rec'],
                             rows, ['<', '<', '<', '>', '>', '>', '>', '>', '>']):
            ui.say('  ' + line)
        s = M.summarize(self.results)
        ui.say('')
        color = 'green' if s['successful'] == s['trials'] else 'yellow'
        ui.say(f'  Success rate: {s["successful"]}/{s["trials"]} = {s["success_rate_pct"]:.0f} %', color, 'bold')
        for key, label, unit, d in [('final_position_error_m', 'final position error', 'm', 3),
                                    ('final_yaw_error_deg', 'final yaw error', 'deg', 1),
                                    ('navigation_time_s', 'navigation time', 's', 1),
                                    ('path_efficiency', 'path efficiency (planned/driven)', '', 2)]:
            m, sd = s[key]['mean'], s[key]['std']
            if m is not None:
                ui.say(f'  {label}: {m:.{d}f} +/- {sd:.{d}f} {unit}')

    def save(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, 'trials.csv'), 'w', newline='') as f:
            fields = ['trial_id', 'repeat', 'start', 'goal', 'start_room', 'goal_room', 'obstacle'] + \
                     [c for c in CSV_FIELDS if c != 'label'] + ['note']
            w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            w.writeheader()
            for r in self.results:
                w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in r.items()})
        map_yaml = save_map_copy(self.nav._latest_map, out_dir)
        meta = {
            'scenario': self.args.scenario,
            'created': datetime.datetime.now().isoformat(timespec='seconds'),
            'ros_distro': os.environ.get('ROS_DISTRO', ''),
            'robot': ROBOT_ENTITY,
            'map_yaml': os.path.basename(map_yaml) if map_yaml else None,
            'locations_file': self.book.path,
            'summary': M.summarize(self.results),
            'trials': self.results,
        }
        with open(os.path.join(out_dir, 'run.json'), 'w') as f:
            json.dump(meta, f, indent=1)
        ui.say(f'\n  results saved in {out_dir}', 'grey')
        try:
            from .analyze_results import write_report
            write_report([out_dir], out_dir, quiet=True)
            ui.say(f'  report: {os.path.join(out_dir, "summary.md")} (+ plots)', 'grey')
        except Exception as exc:  # plotting is optional
            ui.say(f'  (report/plots skipped: {exc})', 'yellow')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--scenario', default='tour', help='scenario name in the scenarios file (tour, evaluation, ...)')
    ap.add_argument('--scenarios', default=None, help='scenarios YAML (default: package config/scenarios.yaml)')
    ap.add_argument('--locations', default=None, help='locations YAML (default: package config/locations.yaml)')
    ap.add_argument('--repeat', type=int, default=None, help='override the number of repetitions')
    ap.add_argument('--init', default=None, help='tell AMCL the robot is at this location now (no teleport)')
    ap.add_argument('--out', default=None, help='results directory (default: ./results/<scenario>_<time>)')
    ap.add_argument('--timeout', type=float, default=None, help='per-goal timeout in simulated seconds')
    ap.add_argument('--pause', type=float, default=1.0, help='pause between goals (s)')
    ap.add_argument('--no-teleport', action='store_true', help='always drive to trial start poses')
    ap.add_argument('--no-check', action='store_true', help='skip the map clearance check')
    ap.add_argument('--wall-time', action='store_true', help='use wall-clock time (real robot)')
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:])

    scenarios, scen_path = load_scenarios(args.scenarios)
    book = LocationBook(args.locations)
    if args.timeout is None:
        args.timeout = float(scenarios.get('default_timeout_s', 240.0))
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out_dir = os.path.abspath(args.out or os.path.join('results', f'{args.scenario}_{stamp}'))

    rclpy.init()
    nav = P2PNavigator('mission_runner', use_sim_time=not args.wall_time)
    runner = MissionRunner(nav, book, scenarios, args)
    code = 0
    try:
        prepare(nav, book, args.init)   # only an explicit --init overrides the current AMCL pose
        runner.run()
        code = 0 if all(r['success'] for r in runner.results) else 1
    except TimeoutError as exc:
        ui.say(f'Nav2 is not ready: {exc}', 'red')
        code = 3
    except KeyboardInterrupt:
        ui.say('\ninterrupted - cancelling the current goal', 'yellow')
        nav.cancel()
        code = 130
    except KeyError as exc:
        ui.say(str(exc), 'red')
        code = 2
    finally:
        if runner.results:
            runner.save(out_dir)
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
