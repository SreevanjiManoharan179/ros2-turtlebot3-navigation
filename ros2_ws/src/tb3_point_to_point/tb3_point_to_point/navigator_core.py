"""P2PNavigator: Nav2's BasicNavigator plus ground-truth recording and per-goal metrics.

Written against the ROS 2 Humble nav2_simple_commander API (isTaskComplete() with no
arguments, TaskResult enum, NavigateToPose feedback with navigation_time /
number_of_recoveries / distance_remaining).  It also runs on newer distros.

Ground truth: in Gazebo the TurtleBot3 diff-drive plugin publishes /odom from the
simulator's world pose, so /odom is the robot's true pose in the world frame.  The
project's maps are built in that same frame, so comparing /odom with the goal gives
the true final error; the AMCL estimate (TF map->base_footprint) gives the
localisation error.  If the map frame turns out not to match the world frame (for
example a Cartographer map started elsewhere), the AMCL estimate is used instead.
"""
import math
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener

from . import metrics as M
from . import ui
from .map_tools import OccupancyMap

LATCHED = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=QoSReliabilityPolicy.RELIABLE)

RESULT_NAMES = {
    TaskResult.SUCCEEDED: 'SUCCEEDED',
    TaskResult.CANCELED: 'CANCELED',
    TaskResult.FAILED: 'FAILED',
    TaskResult.UNKNOWN: 'UNKNOWN',
}


def make_pose(x, y, yaw, frame_id='map', stamp=None):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    if stamp is not None:
        pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    qx, qy, qz, qw = M.quaternion_from_yaw(float(yaw))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def _color(marker, r, g, b, a=1.0):
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = float(r), float(g), float(b), float(a)


# one colour per goal, so the RViz trail shows which leg was driven where
TRAIL_COLORS = [
    (0.20, 0.85, 0.45), (1.00, 0.55, 0.10), (0.25, 0.65, 1.00), (0.95, 0.30, 0.70),
    (1.00, 0.90, 0.20), (0.60, 0.45, 1.00), (0.20, 0.90, 0.90), (1.00, 0.35, 0.30),
]


class P2PNavigator(BasicNavigator):

    def __init__(self, node_name='p2p_navigator', use_sim_time=True, robot_radius=0.105):
        super().__init__(node_name=node_name)
        if use_sim_time:
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.robot_radius = robot_radius
        self.odom_pose = None          # (x, y, yaw) ground truth in simulation
        self.odom_speed = 0.0
        self._recording = False
        self._track = []               # points of the current goal
        self._mission_track = []       # all points since the mission started (RViz trail)
        self._legs = []                # [(colour, [points])] one entry per goal, for the trail
        self._min_range = math.inf
        self._plan_length = None
        self._plan_events = []
        self._latest_map = None
        self._have_initial_pose = False
        self.quiet = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, 'odom', self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(Path, 'plan', self._plan_cb, 10)
        self.create_subscription(OccupancyGrid, 'map', self._map_cb, LATCHED)
        self.trajectory_pub = self.create_publisher(Path, 'p2p/trajectory', 10)
        self.trail_pub = self.create_publisher(MarkerArray, 'p2p/trail', 10)
        self.status_pub = self.create_publisher(MarkerArray, 'p2p/status', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'p2p/goal', LATCHED)

    # ------------------------------------------------------------ callbacks
    def _odom_cb(self, msg):
        p = msg.pose.pose
        yaw = M.yaw_from_quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        self.odom_pose = (p.position.x, p.position.y, yaw)
        self.odom_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        if self._recording:
            pt = (p.position.x, p.position.y)
            if not self._track or math.hypot(pt[0] - self._track[-1][0], pt[1] - self._track[-1][1]) >= 0.01:
                self._track.append(pt)
                self._mission_track.append(pt)
                if self._legs:
                    self._legs[-1][1].append(pt)

    def _scan_cb(self, msg):
        if not self._recording:
            return
        lo, hi = max(msg.range_min, 0.0), msg.range_max
        for r in msg.ranges:
            if lo < r < hi and r < self._min_range:
                self._min_range = r

    def _plan_cb(self, msg):
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if len(pts) < 2:
            return
        length = M.path_length(pts)
        if self._recording and self._plan_length is not None and abs(length - self._plan_length) > 1.5:
            self._plan_events.append((self._plan_length, length))
            if not self.quiet:
                ui.say(f'      route changed: {self._plan_length:.1f} m -> {length:.1f} m '
                       '(obstacle seen, re-planning)', 'yellow')
        self._plan_length = length

    def _map_cb(self, msg):
        self._latest_map = msg

    # ------------------------------------------- robust Nav2 calls (timeouts)
    # BasicNavigator blocks forever if a service/action reply is lost; these
    # versions keep the same state (goal_handle, result_future, status, feedback)
    # so isTaskComplete()/getResult() still work, but they give up after a timeout.
    def _spin_future(self, future, timeout_s):
        end = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.done()

    def node_state(self, node_name, timeout_s=3.0):
        client = self.create_client(GetState, f'{node_name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return None
            future = client.call_async(GetState.Request())
            if not self._spin_future(future, timeout_s) or future.result() is None:
                return None
            return future.result().current_state.label
        finally:
            self.destroy_client(client)

    def setInitialPose(self, initial_pose):
        self._have_initial_pose = True
        super().setInitialPose(initial_pose)

    def wait_until_active(self, localizer='amcl', navigator='bt_navigator', timeout_s=300.0):
        """Like BasicNavigator.waitUntilNav2Active() but with a timeout and progress messages.

        Unlike the original it never publishes a made-up (0, 0) initial pose: it only
        re-sends a pose that was given with setInitialPose().
        """
        deadline = time.monotonic() + timeout_s
        for name in [n for n in (localizer, navigator) if n]:
            last_note = time.monotonic()
            while True:
                state = self.node_state(name)
                if state == 'active':
                    break
                if time.monotonic() > deadline:
                    raise TimeoutError(f'{name} did not become active (last state: {state})')
                if time.monotonic() - last_note > 10.0:
                    ui.say(f'  still waiting for {name} (state: {state or "not available"}) ...', 'grey')
                    last_note = time.monotonic()
                self.spin_for(1.0)
            if name == 'amcl':
                last_pub = 0.0
                while not self.initial_pose_received:
                    if time.monotonic() > deadline:
                        raise TimeoutError('AMCL has no pose - set it with "2D Pose Estimate" in RViz '
                                           'or pass --init <location>')
                    if self._have_initial_pose and time.monotonic() - last_pub > 2.0:
                        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
                        self._setInitialPose()
                        last_pub = time.monotonic()
                    rclpy.spin_once(self, timeout_sec=0.2)
        return True

    def send_goal(self, pose, timeout_s=20.0):
        """BasicNavigator.goToPose() with timeouts. Returns True if Nav2 accepted the goal."""
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=timeout_s):
            self.get_logger().error('navigate_to_pose action server not available')
            return False
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.result_future = None
        self.status = None
        self.feedback = None
        future = self.nav_to_pose_client.send_goal_async(goal, self._feedbackCallback)
        if not self._spin_future(future, timeout_s) or future.result() is None:
            self.get_logger().error('no answer from navigate_to_pose')
            return False
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('goal rejected by Nav2')
            return False
        self.result_future = self.goal_handle.get_result_async()
        return True

    def cancel(self, timeout_s=5.0):
        if self.goal_handle is not None and self.result_future is not None and not self.result_future.done():
            self._spin_future(self.goal_handle.cancel_goal_async(), timeout_s)

    def plan_path(self, goal, timeout_s=10.0):
        """Ask the planner for a path from the current pose (None if unavailable)."""
        client = self.compute_path_to_pose_client
        if not client.wait_for_server(timeout_sec=2.0):
            return None
        req = ComputePathToPose.Goal()
        req.goal = goal
        req.use_start = False
        future = client.send_goal_async(req)
        if not self._spin_future(future, timeout_s) or future.result() is None or not future.result().accepted:
            return None
        result = future.result().get_result_async()
        if not self._spin_future(result, timeout_s) or result.result() is None:
            return None
        if result.result().status != GoalStatus.STATUS_SUCCEEDED:
            return None
        return result.result().result.path

    def clear_costmaps(self, timeout_s=5.0):
        for srv in ('global_costmap/clear_entirely_global_costmap',
                    'local_costmap/clear_entirely_local_costmap'):
            client = self.create_client(ClearEntireCostmap, srv)
            try:
                if client.wait_for_service(timeout_sec=2.0):
                    self._spin_future(client.call_async(ClearEntireCostmap.Request()), timeout_s)
            finally:
                self.destroy_client(client)

    # -------------------------------------------------------------- helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_odom(self, timeout_s=20.0):
        end = time.monotonic() + timeout_s
        while self.odom_pose is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom_pose is not None

    def estimated_pose(self, frame='map', base='base_footprint'):
        """Robot pose as Nav2 believes it (AMCL / SLAM via TF), or None."""
        try:
            t = self.tf_buffer.lookup_transform(frame, base, Time())
        except Exception:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, M.yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def get_map(self, timeout_s=15.0):
        end = time.monotonic() + timeout_s
        while self._latest_map is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return OccupancyMap.from_msg(self._latest_map) if self._latest_map is not None else None

    def set_initial_pose_and_wait(self, x, y, yaw, timeout_s=20.0):
        """(Re)initialise AMCL at a known pose and wait until it reports back."""
        self.setInitialPose(make_pose(x, y, yaw, stamp=self.get_clock().now().to_msg()))
        end = time.monotonic() + timeout_s
        last = time.monotonic()
        while not self.initial_pose_received and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - last > 3.0:
                self.initial_pose.header.stamp = self.get_clock().now().to_msg()
                self._setInitialPose()
                last = time.monotonic()
        return self.initial_pose_received

    def reset_mission_trail(self):
        self._mission_track = []
        self._legs = []
        clear = Marker()
        clear.action = Marker.DELETEALL
        self.trail_pub.publish(MarkerArray(markers=[clear]))
        self.publish_trajectory()

    def publish_trajectory(self):
        stamp = self.get_clock().now().to_msg()
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = stamp
        for x, y in self._mission_track:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.trajectory_pub.publish(msg)
        trail = MarkerArray()
        for i, (rgb, pts) in enumerate(self._legs):
            if len(pts) < 2:
                continue
            m = Marker()
            m.header.frame_id = 'odom'
            m.header.stamp = stamp
            m.ns, m.id, m.type, m.action = 'trail', i, Marker.LINE_STRIP, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = 0.06
            _color(m, *rgb)
            for x, y in pts[::2] + [pts[-1]]:
                pt = Point()
                pt.x, pt.y, pt.z = x, y, 0.03
                m.points.append(pt)
            trail.markers.append(m)
        if trail.markers:
            self.trail_pub.publish(trail)

    def publish_status(self, text, rgb=(1.0, 1.0, 1.0)):
        """Floating text above the robot (frame base_footprint), shown in RViz."""
        m = Marker()
        m.header.frame_id = 'base_footprint'
        m.header.stamp = Time().to_msg()  # latest available transform
        m.ns, m.id, m.type, m.action = 'p2p_status', 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        m.pose.position.z = 0.62
        m.pose.orientation.w = 1.0
        m.scale.z = 0.2
        m.text = text
        _color(m, *rgb)
        self.status_pub.publish(MarkerArray(markers=[m]))

    def publish_goal_marker(self, x, y, yaw, label):
        stamp = self.get_clock().now().to_msg()
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = stamp
        arrow.ns, arrow.id, arrow.type, arrow.action = 'p2p_goal', 0, Marker.ARROW, Marker.ADD
        arrow.pose = make_pose(x, y, yaw).pose
        arrow.pose.position.z = 0.05
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.55, 0.09, 0.09
        _color(arrow, 0.85, 0.1, 0.65)
        disc = Marker()
        disc.header = arrow.header
        disc.ns, disc.id, disc.type, disc.action = 'p2p_goal', 1, Marker.CYLINDER, Marker.ADD
        disc.pose = make_pose(x, y, 0.0).pose
        disc.pose.position.z = 0.01
        disc.scale.x = disc.scale.y = 0.5
        disc.scale.z = 0.02
        _color(disc, 0.85, 0.1, 0.65, 0.35)
        text = Marker()
        text.header = arrow.header
        text.ns, text.id, text.type, text.action = 'p2p_goal', 2, Marker.TEXT_VIEW_FACING, Marker.ADD
        text.pose = make_pose(x, y + 0.45, 0.0).pose
        text.pose.position.z = 0.4
        text.scale.z = 0.24
        text.text = f'GOAL: {label}'
        _color(text, 0.75, 0.05, 0.55)
        self.marker_pub.publish(MarkerArray(markers=[arrow, disc, text]))

    # ----------------------------------------------------------- navigation
    def navigate(self, goal_x, goal_y, goal_yaw, label='goal', timeout_s=300.0,
                 status_period_s=2.0, on_tick=None):
        """Drive to (goal_x, goal_y, goal_yaw) in the map frame and measure the run.

        Returns a dict of metrics (see docs/04_EVALUATION.md for definitions).
        """
        self.wait_for_odom(5.0)
        gt_start = self.odom_pose
        est_start = self.estimated_pose()
        aligned = bool(gt_start and est_start and
                       math.hypot(gt_start[0] - est_start[0], gt_start[1] - est_start[1]) < 0.30)
        start = gt_start if aligned else (est_start or gt_start)

        goal = make_pose(goal_x, goal_y, goal_yaw, stamp=self.get_clock().now().to_msg())
        self.publish_goal_marker(goal_x, goal_y, goal_yaw, label)

        planned_length = None
        path = self.plan_path(goal)
        if path is not None and len(path.poses) > 1:
            planned_length = M.path_length([(p.pose.position.x, p.pose.position.y) for p in path.poses])

        self._track = [(gt_start[0], gt_start[1])] if gt_start else []
        rgb = TRAIL_COLORS[len(self._legs) % len(TRAIL_COLORS)]
        self._legs.append((rgb, list(self._track)))
        self._min_range = math.inf
        self._plan_length = planned_length
        self._plan_events = []
        self._recording = True
        wall_start = time.monotonic()
        t_start = self.now_s()
        name = label.replace('_', ' ').upper()
        self.publish_status(f'-> {name}', rgb)

        accepted = self.send_goal(goal)
        result_name = 'REJECTED'
        last_feedback = None
        timed_out = False
        if accepted:
            last_status = -1e9
            last_trail = 0.0
            while not self.isTaskComplete():
                elapsed = self.now_s() - t_start
                fb = self.getFeedback()
                if fb is not None:
                    last_feedback = fb
                remaining = fb.distance_remaining if fb is not None else float('nan')
                if not self.quiet and elapsed - last_status >= status_period_s:
                    last_status = elapsed
                    driven = M.path_length(self._track)
                    rec = fb.number_of_recoveries if fb is not None else 0
                    ui.say(f'      t={elapsed:6.1f} s | to go {remaining:5.2f} m | driven {driven:5.2f} m'
                           f' | recoveries {rec}', 'grey')
                if time.monotonic() - last_trail > 0.5:
                    self.publish_trajectory()
                    togo = f'{remaining:.1f} m to go' if remaining == remaining and remaining > 0.0 else 'arriving'
                    self.publish_status(f'-> {name}\n{togo} | {self.odom_speed:.2f} m/s | {elapsed:.0f} s', rgb)
                    last_trail = time.monotonic()
                if on_tick is not None:
                    on_tick(self, elapsed)
                if elapsed > timeout_s and not timed_out:
                    timed_out = True
                    ui.say(f'      timeout after {timeout_s:.0f} s - cancelling goal', 'red')
                    self.cancel()
            result_name = 'TIMEOUT' if timed_out else RESULT_NAMES.get(self.getResult(), 'UNKNOWN')

        # let the last odometry / TF messages arrive before measuring
        self.spin_for(0.3)
        self._recording = False
        self.publish_trajectory()
        nav_time = self.now_s() - t_start

        gt_end = self.odom_pose
        est_end = self.estimated_pose()
        final = gt_end if (aligned and gt_end) else (est_end or gt_end)
        pos_err = yaw_err = None
        if final is not None:
            pos_err = M.position_error(final[0], final[1], goal_x, goal_y)
            yaw_err = math.degrees(M.yaw_error(final[2], goal_yaw))
        loc_err = None
        if aligned and gt_end and est_end:
            loc_err = math.hypot(gt_end[0] - est_end[0], gt_end[1] - est_end[1])
        driven = M.path_length(self._track)
        straight = M.straight_line_distance(start, (goal_x, goal_y)) if start else None
        reference = planned_length if planned_length else straight
        min_range = None if math.isinf(self._min_range) else self._min_range
        if result_name == 'SUCCEEDED':
            err = f'{pos_err:.2f} m' if pos_err is not None else '-'
            self.publish_status(f'REACHED {name}\nerror {err} | {nav_time:.0f} s', (0.3, 1.0, 0.4))
        else:
            self.publish_status(f'{result_name}: {name}', (1.0, 0.3, 0.3))

        return {
            'label': label,
            'goal_x': goal_x,
            'goal_y': goal_y,
            'goal_yaw_deg': math.degrees(goal_yaw),
            'result': result_name,
            'success': result_name == 'SUCCEEDED',
            'navigation_time_s': nav_time,
            'wall_time_s': time.monotonic() - wall_start,
            'path_length_m': driven,
            'planned_path_length_m': planned_length,
            'straight_line_m': straight,
            'path_efficiency': M.path_efficiency(reference, driven),
            'average_speed_mps': driven / nav_time if nav_time > 0 else None,
            'start_x': start[0] if start else None,
            'start_y': start[1] if start else None,
            'final_x': final[0] if final else None,
            'final_y': final[1] if final else None,
            'final_yaw_deg': math.degrees(final[2]) if final else None,
            'final_position_error_m': pos_err,
            'final_yaw_error_deg': yaw_err,
            'localization_error_m': loc_err,
            'recoveries': int(last_feedback.number_of_recoveries) if last_feedback is not None else 0,
            'replans': len(self._plan_events),
            'min_obstacle_distance_m': min_range,
            'min_clearance_m': (min_range - self.robot_radius) if min_range is not None else None,
            'ground_truth': 'gazebo' if aligned else 'amcl_estimate',
            'trajectory': list(self._track),
        }


def print_leg_result(res, index=None, total=None):
    head = f'[{index}/{total}] ' if index is not None else ''
    if res['success']:
        ui.say(f'  {head}REACHED {res["label"]}: {ui.fmt(res["navigation_time_s"], 1)} s, '
               f'path {ui.fmt(res["path_length_m"])} m (planned {ui.fmt(res["planned_path_length_m"])} m), '
               f'error {ui.fmt(res["final_position_error_m"], 3)} m / {ui.fmt(res["final_yaw_error_deg"], 1)} deg',
               'green', 'bold')
    else:
        ui.say(f'  {head}{res["result"]}: {res["label"]} after {ui.fmt(res["navigation_time_s"], 1)} s '
               f'(still {ui.fmt(res["final_position_error_m"])} m away)', 'red', 'bold')
