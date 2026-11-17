"""Minimal Nav2 action client: send ONE goal pose and wait for the result.

This is the 'send_goal' node from the original project plan, corrected:
  * exits with a clear message if Nav2 is not running (the original spun forever),
  * uses action_msgs/GoalStatus constants instead of the magic number 4,
  * prints live feedback (distance remaining, recoveries),
  * cancels the goal after a timeout and returns a shell exit code.

    ros2 run tb3_point_to_point send_goal --ros-args \
        -p x:=-3.0 -p y:=1.6 -p yaw:=1.5708 -p use_sim_time:=true

For named rooms and performance metrics use the 'go_to' node instead.
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
}


class PointToPointNavigator(Node):

    def __init__(self):
        super().__init__('point_to_point_navigator')
        self.declare_parameter('x', -3.0)          # metres, map frame (default: Lab)
        self.declare_parameter('y', 1.6)
        self.declare_parameter('yaw', 1.5708)      # radians
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('server_timeout', 30.0)
        self.declare_parameter('goal_timeout', 300.0)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._last_print = None

    def run(self):
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)

        self.get_logger().info('Waiting for Nav2 navigation server...')
        if not self.client.wait_for_server(timeout_sec=float(self.get_parameter('server_timeout').value)):
            self.get_logger().error('Nav2 server not available! Start the simulation and navigation first '
                                    '(scripts/03_simulation.sh and scripts/07_navigation.sh).')
            return 2

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter('frame_id').value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)   # yaw -> quaternion
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Navigating to X={x:.2f}, Y={y:.2f}, yaw={math.degrees(yaw):.0f} deg')
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Navigation goal rejected (is the robot localised? is the goal free?)')
            return 3
        self.get_logger().info('Goal accepted!')

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future,
                                         timeout_sec=float(self.get_parameter('goal_timeout').value))
        if not result_future.done():
            self.get_logger().warn('Goal timed out - cancelling it')
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=5.0)
            return 4

        status = result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('SUCCESS: Destination reached!')
            return 0
        self.get_logger().warn(f'Navigation finished with status: {STATUS_NAMES.get(status, status)}')
        return 1

    def _on_feedback(self, msg):
        fb = msg.feedback
        now = self.get_clock().now()
        if self._last_print is None or (now - self._last_print).nanoseconds > 2e9:
            self._last_print = now
            self.get_logger().info(f'distance remaining {fb.distance_remaining:.2f} m, '
                                   f'recoveries {fb.number_of_recoveries}')


def main(args=None):
    rclpy.init(args=args)
    node = PointToPointNavigator()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
