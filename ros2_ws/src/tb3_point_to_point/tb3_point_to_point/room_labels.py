"""Publish room names and named goal locations as RViz markers (/room_labels).

    ros2 run tb3_point_to_point room_labels --ros-args -p locations_file:=/path/locations.yaml
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from .locations import LocationBook, default_path


class RoomLabels(Node):

    def __init__(self):
        super().__init__('room_labels')
        self.declare_parameter('locations_file', default_path('config', 'locations.yaml'))
        self.declare_parameter('show_goals', True)
        self.declare_parameter('frame_id', 'map')
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, 'room_labels', qos)
        self.book = LocationBook(self.get_parameter('locations_file').value)
        self.markers = self._build()
        self.pub.publish(self.markers)
        self.create_timer(5.0, lambda: self.pub.publish(self.markers))

    def _marker(self, mid, mtype, ns):
        m = Marker()
        m.header.frame_id = self.get_parameter('frame_id').value
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _build(self):
        out = MarkerArray()
        mid = 0
        for name, (x0, y0, x1, y1) in self.book.rooms.items():
            m = self._marker(mid, Marker.TEXT_VIEW_FACING, 'rooms')
            m.pose.position.x = (x0 + x1) / 2.0
            m.pose.position.y = (y0 + y1) / 2.0 if name == 'Corridor' else y1 - 0.45
            m.pose.position.z = 1.2
            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.95, 1.0, 0.95
            m.text = name.upper()
            out.markers.append(m)
            mid += 1
        if self.get_parameter('show_goals').value:
            for name, loc in self.book.locations.items():
                disc = self._marker(mid, Marker.CYLINDER, 'goal_spots')
                disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = loc.x, loc.y, 0.005
                disc.scale.x = disc.scale.y = 0.30
                disc.scale.z = 0.01
                disc.color.r, disc.color.g, disc.color.b, disc.color.a = 0.55, 0.35, 0.95, 0.55
                out.markers.append(disc)
                mid += 1
                txt = self._marker(mid, Marker.TEXT_VIEW_FACING, 'goal_names')
                txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = loc.x, loc.y - 0.3, 0.25
                txt.scale.z = 0.2
                txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.80, 0.70, 1.0, 0.9
                txt.text = name
                out.markers.append(txt)
                mid += 1
        return out


def main(args=None):
    rclpy.init(args=args)
    node = RoomLabels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
