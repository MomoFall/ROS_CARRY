"""
Node leg_detector: find the leg candidates in the LiDAR scan (F2.1).

Subscribes to /scan and publishes the centre of every leg-sized cluster on
/legs (geometry_msgs/PoseArray, in the frame of the scan).
"""

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

from . import markers
from .core.clustering import detect_legs


class LegDetectorNode(Node):
    """Cut every scan into clusters and publish those with the width of a leg."""

    def __init__(self):
        super().__init__('leg_detector')
        self.jump_distance = self._param('cluster_jump_distance', 0.10)
        self.max_missing_rays = self._param('cluster_max_missing_rays', 2)
        self.min_points = self._param('cluster_min_points', 3)
        self.width_min = self._param('leg_width_min', 0.05)
        self.width_max = self._param('leg_width_max', 0.25)
        self.max_distance = self._param('leg_max_distance', 3.0)
        self.publish_markers = self._param('publish_markers', True)

        self.legs_pub = self.create_publisher(PoseArray, 'legs', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'follow_markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def on_scan(self, scan):
        clusters = detect_legs(scan.ranges, scan.angle_min, scan.angle_increment,
                               scan.range_min, scan.range_max, self.jump_distance,
                               self.min_points, self.width_min, self.width_max,
                               self.max_distance, self.max_missing_rays)
        msg = PoseArray()
        msg.header = scan.header
        for cluster in clusters:
            if cluster.is_leg:
                pose = Pose()
                pose.position.x, pose.position.y = cluster.center
                pose.orientation.w = 1.0
                msg.poses.append(pose)
        self.legs_pub.publish(msg)
        if self.publish_markers:
            self.markers_pub.publish(self.make_markers(scan.header, clusters))

    def make_markers(self, header, clusters):
        """Show the legs as green cylinders and the other clusters as grey points."""
        frame, stamp = header.frame_id, header.stamp
        array = MarkerArray()
        others = []
        leg_id = 0
        for cluster in clusters:
            if cluster.is_leg:
                array.markers.append(markers.cylinder(
                    frame, 'legs', leg_id, cluster.center[0], cluster.center[1],
                    max(cluster.width, 0.05), 0.3, markers.GREEN, stamp=stamp, lifetime=0.4))
                leg_id += 1
            else:
                others.extend((float(x), float(y)) for x, y in cluster.points)
        array.markers.append(markers.points(frame, 'clusters', 0, others, markers.GREY,
                                            size=0.03, stamp=stamp, lifetime=0.4))
        return array


def main(args=None):
    rclpy.init(args=args)
    node = LegDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
