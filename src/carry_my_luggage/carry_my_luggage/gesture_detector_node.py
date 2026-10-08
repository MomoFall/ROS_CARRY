"""
Node gesture_detector: recognise the stop gesture (F4.1).

Subscribes to /scan, /target and /odom; publishes std_msgs/Empty on
/stop_gesture once the hand has been held in front of the LiDAR for 2 s, with
the operator close and the robot stopped.
"""

import math

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray

from . import markers
from .core.gesture import count_hand_points, GestureParams, StopGestureDetector


class GestureDetectorNode(Node):
    """Watch the front of the robot for the hand of the operator."""

    def __init__(self):
        super().__init__('gesture_detector')
        self.params = GestureParams(
            gesture_distance=self._param('gesture_distance', 0.25),
            half_angle=math.radians(self._param('gesture_half_angle_deg', 20.0)),
            min_points=self._param('gesture_min_points', 5),
            operator_max_distance=self._param('gesture_operator_distance', 0.6),
            hold_duration=self._param('stop_gesture_duration', 2.0),
            dropout_tolerance=self._param('gesture_dropout_tolerance', 0.3))
        self.stopped_linear = self._param('stopped_linear_speed', 0.02)
        self.stopped_angular = self._param('stopped_angular_speed', 0.05)
        self.input_timeout = self._param('input_timeout', 0.5)
        self.detector = StopGestureDetector(self.params)

        self.target = None
        self.target_time = None
        self.odom = None
        self.odom_time = None
        self.range_checked = False

        self.gesture_pub = self.create_publisher(Empty, 'stop_gesture', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'follow_markers', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PointStamped, 'target', self.on_target, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_target(self, msg):
        self.target = (msg.point.x, msg.point.y)
        self.target_time = self.now()

    def on_odom(self, msg):
        self.odom = msg
        self.odom_time = self.now()

    def fresh(self, stamp, now):
        return stamp is not None and now - stamp <= self.input_timeout

    def on_scan(self, scan):
        now = self.now()
        if not self.range_checked:
            self.range_checked = True
            if scan.range_min >= self.params.gesture_distance:
                self.get_logger().warn(
                    f'gesture_distance ({self.params.gesture_distance:.2f} m) <= portée '
                    f'minimale du LiDAR ({scan.range_min:.2f} m) : geste impossible')
        hand_points = count_hand_points(scan.ranges, scan.angle_min, scan.angle_increment,
                                        scan.range_min, self.params.gesture_distance,
                                        self.params.half_angle)
        operator_distance = None
        robot_stopped = False
        if self.fresh(self.odom_time, now):
            position = self.odom.pose.pose.position
            twist = self.odom.twist.twist
            robot_stopped = (abs(twist.linear.x) < self.stopped_linear
                             and abs(twist.angular.z) < self.stopped_angular)
            if self.fresh(self.target_time, now):
                operator_distance = math.hypot(self.target[0] - position.x,
                                               self.target[1] - position.y)
        if self.detector.update(now, hand_points, operator_distance, robot_stopped):
            self.get_logger().info("Geste d'arrêt reconnu")
            self.gesture_pub.publish(Empty())
        self.publish_markers(scan.header.frame_id, now)

    def publish_markers(self, frame, now):
        """Show the gesture zone; it turns red as the hand is held."""
        progress = self.detector.progress(now)
        color = markers.RED if progress > 0.0 else markers.YELLOW
        zone = markers.sector(self.params.gesture_distance, self.params.half_angle)
        array = MarkerArray()
        array.markers.append(markers.line_strip(frame, 'gesture', 0, zone, color,
                                                width=0.008, lifetime=0.4))
        if progress > 0.0:
            array.markers.append(markers.text(
                frame, 'gesture', 1, 0.0, 0.0, 0.5, f'geste {progress * 100:.0f} %',
                markers.RED, lifetime=0.4))
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = GestureDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
