"""
Node operator_tracker: lock the operator and track it with a Kalman filter.

Subscribes to /legs and /follow_state, uses tf to bring the legs into the odom
frame, and publishes the predicted position of the operator on /target
(geometry_msgs/PointStamped, odom frame) at 20 Hz while it is tracked.
Nothing is published on /target once the operator is lost.
"""

import math

from geometry_msgs.msg import PointStamped, PoseArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import MarkerArray

from . import markers
from .core.geometry import to_world, yaw_from_quaternion
from .core.kalman import midpoint, OperatorTracker, pair_legs, TrackerParams


class OperatorTrackerNode(Node):
    """Turn leg detections into one tracked target: the operator."""

    def __init__(self):
        super().__init__('operator_tracker')
        self.odom_frame = self._param('odom_frame', 'odom')
        rate = self._param('publish_rate', 20.0)
        self.auto_acquire = self._param('auto_acquire', False)
        self.tracker = OperatorTracker(TrackerParams(
            pair_distance=self._param('leg_pair_distance', 0.45),
            acquire_min_distance=self._param('acquire_min_distance', 0.4),
            acquire_max_distance=self._param('acquire_max_distance', 1.2),
            acquire_half_angle=math.radians(self._param('acquire_half_angle_deg', 30.0)),
            acquire_confirm_scans=self._param('acquire_confirm_scans', 3),
            acquire_confirm_distance=self._param('acquire_confirm_distance', 0.2),
            association_gate=self._param('association_gate', 0.5),
            single_leg_gate=self._param('single_leg_gate', 0.3),
            lost_timeout=self._param('lost_timeout', 1.0),
            reacquire_gate_growth=self._param('reacquire_gate_growth', 0.5),
            reacquire_gate_max=self._param('reacquire_gate_max', 1.2),
            reacquire_pairs_only=self._param('reacquire_pairs_only', False),
            single_leg_penalty=self._param('single_leg_penalty', 0.1),
            measurement_std=self._param('measurement_std', 0.08),
            acceleration_std=self._param('acceleration_std', 0.5),
            max_target_speed=self._param('max_target_speed', 1.5),
            prediction_horizon=self._param('prediction_horizon', 0.5)))
        self.manager_state = ''

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_pub = self.create_publisher(PointStamped, 'target', 10)
        self.markers_pub = self.create_publisher(MarkerArray, 'follow_markers', 10)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'follow_state', self.on_state, state_qos)
        self.create_subscription(PoseArray, 'legs', self.on_legs, 10)
        self.create_timer(1.0 / rate, self.on_timer)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.manager_state = msg.data

    def lookup(self, source_frame, stamp):
        """Return the transform odom <- source_frame at stamp, or the latest one."""
        when = Time.from_msg(stamp)
        if not self.tf_buffer.can_transform(self.odom_frame, source_frame, when):
            when = Time()
        return self.tf_buffer.lookup_transform(self.odom_frame, source_frame, when)

    def on_legs(self, msg):
        now = self.now()
        try:
            tf = self.lookup(msg.header.frame_id, msg.header.stamp)
        except TransformException as error:
            self.get_logger().warn(f'Pas de transformation vers {self.odom_frame} : {error}',
                                   throttle_duration_sec=2.0)
            return
        rotation = tf.transform.rotation
        pose = (tf.transform.translation.x, tf.transform.translation.y,
                yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w))
        legs_local = [(p.position.x, p.position.y) for p in msg.poses]
        legs_odom = [to_world(leg, pose) for leg in legs_local]

        # The follow manager drives the lock: a lost target is dropped when a new
        # acquisition starts, and the mission ends in STOPPED.
        status = self.tracker.status
        if self.manager_state == 'STOPPED' and status != OperatorTracker.NONE:
            self.tracker.reset()
        elif self.manager_state == 'ACQUIRE' and status == OperatorTracker.LOST:
            self.tracker.reset()
        acquire = self.auto_acquire or self.manager_state == 'ACQUIRE'
        new_status = self.tracker.update(legs_odom, legs_local, now, acquire)
        if new_status != status:
            self.get_logger().info(f'Cible : {status} -> {new_status}')

        self.publish_target(now)
        self.publish_markers(legs_odom, now)

    def on_timer(self):
        self.publish_target(self.now())

    def publish_target(self, now):
        position = self.tracker.target_position(now)
        if position is None:
            return
        msg = PointStamped()
        msg.header.frame_id = self.odom_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = float(position[0]), float(position[1])
        self.target_pub.publish(msg)

    def publish_markers(self, legs_odom, now):
        """Show the persons (pairs of legs), the target and its association gate."""
        frame = self.odom_frame
        array = MarkerArray()
        pairs, _ = pair_legs(legs_odom, self.tracker.params.pair_distance)
        for k, (i, j) in enumerate(pairs):
            x, y = midpoint(legs_odom[i], legs_odom[j])
            array.markers.append(markers.cylinder(frame, 'persons', k, x, y, 0.35, 0.05,
                                                  markers.BLUE))
        kf = self.tracker.kf
        if kf is not None:
            x, y = kf.position
            tracking = self.tracker.is_tracking(now)
            color = markers.RED if tracking else markers.ORANGE
            gate, _ = self.tracker.gate(now - self.tracker.last_seen)
            label = 'operateur' if tracking else 'operateur perdu'
            array.markers.append(markers.sphere(frame, 'target', 0, x, y, 0.5, 0.15, color))
            array.markers.append(markers.line_strip(frame, 'target', 1,
                                                    markers.circle(x, y, gate), color))
            array.markers.append(markers.text(frame, 'target', 2, x, y, 0.75, label,
                                              markers.WHITE))
        self.markers_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = OperatorTrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
