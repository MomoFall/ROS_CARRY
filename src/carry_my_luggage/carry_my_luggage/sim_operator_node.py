"""
Node sim_operator (simulation only): walk the operator legs along a path in Gazebo.

The operator is played by two cylinders (model operator_legs), moved with the
service /gazebo/set_entity_state of the gazebo_ros_state world plugin: the
simulated LiDAR does not always see the animated actors of Gazebo.
"""

import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .core.path_walker import PathWalker


class SimOperatorNode(Node):
    """Wait start_delay seconds in front of the robot, then walk the path."""

    def __init__(self):
        super().__init__('sim_operator')
        flat = list(self._param('waypoints', [0.8, 0.0, 4.0, 0.0, 4.0, 3.0, 7.0, 3.0]))
        if len(flat) < 4 or len(flat) % 2:
            raise ValueError('waypoints doit contenir au moins deux points x, y')
        self.walker = PathWalker(list(zip(flat[0::2], flat[1::2])),
                                 self._param('speed', 0.15),
                                 self._param('pause_index', -1),
                                 self._param('pause_duration', 0.0))
        self.entity = self._param('entity_name', 'operator_legs')
        self.start_delay = self._param('start_delay', 10.0)
        service = self._param('set_state_service', '/gazebo/set_entity_state')
        rate = self._param('rate', 20.0)

        self.client = self.create_client(SetEntityState, service)
        self.t0 = None
        self.pending = None
        self.create_timer(1.0 / rate, self.on_timer)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_timer(self):
        now = self.now()
        if now <= 0.0:
            return  # /clock not received yet
        if not self.client.service_is_ready():
            self.get_logger().info('En attente du service set_entity_state de Gazebo',
                                   throttle_duration_sec=5.0)
            return
        if self.pending is not None:
            if not self.pending.done():
                return
            result = self.pending.result()
            if self.t0 is None and result is not None and result.success:
                # The legs model exists in Gazebo: the countdown starts now.
                self.t0 = now
                self.get_logger().info(
                    f'Opérateur simulé en place, départ dans {self.start_delay:.0f} s '
                    f'({self.walker.total_length:.1f} m à {self.walker.speed:.2f} m/s)')
        elapsed = -1.0 if self.t0 is None else now - self.t0 - self.start_delay
        x, y, yaw, speed = self.walker.pose_at(elapsed)

        state = EntityState()
        state.name = self.entity
        state.reference_frame = 'world'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        state.twist.linear.x = speed * math.cos(yaw)
        state.twist.linear.y = speed * math.sin(yaw)
        self.pending = self.client.call_async(SetEntityState.Request(state=state))


def main(args=None):
    rclpy.init(args=args)
    node = SimOperatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
