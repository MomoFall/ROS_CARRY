"""
Node sim_operator (simulation only): move the operator legs in Gazebo.

The operator is played by two cylinders (model operator_legs), moved with the
service /gazebo/set_entity_state of the gazebo_ros_state world plugin: the
simulated LiDAR does not always see the animated actors of Gazebo.
Two modes (parameter mode):
- teleop: the operator walks according to /operator/cmd_vel (geometry_msgs/Twist),
  sent with the arrow keys by operator_teleop;
- path: the operator waits start_delay seconds, then walks along waypoints.
"""

import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .core.path_walker import PathWalker
from .core.teleop import TeleopWalker


class SimOperatorNode(Node):
    """Place the operator legs in Gazebo at every cycle."""

    def __init__(self):
        super().__init__('sim_operator')
        self.mode = self._param('mode', 'teleop')
        if self.mode not in ('teleop', 'path'):
            raise ValueError(f"mode doit valoir 'teleop' ou 'path', pas '{self.mode}'")
        flat = list(self._param('waypoints', [0.8, 0.0, 4.0, 0.0, 4.0, 3.0, 7.0, 3.0]))
        if len(flat) < 4 or len(flat) % 2:
            raise ValueError('waypoints doit contenir au moins deux points x, y')
        self.walker = PathWalker(list(zip(flat[0::2], flat[1::2])),
                                 self._param('speed', 0.15),
                                 self._param('pause_index', -1),
                                 self._param('pause_duration', 0.0))
        # In teleop mode the operator starts on the first waypoint, facing the second.
        self.teleop = TeleopWalker(flat[0], flat[1],
                                   math.atan2(flat[3] - flat[1], flat[2] - flat[0]),
                                   self._param('teleop_timeout', 0.7))
        self.entity = self._param('entity_name', 'operator_legs')
        self.start_delay = self._param('start_delay', 10.0)
        service = self._param('set_state_service', '/gazebo/set_entity_state')
        rate = self._param('rate', 20.0)

        self.client = self.create_client(SetEntityState, service)
        if self.mode == 'teleop':
            self.create_subscription(Twist, 'operator/cmd_vel', self.on_cmd, 10)
        self.t0 = None
        self.pending = None
        self.create_timer(1.0 / rate, self.on_timer)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.teleop.command(msg.linear.x, msg.angular.z, self.now())

    def announce(self):
        if self.mode == 'teleop':
            self.get_logger().info(
                'Opérateur simulé en place. Pour le piloter au clavier, dans un autre '
                'terminal : ros2 run carry_my_luggage operator_teleop')
        else:
            self.get_logger().info(
                f'Opérateur simulé en place, départ dans {self.start_delay:.0f} s '
                f'({self.walker.total_length:.1f} m à {self.walker.speed:.2f} m/s)')

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
                self.t0 = now  # the legs model exists in Gazebo
                self.announce()

        if self.mode == 'teleop':
            x, y, yaw, speed, turn = self.teleop.pose_at(now)
        else:
            elapsed = -1.0 if self.t0 is None else now - self.t0 - self.start_delay
            x, y, yaw, speed = self.walker.pose_at(elapsed)
            turn = 0.0

        state = EntityState()
        state.name = self.entity
        state.reference_frame = 'world'
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        state.twist.linear.x = float(speed * math.cos(yaw))
        state.twist.linear.y = float(speed * math.sin(yaw))
        state.twist.angular.z = float(turn)
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
