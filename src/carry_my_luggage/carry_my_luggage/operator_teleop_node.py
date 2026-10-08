"""
Node operator_teleop (simulation only): walk the simulated operator with the arrow keys.

Run it in its own terminal, like turtle_teleop_key:
ros2 run carry_my_luggage operator_teleop
It publishes geometry_msgs/Twist on /operator/cmd_vel, read by sim_operator
(sim.launch.py with operator:=teleop, the default).
"""

import os
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

from .core.teleop import DOWN, LEFT, RIGHT, split_keys, UP

HELP = """
Opérateur simulé : pilotez-le avec les flèches (comme turtle_teleop_key).
  ↑ avancer    ↓ reculer    ← tourner à gauche    → tourner à droite
  Maintenez la touche pour continuer à marcher, relâchez pour vous arrêter.
  espace : arrêt    + / - : vitesse de marche    Ctrl+C : quitter
"""


class OperatorTeleopNode(Node):
    """Turn key presses into velocity commands for the simulated operator."""

    def __init__(self):
        super().__init__('operator_teleop')
        self.walk_speed = self.declare_parameter('walk_speed', 0.15).value
        self.turn_speed = self.declare_parameter('turn_speed', 1.0).value
        self.cmd_pub = self.create_publisher(Twist, 'operator/cmd_vel', 10)

    def send(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def on_key(self, key):
        """Publish the command matching a key, or change the walking speed."""
        if key == UP:
            self.send(self.walk_speed, 0.0)
        elif key == DOWN:
            self.send(-self.walk_speed, 0.0)
        elif key == LEFT:
            self.send(0.0, self.turn_speed)
        elif key == RIGHT:
            self.send(0.0, -self.turn_speed)
        elif key == ' ':
            self.send(0.0, 0.0)
        elif key in ('+', '='):
            self.walk_speed = min(0.5, round(self.walk_speed + 0.05, 2))
        elif key in ('-', '_'):
            self.walk_speed = max(0.05, round(self.walk_speed - 0.05, 2))


def main(args=None):
    if not sys.stdin.isatty():
        print('operator_teleop doit être lancé dans un terminal : '
              'ros2 run carry_my_luggage operator_teleop')
        return
    rclpy.init(args=args)
    node = OperatorTeleopNode()
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(HELP)
    shown_speed = None
    try:
        tty.setraw(fd)  # keys arrive at once, Ctrl+C included
        while rclpy.ok():
            if node.walk_speed != shown_speed:
                shown_speed = node.walk_speed
                sys.stdout.write(f'\rvitesse de marche : {shown_speed:.2f} m/s ')
                sys.stdout.flush()
            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                continue
            keys = split_keys(os.read(fd, 64).decode(errors='ignore'))
            if '\x03' in keys:
                break
            for key in keys:
                node.on_key(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        print()
        node.send(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
