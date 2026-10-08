"""
Keyboard teleoperation of the simulated operator (simulation only).

Like turtlesim: a key press sends a velocity command that lasts a short time;
holding the key (keyboard auto-repeat) keeps the operator walking, releasing
it stops the operator.
"""

import math

from .geometry import normalize_angle

UP, DOWN, RIGHT, LEFT = '\x1b[A', '\x1b[B', '\x1b[C', '\x1b[D'


def split_keys(data):
    """
    Split the characters read from a raw terminal into keys.

    Arrow keys arrive as three-character escape sequences (ESC [ A to D).
    """
    keys = []
    i = 0
    while i < len(data):
        if data.startswith('\x1b[', i) and i + 2 < len(data):
            keys.append(data[i:i + 3])
            i += 3
        else:
            keys.append(data[i])
            i += 1
    return keys


class TeleopWalker:
    """Pose of the operator driven by velocity commands (unicycle model)."""

    def __init__(self, x, y, yaw, timeout=0.7, max_speed=0.5, max_turn=2.0):
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)
        self.timeout = timeout
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.v = 0.0
        self.w = 0.0
        self.command_time = None
        self.t = None

    def command(self, v, w, t):
        """Apply the speeds v in m/s and w in rad/s, received at time t."""
        self._advance(t)
        self.v = max(-self.max_speed, min(self.max_speed, v))
        self.w = max(-self.max_turn, min(self.max_turn, w))
        self.command_time = t

    def velocity(self, t):
        """Return the (v, w) command still in force at time t."""
        if self.command_time is None or t - self.command_time > self.timeout:
            return 0.0, 0.0
        return self.v, self.w

    def pose_at(self, t):
        """Move the operator up to time t and return (x, y, yaw, v, w)."""
        self._advance(t)
        v, w = self.velocity(t)
        return self.x, self.y, self.yaw, v, w

    def _advance(self, t):
        """Integrate the current command from the last update up to time t."""
        if self.t is not None and t > self.t and self.command_time is not None:
            start = max(self.t, self.command_time)
            end = min(t, self.command_time + self.timeout)
            if end > start:
                dt = end - start
                heading = self.yaw + self.w * dt / 2.0
                self.x += self.v * math.cos(heading) * dt
                self.y += self.v * math.sin(heading) * dt
                self.yaw = normalize_angle(self.yaw + self.w * dt)
        if self.t is None or t > self.t:
            self.t = t
