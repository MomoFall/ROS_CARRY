"""
Stop gesture: a hand held in front of the LiDAR for 2 s (F4.1).

The gesture is recognised when three conditions hold during hold_duration:
at least min_points scan points closer than gesture_distance in the front
sector, the operator closer than operator_max_distance, and the robot stopped.
"""

from dataclasses import dataclass
import math

from .geometry import sector_ranges


@dataclass
class GestureParams:
    """Thresholds of the stop gesture (see config/params.yaml)."""

    gesture_distance: float = 0.25
    half_angle: float = math.radians(20.0)
    min_points: int = 5
    operator_max_distance: float = 0.6
    hold_duration: float = 2.0
    dropout_tolerance: float = 0.3


def count_hand_points(ranges, angle_min, angle_increment, range_min, gesture_distance,
                      half_angle):
    """Count the scan points closer than gesture_distance in the front sector."""
    near = sector_ranges(ranges, angle_min, angle_increment, range_min, gesture_distance,
                         half_angle)
    return int(near.size)


class StopGestureDetector:
    """
    Recognise the stop gesture over time.

    A short interruption of the conditions (less than dropout_tolerance) does
    not restart the count, so that a slightly moving hand is still accepted.
    """

    def __init__(self, params=None):
        self.params = params or GestureParams()
        self.reset()

    def reset(self):
        """Restart the recognition."""
        self.start = None
        self.last_ok = None
        self.fired = False

    def conditions(self, hand_points, operator_distance, robot_stopped):
        """Tell whether the three conditions of the gesture hold right now."""
        p = self.params
        return (hand_points >= p.min_points
                and operator_distance is not None
                and operator_distance < p.operator_max_distance
                and robot_stopped)

    def update(self, t, hand_points, operator_distance, robot_stopped):
        """Feed one observation; return True once, when the gesture is recognised."""
        p = self.params
        if self.conditions(hand_points, operator_distance, robot_stopped):
            if self.start is None:
                self.start = t
            self.last_ok = t
            if not self.fired and t - self.start >= p.hold_duration:
                self.fired = True
                return True
            return False
        if self.last_ok is not None and t - self.last_ok <= p.dropout_tolerance:
            return False
        self.reset()
        return False

    def progress(self, t):
        """Return the fraction of the hold duration already done, in [0, 1]."""
        if self.start is None:
            return 0.0
        return min(1.0, (t - self.start) / self.params.hold_duration)
