"""
Control law of the follower (F3.1 to F3.3, F2.3 and S1).

Two saturated proportional controllers: the linear speed follows the distance
error, the angular speed follows the bearing of the operator.
"""

from dataclasses import dataclass


@dataclass
class ControllerParams:
    """Gains and limits of the control law (see config/params.yaml)."""

    follow_distance: float = 0.8
    min_follow_distance: float = 0.5
    k_v: float = 1.5
    k_w: float = 1.5
    max_linear_speed: float = 0.22
    max_angular_speed: float = 1.5
    turn_slowdown_angle: float = 1.05
    distance_deadband: float = 0.05
    angle_deadband: float = 0.05
    search_angular_speed: float = 0.8


def saturate(value, low, high):
    """Clamp value to [low, high]."""
    return max(low, min(high, value))


def follow_command(distance, angle, params):
    """
    Return (v, w) that keep the operator at follow_distance (F3.1, F3.2).

    distance and angle locate the operator in the robot frame. Closer than
    min_follow_distance the robot stops and never backs up (F3.3). The linear
    speed shrinks when the operator is off-axis, so that the robot turns first.
    """
    p = params
    if distance < p.min_follow_distance:
        return 0.0, 0.0
    error = distance - p.follow_distance
    v = 0.0 if abs(error) < p.distance_deadband else p.k_v * error
    v = saturate(v, 0.0, p.max_linear_speed)
    v *= max(0.0, 1.0 - abs(angle) / p.turn_slowdown_angle)
    w = 0.0 if abs(angle) < p.angle_deadband else p.k_w * angle
    w = saturate(w, -p.max_angular_speed, p.max_angular_speed)
    return v, w


def search_command(angle, params):
    """Return (v, w) that pivot in place toward the last known direction (F2.3)."""
    p = params
    if abs(angle) < p.angle_deadband:
        return 0.0, 0.0
    return 0.0, saturate(p.k_w * angle, -p.search_angular_speed, p.search_angular_speed)


def limit_acceleration(v_command, v_previous, max_accel, dt):
    """Limit the increase of the linear speed; braking is never limited."""
    if v_command > v_previous:
        return min(v_command, v_previous + max_accel * dt)
    return v_command
