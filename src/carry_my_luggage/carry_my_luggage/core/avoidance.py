"""
Obstacle avoidance with a vector field histogram (VFH).

The scan points lying between the robot and its goal (the operator, or the
start pose on the way back) are enlarged by the robot radius plus a clearance:
each one blocks the angular sectors in which the robot would hit it. The robot
then heads for the free sector closest to its goal, instead of stopping in
front of the obstacle. Everything is expressed in the robot frame.
"""

from dataclasses import dataclass
import math

import numpy as np

from .geometry import normalize_angle


@dataclass
class AvoidanceParams:
    """Tuning of the obstacle avoidance (see config/params.yaml)."""

    window: float = 1.0
    robot_radius: float = 0.11
    clearance: float = 0.08
    sector: float = math.radians(5.0)
    goal_clearance: float = 0.35
    max_detour: float = math.radians(100.0)
    hysteresis: float = 0.5


def wrap(angles):
    """Wrap an array of angles to [-pi, pi]."""
    return np.arctan2(np.sin(angles), np.cos(angles))


def obstacle_points(points, goal_distance, params, goal=None):
    """
    Keep the scan points that can block the way to the goal.

    Points farther than the window or than the goal are ignored, as well as the
    points closer than goal_clearance to the goal (the operator's own legs).
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    distances = np.hypot(points[:, 0], points[:, 1])
    keep = distances < min(params.window, goal_distance)
    if goal is not None:
        keep &= np.hypot(points[:, 0] - goal[0], points[:, 1] - goal[1]) > params.goal_clearance
    return points[keep]


def polar_histogram(obstacles, params):
    """
    Return (centres, blocked): the angle of every sector and which ones are blocked.

    An obstacle at distance d blocks the sectors within asin((radius + clearance) / d)
    of its direction: heading there, the robot would pass too close to it.
    """
    count = int(round(2.0 * math.pi / params.sector))
    width = 2.0 * math.pi / count
    centres = -math.pi + (np.arange(count) + 0.5) * width
    blocked = np.zeros(count, dtype=bool)
    if len(obstacles):
        distances = np.hypot(obstacles[:, 0], obstacles[:, 1])
        bearings = np.arctan2(obstacles[:, 1], obstacles[:, 0])
        inflated = params.robot_radius + params.clearance
        half_widths = np.arcsin(np.clip(inflated / np.maximum(distances, 1e-6), 0.0, 1.0))
        offsets = np.abs(wrap(centres[np.newaxis, :] - bearings[:, np.newaxis]))
        blocked = (offsets <= half_widths[:, np.newaxis] + width / 2.0).any(axis=0)
    return centres, blocked


def choose_heading(centres, blocked, goal_angle, params, previous=None):
    """
    Return the direction to drive toward, or None when every way is blocked.

    The goal direction itself when its sector is free; otherwise the free sector
    closest to it, with a preference for the previous choice so that the robot
    does not hesitate between the two sides of an obstacle. A detour of more than
    max_detour from the goal is refused.
    """
    width = centres[1] - centres[0]
    index = int((normalize_angle(goal_angle) + math.pi) // width) % len(centres)
    if not blocked[index]:
        return goal_angle
    deviation = np.abs(wrap(centres - goal_angle))
    cost = deviation.copy()
    if previous is not None:
        cost += params.hysteresis * np.abs(wrap(centres - previous))
    cost[blocked | (deviation > params.max_detour)] = np.inf
    best = int(np.argmin(cost))
    return None if np.isinf(cost[best]) else float(centres[best])


def avoid(points, goal, params, previous=None, ignore_goal=True):
    """
    Choose the heading toward a goal (x, y) in the robot frame, around the obstacles.

    Return (heading, centres, blocked); heading is None when the way is blocked.
    ignore_goal drops the scan points around the goal: the operator is not an
    obstacle to avoid, but the target to reach.
    """
    distance = math.hypot(goal[0], goal[1])
    angle = math.atan2(goal[1], goal[0])
    obstacles = obstacle_points(points, distance, params, goal if ignore_goal else None)
    centres, blocked = polar_histogram(obstacles, params)
    return choose_heading(centres, blocked, angle, params, previous), centres, blocked
