"""Tests of the obstacle avoidance (VFH)."""

import math

from carry_my_luggage.core.avoidance import avoid, AvoidanceParams, polar_histogram
import numpy as np
import pytest

P = AvoidanceParams()


def box(cx, cy, half=0.15, step=0.02):
    """Return the points of the outline of a square box seen by the LiDAR."""
    ticks = np.arange(-half, half + 1e-9, step)
    sides = [(cx - half, cy + t) for t in ticks] + [(cx + t, cy - half) for t in ticks]
    sides += [(cx + t, cy + half) for t in ticks]
    return np.array(sides)


def legs(x, y):
    return np.array([(x, y + 0.1), (x, y - 0.1), (x - 0.03, y + 0.1), (x - 0.03, y - 0.1)])


def sector_blocked(heading, centres, blocked):
    index = int(np.argmin(np.abs(np.arctan2(np.sin(centres - heading),
                                            np.cos(centres - heading)))))
    return bool(blocked[index])


def test_free_way_goes_straight_to_the_goal():
    heading, _, blocked = avoid(np.empty((0, 2)), (1.2, 0.3), P)
    assert heading == pytest.approx(math.atan2(0.3, 1.2))
    assert not blocked.any()


def test_operator_legs_are_not_obstacles():
    heading, _, _ = avoid(legs(1.0, 0.0), (1.0, 0.0), P)
    assert heading == pytest.approx(0.0)


def test_obstacles_beyond_the_goal_are_ignored():
    wall = np.array([(2.0, y) for y in np.arange(-1.0, 1.0, 0.05)])
    heading, _, _ = avoid(wall, (1.0, 0.0), P, ignore_goal=False)
    assert heading == pytest.approx(0.0)


def test_box_between_robot_and_operator_is_bypassed():
    points = np.vstack((box(0.6, 0.0), legs(1.4, 0.0)))
    heading, centres, blocked = avoid(points, (1.4, 0.0), P)
    assert heading is not None
    # Passing the box (half width 0.15) with the robot radius and the clearance.
    assert abs(heading) > math.atan2(0.15 + P.robot_radius, 0.45)
    assert not sector_blocked(heading, centres, blocked)


def test_side_choice_follows_the_previous_one():
    points = box(0.6, 0.0)
    left, _, _ = avoid(points, (1.4, 0.0), P, previous=0.8)
    right, _, _ = avoid(points, (1.4, 0.0), P, previous=-0.8)
    assert left > 0.0 > right


def test_box_on_the_side_does_not_deviate():
    heading, _, _ = avoid(box(0.6, 0.6), (1.4, 0.0), P)
    assert heading == pytest.approx(0.0)


def test_surrounded_robot_is_blocked():
    ring = np.array([(0.3 * math.cos(a), 0.3 * math.sin(a))
                     for a in np.linspace(-math.pi, math.pi, 120)])
    heading, _, blocked = avoid(ring, (1.5, 0.0), P, ignore_goal=False)
    assert heading is None
    assert blocked.all()


def test_wall_across_the_way_slides_along_it_within_max_detour():
    wall = np.array([(0.4, y) for y in np.arange(-3.0, 3.0, 0.02)])
    heading, _, _ = avoid(wall, (1.5, 0.0), P, ignore_goal=False)
    assert math.radians(70.0) < abs(heading) <= P.max_detour
    narrow = AvoidanceParams(max_detour=math.radians(60.0))
    heading, _, _ = avoid(wall, (1.5, 0.0), narrow, ignore_goal=False)
    assert heading is None


def test_very_close_obstacle_blocks_a_half_plane():
    centres, blocked = polar_histogram(np.array([[0.15, 0.0]]), P)
    assert blocked[np.abs(centres) < math.radians(85.0)].all()
    assert not blocked[np.abs(centres) > math.radians(100.0)].any()
