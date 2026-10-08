"""Tests of the geometry helpers and of the simulated operator path."""

import math

from carry_my_luggage.core.geometry import (min_range_in_sector, normalize_angle, polar,
                                            to_local, to_world, yaw_from_quaternion)
from carry_my_luggage.core.path_walker import PathWalker
import pytest
from scan_utils import ANGLE_MIN, INCREMENT, make_scan, RANGE_MAX, RANGE_MIN


def test_normalize_angle():
    assert normalize_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert normalize_angle(-0.5) == pytest.approx(-0.5)


def test_yaw_from_quaternion():
    yaw = 1.2
    assert yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == \
        pytest.approx(yaw)


def test_local_and_world_are_inverse():
    pose = (1.0, 2.0, 0.7)
    point = (3.0, -1.0)
    back = to_world(to_local(point, pose), pose)
    assert back == pytest.approx(point)


def test_point_ahead_of_rotated_robot():
    local = to_local((0.0, 1.0), (0.0, 0.0, math.pi / 2))
    distance, bearing = polar(local)
    assert distance == pytest.approx(1.0)
    assert bearing == pytest.approx(0.0, abs=1e-9)


def front_range(circles):
    return min_range_in_sector(make_scan(circles), ANGLE_MIN, INCREMENT, RANGE_MIN,
                               RANGE_MAX, math.radians(30.0))


def test_min_range_in_front_sector():
    assert front_range([(0.14, 0.0, 0.01), (1.0, 0.1, 0.05)]) == pytest.approx(0.13, abs=0.01)
    assert front_range([(0.0, -0.3, 0.05)]) == math.inf  # on the side: ignored
    assert front_range([]) == math.inf


def test_objects_closer_than_range_min_are_invisible():
    # The LiDAR reports 0 under range_min: such points must not count as obstacles.
    assert front_range([(0.08, 0.0, 0.02)]) == math.inf


def test_path_walker_with_pause():
    walker = PathWalker([(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)], speed=0.5, pause_index=1,
                        pause_duration=3.0)
    assert walker.pose_at(-1.0)[:2] == (0.0, 0.0)
    x, y, yaw, speed = walker.pose_at(2.0)
    assert (x, y, speed) == pytest.approx((1.0, 0.0, 0.5))
    assert walker.pose_at(5.0)[3] == 0.0  # paused at the corner
    x, y, yaw, _ = walker.pose_at(9.0)  # 4 s to the corner + 3 s pause + 2 s
    assert (x, y, yaw) == pytest.approx((2.0, 1.0, math.pi / 2))
    assert walker.pose_at(100.0)[:2] == pytest.approx((2.0, 2.0))
