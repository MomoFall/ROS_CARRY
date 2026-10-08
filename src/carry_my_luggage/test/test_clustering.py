"""Tests of the leg detection (F2.1)."""

import math

from carry_my_luggage.core.clustering import (cluster_width, detect_legs, leg_centers,
                                              split_clusters)
import numpy as np
import pytest
from scan_utils import ANGLE_MIN, INCREMENT, legs_at, make_scan, RANGE_MAX, RANGE_MIN


def legs_in(circles):
    ranges = make_scan(circles)
    return leg_centers(detect_legs(ranges, ANGLE_MIN, INCREMENT, RANGE_MIN, RANGE_MAX))


def test_split_clusters_on_jumps():
    points = np.array([[1.0, 0.0], [1.0, 0.02], [1.0, 0.04], [2.0, 1.0], [2.0, 1.02]])
    clusters = split_clusters(points, 0.10)
    assert [len(c) for c in clusters] == [3, 2]


def test_split_clusters_wrap_around_merges_first_and_last():
    points = np.array([[1.0, 0.01], [1.0, 0.03], [0.0, 2.0], [1.0, -0.03], [1.0, -0.01]])
    clusters = split_clusters(points, 0.10, ray_count=5)
    assert sorted(len(c) for c in clusters) == [1, 4]


def test_split_clusters_on_missing_rays():
    points = np.array([[1.0, 0.0], [1.0, 0.02], [1.0, 0.09], [1.0, 0.11]])
    assert len(split_clusters(points, 0.10, np.array([0, 1, 5, 6]))) == 2  # 3 rays missing
    assert len(split_clusters(points, 0.10, np.array([0, 1, 4, 5]))) == 1  # 2 rays missing


def test_cluster_width():
    assert cluster_width(np.array([[0.0, 0.0], [0.0, 0.1], [0.0, 0.2]])) == 0.2
    assert cluster_width(np.array([[1.0, 1.0]])) == 0.0


@pytest.mark.parametrize('distance', [0.6, 0.8, 1.0, 1.2, 2.0])
def test_two_legs_in_front_are_detected(distance):
    # Nothing behind the legs: the rays between them have no echo.
    legs = legs_in(legs_at(distance, 0.0))
    assert len(legs) == 2
    for x, y in legs:
        assert abs(x - distance) < 0.08
        assert abs(abs(y) - 0.1) < 0.03


def test_leg_straddling_the_scan_seam_is_one_cluster():
    # A leg right in front (angle 0) appears at both ends of a 0..2pi scan.
    legs = legs_in([(0.8, 0.0, 0.06)])
    assert len(legs) == 1
    assert abs(legs[0][1]) < 0.02


def test_thin_and_wide_objects_are_rejected():
    thin = [(1.0, 0.5, 0.01)]  # chair leg of 2 cm
    wide = [(1.0, -1.0, 0.3)]  # bin of 60 cm
    assert legs_in(thin + wide) == []


def test_far_legs_are_ignored():
    assert legs_in(legs_at(3.2, 0.0)) == []


def test_empty_scan():
    ranges = np.full(360, math.inf)
    assert detect_legs(ranges, ANGLE_MIN, INCREMENT, RANGE_MIN, RANGE_MAX) == []
