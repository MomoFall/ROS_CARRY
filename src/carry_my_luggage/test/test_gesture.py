"""Tests of the stop gesture (F4.1)."""

import math

from carry_my_luggage.core.gesture import count_hand_points, StopGestureDetector
from scan_utils import ANGLE_MIN, INCREMENT, make_scan, RANGE_MIN

HALF_ANGLE = math.radians(20.0)


def hand_points(circles):
    return count_hand_points(make_scan(circles), ANGLE_MIN, INCREMENT, RANGE_MIN, 0.25,
                             HALF_ANGLE)


def run(detector, duration, hand=10, operator=0.5, stopped=True, start=0.0, dt=0.2):
    """Feed constant observations; return the times at which the gesture fired."""
    fired = []
    t = start
    while t <= start + duration + 1e-9:
        if detector.update(t, hand, operator, stopped):
            fired.append(t)
        t += dt
    return fired


def test_hand_in_front_is_counted():
    assert hand_points([(0.2, 0.0, 0.05)]) >= 5


def test_hand_too_far_or_on_the_side_is_ignored():
    assert hand_points([(0.35, 0.0, 0.05)]) == 0
    assert hand_points([(0.0, 0.2, 0.05)]) == 0
    assert hand_points([(-0.2, 0.0, 0.05)]) == 0


def test_gesture_needs_two_seconds():
    detector = StopGestureDetector()
    assert run(detector, 1.8) == []
    detector = StopGestureDetector()
    fired = run(detector, 3.0)
    assert len(fired) == 1
    assert 2.0 <= fired[0] <= 2.2


def test_gesture_requires_all_conditions():
    assert run(StopGestureDetector(), 3.0, hand=2) == []
    assert run(StopGestureDetector(), 3.0, operator=0.8) == []
    assert run(StopGestureDetector(), 3.0, operator=None) == []
    assert run(StopGestureDetector(), 3.0, stopped=False) == []


def test_short_dropout_is_tolerated_but_release_resets():
    detector = StopGestureDetector()
    run(detector, 1.0)
    detector.update(1.2, 0, 0.5, True)  # hand lost for one scan
    assert run(detector, 1.0, start=1.4) != []

    detector = StopGestureDetector()
    run(detector, 1.6)
    run(detector, 1.0, hand=0, start=1.8)  # hand removed
    assert run(detector, 1.6, start=3.0) == []


def test_gesture_fires_again_after_release():
    detector = StopGestureDetector()
    assert len(run(detector, 3.0)) == 1
    run(detector, 1.0, hand=0, start=3.2)
    assert len(run(detector, 3.0, start=4.4)) == 1
