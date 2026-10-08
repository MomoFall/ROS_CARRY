"""Tests of the operator tracking (F1.2, F2.2 to F2.4, R1, R2)."""

import math

from carry_my_luggage.core.kalman import (best_candidate, ConstantVelocityKalman,
                                          OperatorTracker, pair_legs, TrackerParams)
import numpy as np


def person(x, y, spacing=0.2):
    return [(x, y + spacing / 2.0), (x, y - spacing / 2.0)]


def feed(tracker, legs, t, acquire=False):
    # Robot at the origin of odom, facing +x: both frames are the same.
    return tracker.update(legs, legs, t, acquire)


def locked_tracker(x=0.8, y=0.0):
    tracker = OperatorTracker()
    t = 0.0
    while tracker.status != OperatorTracker.TRACKING:
        feed(tracker, person(x, y), t, acquire=True)
        t += 0.2
        assert t < 3.0, 'lock must happen in less than 3 s (F1)'
    return tracker, t


def test_pair_legs_groups_close_legs():
    legs = person(1.0, 0.0) + [(3.0, 2.0)]
    pairs, singles = pair_legs(legs, 0.45)
    assert pairs == [(0, 1)]
    assert singles == [2]


def test_kalman_estimates_constant_velocity():
    rng = np.random.default_rng(0)
    kf = ConstantVelocityKalman(0.0, 0.0, 0.0)
    velocities = []
    for k in range(1, 50):
        t = 0.2 * k
        kf.predict(t)
        kf.update((0.2 * t + rng.normal(0, 0.03), 0.1 * t + rng.normal(0, 0.03)))
        velocities.append(kf.velocity)
    vx, vy = np.mean(velocities[-20:], axis=0)
    assert abs(vx - 0.2) < 0.03
    assert abs(vy - 0.1) < 0.03
    x, _ = kf.position_at(kf.t + 0.5)
    assert x > kf.position[0]


def test_lock_on_person_in_front():
    tracker, t = locked_tracker()
    position = tracker.target_position(t)
    assert math.dist(position, (0.8, 0.0)) < 0.05


def test_no_lock_outside_cone_or_range():
    for x, y in ((2.0, 0.0), (0.2, 0.0), (0.5, 0.8), (-0.8, 0.0)):
        tracker = OperatorTracker()
        for k in range(10):
            feed(tracker, person(x, y), 0.2 * k, acquire=True)
        assert tracker.status == OperatorTracker.NONE


def test_no_lock_without_acquire():
    tracker = OperatorTracker()
    for k in range(10):
        feed(tracker, person(0.8, 0.0), 0.2 * k, acquire=False)
    assert tracker.status == OperatorTracker.NONE


def test_lock_takes_closest_person():
    tracker = OperatorTracker()
    for k in range(5):
        feed(tracker, person(1.1, 0.3) + person(0.6, -0.1), 0.2 * k, acquire=True)
    assert math.dist(tracker.kf.position, (0.6, -0.1)) < 0.05


def test_follows_walking_operator_and_ignores_furniture():
    tracker, t = locked_tracker()
    chair_leg = [(1.6, 0.45)]  # single leg-sized object 0.45 m beside the path
    x = 0.8
    for _ in range(50):
        t += 0.2
        x += 0.15 * 0.2
        feed(tracker, person(x, 0.0) + chair_leg, t)
        position = tracker.target_position(t)
        assert position is not None
        assert abs(position[0] - x) < 0.15
        assert abs(position[1]) < 0.15


def test_target_lost_after_timeout_then_found_near_prediction():
    tracker, t = locked_tracker()
    for _ in range(4):  # 0.8 s unseen: still tracked
        t += 0.2
        feed(tracker, [], t)
    assert tracker.status == OperatorTracker.TRACKING
    for _ in range(2):  # 1.2 s unseen: lost (F2.3)
        t += 0.2
        feed(tracker, [], t)
    assert tracker.status == OperatorTracker.LOST
    assert tracker.target_position(t) is None
    t += 0.2
    feed(tracker, person(1.2, 0.3), t)  # back after 1.4 s, close to the last position (R1)
    assert tracker.status == OperatorTracker.TRACKING


def test_lost_target_is_not_replaced_by_far_person_or_furniture_leg():
    tracker, t = locked_tracker()
    table_leg = [(1.2, 0.0)]  # leg-sized, 0.4 m from the last position (R2)
    for _ in range(8):
        t += 0.2
        feed(tracker, person(3.0, 1.5) + table_leg, t)
    assert tracker.status == OperatorTracker.LOST


def test_best_candidate_prefers_pairs_and_respects_gates():
    legs = person(1.0, 0.0) + [(1.05, 0.05)]
    assert math.dist(best_candidate(legs, (1.0, 0.0), 0.5, 0.45, 0.3), (1.0, 0.0)) < 0.06
    assert best_candidate(legs, (3.0, 0.0), 0.5, 0.45, 0.3) is None
    single = [(1.0, 0.0)]
    assert best_candidate(single, (1.2, 0.0), 0.5, 0.45, 0.3) == (1.0, 0.0)
    assert best_candidate(single, (1.4, 0.0), 0.5, 0.45, 0.3) is None
    assert best_candidate(single, (1.0, 0.0), 0.5, 0.45, 0.0) is None


def test_gate_widens_while_lost():
    params = TrackerParams(association_gate=0.5, single_leg_gate=0.3, lost_timeout=1.0,
                           reacquire_gate_growth=0.5, reacquire_gate_max=1.2)
    tracker = OperatorTracker(params)
    assert tracker.gate(0.5) == (0.5, 0.3)
    assert tracker.gate(2.0) == (1.0, 0.3)
    assert tracker.gate(10.0) == (1.2, 0.3)
    params.reacquire_pairs_only = True
    assert tracker.gate(2.0) == (1.0, 0.0)


def test_lost_target_found_again_from_one_leg_close_to_last_position():
    tracker, t = locked_tracker()
    for _ in range(7):
        t += 0.2
        feed(tracker, [], t)
    assert tracker.status == OperatorTracker.LOST
    t += 0.2
    feed(tracker, [(0.9, 0.1)], t)  # the other leg is hidden (tight turn)
    assert tracker.status == OperatorTracker.TRACKING
