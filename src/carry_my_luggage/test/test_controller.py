"""Tests of the control law (F3.1 to F3.3, F2.3, S1)."""

import math

from carry_my_luggage.core.controller import (ControllerParams, follow_command,
                                              limit_acceleration, search_command)
import pytest

P = ControllerParams()


def test_stays_still_at_follow_distance():
    assert follow_command(0.8, 0.0, P) == (0.0, 0.0)


def test_moves_forward_when_operator_walks_away():
    v, w = follow_command(0.9, 0.0, P)
    assert v == pytest.approx(P.k_v * 0.1)
    assert w == 0.0


def test_speeds_are_saturated():
    v, w = follow_command(3.0, 1.0, P)
    assert 0.0 <= v <= P.max_linear_speed
    assert w == P.max_angular_speed
    v, _ = follow_command(3.0, 0.0, P)
    assert v == P.max_linear_speed
    _, w = follow_command(1.0, -2.0, P)
    assert w == -P.max_angular_speed


def test_never_backs_up_and_stops_when_too_close():
    assert follow_command(0.6, 0.0, P)[0] == 0.0
    assert follow_command(0.45, 0.5, P) == (0.0, 0.0)


def test_turns_toward_operator_and_slows_down():
    v_straight, _ = follow_command(1.2, 0.0, P)
    v_side, w_side = follow_command(1.2, math.radians(30.0), P)
    assert w_side > 0.0
    assert v_side < v_straight
    v_behind, _ = follow_command(1.2, math.radians(90.0), P)
    assert v_behind == 0.0


def test_search_pivots_in_place():
    v, w = search_command(math.radians(-90.0), P)
    assert v == 0.0
    assert w == -P.search_angular_speed
    assert search_command(0.0, P) == (0.0, 0.0)


def test_acceleration_limit_only_on_speed_up():
    assert limit_acceleration(0.22, 0.0, 0.5, 0.05) == pytest.approx(0.025)
    assert limit_acceleration(0.0, 0.22, 0.5, 0.05) == 0.0
