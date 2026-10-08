"""Tests of the keyboard teleoperation of the simulated operator."""

import math

from carry_my_luggage.core.teleop import DOWN, LEFT, RIGHT, split_keys, TeleopWalker, UP
import pytest


def test_split_keys_separates_arrows_and_characters():
    assert split_keys(UP + RIGHT + 'x ' + DOWN + LEFT) == [UP, RIGHT, 'x', ' ', DOWN, LEFT]
    assert split_keys('\x03') == ['\x03']
    assert split_keys('') == []


def walk(walker, v, w, duration, dt=0.05, start=0.0, repeat=0.03):
    """Hold a key: send the command every repeat seconds (keyboard auto-repeat)."""
    t, last = start, -math.inf
    while t <= start + duration + 1e-9:
        if t - last >= repeat - 1e-9:
            walker.command(v, w, t)
            last = t
        walker.pose_at(t)
        t += dt
    return t


def test_holding_up_walks_straight():
    walker = TeleopWalker(0.8, 0.0, 0.0)
    walker.pose_at(0.0)
    walk(walker, 0.2, 0.0, 2.0)
    x, y, yaw, v, _ = walker.pose_at(2.0)
    assert x == pytest.approx(1.2, abs=0.02)
    assert y == pytest.approx(0.0)
    assert v == 0.2


def test_operator_stops_after_release():
    walker = TeleopWalker(0.0, 0.0, 0.0, timeout=0.7)
    walker.pose_at(0.0)
    walker.command(0.2, 0.0, 0.0)
    assert walker.pose_at(0.5)[3] == 0.2
    x_stop = walker.pose_at(1.0)[0]
    assert walker.pose_at(1.0)[3] == 0.0
    assert walker.pose_at(5.0)[0] == pytest.approx(x_stop)


def test_turn_left_then_walk():
    walker = TeleopWalker(0.0, 0.0, 0.0, timeout=0.1)
    walker.pose_at(0.0)
    end = walk(walker, 0.0, 1.0, math.pi / 2 - 0.1)
    walker.pose_at(end + 1.0)  # key released: the turn goes on for the timeout, then stops
    assert walker.yaw == pytest.approx(math.pi / 2, abs=0.06)
    end = walk(walker, 0.2, 0.0, 1.0, start=end + 1.0)
    x, y, *_ = walker.pose_at(end + 1.0)
    assert x == pytest.approx(0.0, abs=0.03)
    assert y == pytest.approx(0.22, abs=0.03)  # 1 s of walk + 0.1 s of timeout


def test_commands_are_clamped():
    walker = TeleopWalker(0.0, 0.0, 0.0, max_speed=0.5, max_turn=2.0)
    walker.command(3.0, -9.0, 0.0)
    assert walker.velocity(0.1) == (0.5, -2.0)
