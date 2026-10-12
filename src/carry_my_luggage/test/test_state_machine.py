"""Tests of the state machine (F1 to F4, S2 to S5)."""

from carry_my_luggage.core.state_machine import (Event, FollowStateMachine, Inputs,
                                                 SafetyStatus, State)

SAFE = SafetyStatus()


def step(sm, t, target=False, stop=False, safety=SAFE):
    sm.step(Inputs(t, safety, target, stop))
    return sm.state


def test_starts_idle_then_acquires():
    sm = FollowStateMachine()
    assert sm.state == State.IDLE
    assert step(sm, 0.0) == State.ACQUIRE
    assert step(sm, 0.1, target=True) == State.FOLLOW


def test_nominal_mission():
    sm = FollowStateMachine()
    states = [step(sm, 0.0), step(sm, 0.5, target=True), step(sm, 1.0, target=True),
              step(sm, 2.0, target=True, stop=True), step(sm, 3.0, target=True)]
    assert states == [State.ACQUIRE, State.FOLLOW, State.FOLLOW, State.STOPPED,
                      State.STOPPED]


def test_search_then_found():
    sm = FollowStateMachine()
    step(sm, 0.0, target=True)
    assert step(sm, 1.0) == State.SEARCH
    assert step(sm, 2.0, target=True) == State.FOLLOW


def test_search_timeout_goes_back_to_acquire():
    sm = FollowStateMachine(search_timeout=15.0)
    step(sm, 0.0, target=True)
    step(sm, 1.0)
    assert step(sm, 15.9) == State.SEARCH
    assert step(sm, 16.1) == State.ACQUIRE


def test_danger_brings_back_to_idle_from_any_active_state():
    for reason in ('obstacle', 'scan_timeout', 'battery_critical', 'emergency_stop'):
        danger = SafetyStatus(**{reason: True})
        sm = FollowStateMachine()
        step(sm, 0.0, target=True)
        assert sm.state == State.FOLLOW
        assert step(sm, 0.1, target=True, safety=danger) == State.IDLE
        assert step(sm, 0.2, target=True, safety=danger) == State.IDLE


def test_resumes_after_obstacle_without_new_lock():
    sm = FollowStateMachine()
    step(sm, 0.0, target=True)
    step(sm, 0.1, target=True, safety=SafetyStatus(obstacle=True))
    assert step(sm, 0.2, target=True) == State.FOLLOW


def test_critical_battery_refuses_to_start():
    sm = FollowStateMachine()
    assert step(sm, 0.0, safety=SafetyStatus(battery_critical=True)) == State.IDLE


def test_stopped_is_final_until_reset():
    sm = FollowStateMachine()
    step(sm, 0.0, target=True)
    step(sm, 1.0, target=True, stop=True)
    assert step(sm, 2.0, target=True, safety=SafetyStatus(obstacle=True)) == State.STOPPED
    assert step(sm, 3.0, target=True) == State.STOPPED
    sm.handle(Event.RESET, 4.0)
    assert sm.state == State.IDLE
    assert step(sm, 4.1) == State.ACQUIRE


def test_step_reports_transitions():
    sm = FollowStateMachine()
    transitions = sm.step(Inputs(0.0, SAFE, True))
    assert transitions == [(Event.START, State.IDLE, State.ACQUIRE),
                           (Event.TARGET_LOCKED, State.ACQUIRE, State.FOLLOW)]


def delivered(return_delay=5.0, return_timeout=120.0):
    """Return a state machine that has just delivered the luggage at t = 10 s."""
    sm = FollowStateMachine(return_delay=return_delay, return_timeout=return_timeout)
    step(sm, 0.0, target=True)
    step(sm, 10.0, target=True, stop=True)
    assert sm.state == State.STOPPED
    return sm


def home_step(sm, t, reached, safety=SAFE, stop=False):
    sm.step(Inputs(t, safety, False, stop, reached))
    return sm.state


def test_way_back_after_delivery():
    sm = delivered()
    assert home_step(sm, 14.9, False) == State.STOPPED
    assert home_step(sm, 15.0, False) == State.RETURN
    assert home_step(sm, 30.0, False) == State.RETURN
    assert home_step(sm, 31.0, True) == State.HOME
    assert home_step(sm, 60.0, True) == State.HOME
    sm.handle(Event.RESET, 61.0)
    assert sm.state == State.IDLE


def test_danger_only_pauses_the_way_back():
    sm = delivered()
    home_step(sm, 15.0, False)
    assert home_step(sm, 16.0, False, safety=SafetyStatus(obstacle=True)) == State.RETURN
    assert home_step(sm, 17.0, False, safety=SafetyStatus(emergency_stop=True)) == State.RETURN
    assert home_step(sm, 18.0, True) == State.HOME


def test_stop_cancels_the_way_back_for_good():
    sm = delivered()
    home_step(sm, 15.0, False)
    assert home_step(sm, 16.0, False, stop=True) == State.STOPPED
    assert home_step(sm, 100.0, False) == State.STOPPED


def test_way_back_gives_up_after_timeout():
    sm = delivered(return_timeout=60.0)
    home_step(sm, 15.0, False)
    assert home_step(sm, 75.0, False) == State.RETURN
    assert home_step(sm, 75.1, False) == State.STOPPED
    assert home_step(sm, 200.0, False) == State.STOPPED


def test_no_way_back_when_disabled():
    sm = delivered(return_delay=None)
    assert home_step(sm, 1000.0, False) == State.STOPPED


def test_safety_reasons():
    assert SafetyStatus().danger is False
    status = SafetyStatus(obstacle=True, emergency_stop=True)
    assert status.danger is True
    assert status.reasons() == ['obstacle', 'emergency_stop']
