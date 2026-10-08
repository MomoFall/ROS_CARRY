"""
Mission state machine and safety conditions.

Five states: IDLE, ACQUIRE, FOLLOW, SEARCH and STOPPED. From any state but
STOPPED, a danger (obstacle, missing scan, critical battery, emergency stop)
brings the robot back to IDLE at zero speed. STOPPED is the final state of a
mission (luggage delivered).
"""

from dataclasses import dataclass
from enum import Enum


class State(str, Enum):
    """States of the follower."""

    IDLE = 'IDLE'
    ACQUIRE = 'ACQUIRE'
    FOLLOW = 'FOLLOW'
    SEARCH = 'SEARCH'
    STOPPED = 'STOPPED'


class Event(str, Enum):
    """Events that make the state machine move."""

    START = 'START'
    TARGET_LOCKED = 'TARGET_LOCKED'
    TARGET_LOST = 'TARGET_LOST'
    TARGET_FOUND = 'TARGET_FOUND'
    SEARCH_TIMEOUT = 'SEARCH_TIMEOUT'
    STOP = 'STOP'
    DANGER = 'DANGER'
    RESET = 'RESET'


ACTIVE_STATES = (State.IDLE, State.ACQUIRE, State.FOLLOW, State.SEARCH)

TRANSITIONS = {
    (State.IDLE, Event.START): State.ACQUIRE,
    (State.ACQUIRE, Event.TARGET_LOCKED): State.FOLLOW,
    (State.FOLLOW, Event.TARGET_LOST): State.SEARCH,
    (State.SEARCH, Event.TARGET_FOUND): State.FOLLOW,
    (State.SEARCH, Event.SEARCH_TIMEOUT): State.ACQUIRE,
    (State.STOPPED, Event.RESET): State.IDLE,
}
TRANSITIONS.update({(state, Event.STOP): State.STOPPED for state in ACTIVE_STATES})
TRANSITIONS.update({(state, Event.DANGER): State.IDLE for state in ACTIVE_STATES})


@dataclass
class SafetyStatus:
    """Safety conditions checked at every cycle (S2 to S5)."""

    obstacle: bool = False
    scan_timeout: bool = False
    battery_critical: bool = False
    emergency_stop: bool = False

    @property
    def danger(self):
        return bool(self.reasons())

    def reasons(self):
        """Return the names of the active safety conditions."""
        names = ('obstacle', 'scan_timeout', 'battery_critical', 'emergency_stop')
        return [name for name in names if getattr(self, name)]


@dataclass
class Inputs:
    """What the follow manager knows at one cycle."""

    t: float
    safety: SafetyStatus
    target_visible: bool
    stop_requested: bool = False


class FollowStateMachine:
    """Pure Python state machine: events in, new state out."""

    def __init__(self, search_timeout=15.0, t=0.0):
        self.state = State.IDLE
        self.since = t
        self.search_timeout = search_timeout

    def handle(self, event, t):
        """
        Apply an event at time t and return the new state.

        An event that does not apply to the current state is ignored.
        """
        new_state = TRANSITIONS.get((self.state, event))
        if new_state is not None and new_state != self.state:
            self.state = new_state
            self.since = t
        return self.state

    def step(self, inputs):
        """
        Derive the events of one cycle from the inputs and apply them.

        Safety comes first. Return the list of (event, previous, new) transitions.
        """
        transitions = []

        def fire(event):
            previous = self.state
            if self.handle(event, inputs.t) != previous:
                transitions.append((event, previous, self.state))

        if inputs.stop_requested:
            fire(Event.STOP)
        if inputs.safety.danger:
            fire(Event.DANGER)
            return transitions
        if self.state == State.IDLE:
            fire(Event.START)
        if self.state == State.ACQUIRE and inputs.target_visible:
            fire(Event.TARGET_LOCKED)
        elif self.state == State.FOLLOW and not inputs.target_visible:
            fire(Event.TARGET_LOST)
        elif self.state == State.SEARCH:
            if inputs.target_visible:
                fire(Event.TARGET_FOUND)
            elif inputs.t - self.since > self.search_timeout:
                fire(Event.SEARCH_TIMEOUT)
        return transitions
