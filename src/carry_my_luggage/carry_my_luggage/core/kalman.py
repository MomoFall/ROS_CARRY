"""
Operator tracking: leg pairing, target lock and Kalman filter.

Requirements F1.2 (lock), F2.2 (Kalman filter), F2.3 (loss), F2.4, R1 and R2
(only a measurement close to the predicted position updates the target).
"""

from dataclasses import dataclass
import math

import numpy as np

from .geometry import polar


def midpoint(a, b):
    """Return the middle of two points."""
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def pair_legs(legs, max_pair_distance):
    """
    Group legs two by two into persons, starting with the closest pairs.

    Return (pairs, singles): pairs lists the index pairs (i, j), singles the
    indices of the legs left alone.
    """
    count = len(legs)
    close = []
    for i in range(count):
        for j in range(i + 1, count):
            distance = math.dist(legs[i], legs[j])
            if distance <= max_pair_distance:
                close.append((distance, i, j))
    close.sort()
    used = set()
    pairs = []
    for _, i, j in close:
        if i not in used and j not in used:
            used.update((i, j))
            pairs.append((i, j))
    singles = [k for k in range(count) if k not in used]
    return pairs, singles


def person_candidates(legs, max_pair_distance):
    """
    List every possible measurement of a person.

    Return a list of (position, is_pair): the midpoint of any two legs closer
    than max_pair_distance, then every leg alone.
    """
    candidates = []
    count = len(legs)
    for i in range(count):
        for j in range(i + 1, count):
            if math.dist(legs[i], legs[j]) <= max_pair_distance:
                candidates.append((midpoint(legs[i], legs[j]), True))
    candidates.extend((tuple(leg), False) for leg in legs)
    return candidates


def best_candidate(legs, prediction, gate, max_pair_distance, single_gate=0.0,
                   single_penalty=0.1):
    """
    Return the measurement closest to the predicted position, or None (F2.4).

    A pair of legs is considered within gate of the prediction, a single leg
    only within single_gate (0 refuses single legs), so that a leg-sized piece
    of furniture next to the operator cannot steal the target (R2). A single
    leg is also penalised so that a full pair of legs is preferred.
    """
    best, best_score = None, math.inf
    for position, is_pair in person_candidates(legs, max_pair_distance):
        limit = gate if is_pair else single_gate
        distance = math.dist(position, prediction)
        if limit <= 0.0 or distance > limit:
            continue
        score = distance if is_pair else distance + single_penalty
        if score < best_score:
            best, best_score = position, score
    return best


class ConstantVelocityKalman:
    """Kalman filter with a constant velocity model, state X = [x, y, vx, vy]."""

    def __init__(self, x, y, t, measurement_std=0.08, acceleration_std=0.5,
                 initial_velocity_std=0.5, max_speed=1.5):
        self.state = np.array([x, y, 0.0, 0.0], dtype=float)
        self.cov = np.diag([measurement_std ** 2] * 2 + [initial_velocity_std ** 2] * 2)
        self.t = t
        self.accel_var = acceleration_std ** 2
        self.meas_cov = np.eye(2) * measurement_std ** 2
        self.max_speed = max_speed

    @property
    def position(self):
        return (float(self.state[0]), float(self.state[1]))

    @property
    def velocity(self):
        return (float(self.state[2]), float(self.state[3]))

    def predict(self, t):
        """Propagate the state up to time t (white noise acceleration model)."""
        dt = t - self.t
        if dt <= 0.0:
            return
        transition = np.eye(4)
        transition[0, 2] = transition[1, 3] = dt
        noise = np.zeros((4, 4))
        noise[0, 0] = noise[1, 1] = dt ** 4 / 4.0
        noise[0, 2] = noise[2, 0] = noise[1, 3] = noise[3, 1] = dt ** 3 / 2.0
        noise[2, 2] = noise[3, 3] = dt ** 2
        self.state = transition @ self.state
        self.cov = transition @ self.cov @ transition.T + self.accel_var * noise
        self.t = t

    def update(self, measurement):
        """Correct the state with a measured position (x, y)."""
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        innovation = np.asarray(measurement, dtype=float) - h @ self.state
        s = h @ self.cov @ h.T + self.meas_cov
        gain = self.cov @ h.T @ np.linalg.inv(s)
        self.state = self.state + gain @ innovation
        self.cov = (np.eye(4) - gain @ h) @ self.cov
        speed = math.hypot(self.state[2], self.state[3])
        if speed > self.max_speed:
            self.state[2:] *= self.max_speed / speed

    def position_at(self, t, horizon=0.5):
        """Return the position extrapolated at time t, at most horizon seconds ahead."""
        dt = min(max(t - self.t, 0.0), horizon)
        return (float(self.state[0] + self.state[2] * dt),
                float(self.state[1] + self.state[3] * dt))

    def freeze(self):
        """Forget the velocity: the target is assumed to stay where it was last seen."""
        self.state[2:] = 0.0


@dataclass
class TrackerParams:
    """Tuning of the operator tracker (see config/params.yaml)."""

    pair_distance: float = 0.45
    acquire_min_distance: float = 0.4
    acquire_max_distance: float = 1.2
    acquire_half_angle: float = math.radians(30.0)
    acquire_confirm_scans: int = 3
    acquire_confirm_distance: float = 0.2
    association_gate: float = 0.5
    single_leg_gate: float = 0.3
    lost_timeout: float = 1.0
    reacquire_gate_growth: float = 0.5
    reacquire_gate_max: float = 1.2
    reacquire_pairs_only: bool = False
    single_leg_penalty: float = 0.1
    measurement_std: float = 0.08
    acceleration_std: float = 0.5
    max_target_speed: float = 1.5
    prediction_horizon: float = 0.5


class OperatorTracker:
    """Lock the operator in front of the robot, then track it scan after scan."""

    NONE = 'none'
    TRACKING = 'tracking'
    LOST = 'lost'

    def __init__(self, params=None):
        self.params = params or TrackerParams()
        self.reset()

    def reset(self):
        """Forget the target."""
        self.kf = None
        self.status = self.NONE
        self.last_seen = None
        self.last_measurement = None
        self._candidate = None
        self._candidate_count = 0

    def update(self, legs_odom, legs_local, t, acquire=False):
        """
        Process the legs detected in one scan and return the tracker status.

        legs_odom and legs_local hold the same leg centres, in the odom frame
        (used by the filter) and in the robot frame (used by the lock cone).
        acquire allows locking a new target when there is none (state ACQUIRE).
        """
        self.last_measurement = None
        if self.kf is None:
            if acquire:
                self._acquire(legs_odom, legs_local, t)
            return self.status
        p = self.params
        self.kf.predict(t)
        unseen = t - self.last_seen
        gate, single_gate = self.gate(unseen)
        measurement = best_candidate(legs_odom, self.kf.position, gate, p.pair_distance,
                                     single_gate, p.single_leg_penalty)
        if measurement is not None:
            self.kf.update(measurement)
            self.last_seen = t
            self.last_measurement = measurement
            self.status = self.TRACKING
        elif self.status == self.TRACKING and unseen > p.lost_timeout:
            self.status = self.LOST
            self.kf.freeze()
        return self.status

    def gate(self, unseen):
        """
        Return the association gates (pair of legs, single leg).

        The gates are fixed while the target is tracked; once it is lost the
        pair gate widens with the time spent without seeing it, to find the
        operator again (F2.3, F2.4).
        """
        p = self.params
        if unseen <= p.lost_timeout:
            return p.association_gate, p.single_leg_gate
        grown = p.association_gate + p.reacquire_gate_growth * (unseen - p.lost_timeout)
        gate = min(grown, max(p.reacquire_gate_max, p.association_gate))
        return gate, 0.0 if p.reacquire_pairs_only else p.single_leg_gate

    def is_tracking(self, t):
        """Tell whether the target has been seen less than lost_timeout ago."""
        return self.status == self.TRACKING and t - self.last_seen <= self.params.lost_timeout

    def target_position(self, t):
        """Return the predicted position of the target at time t, or None if not tracked."""
        if not self.is_tracking(t):
            return None
        return self.kf.position_at(t, self.params.prediction_horizon)

    def _acquire(self, legs_odom, legs_local, t):
        """Lock the closest person in the cone in front of the robot (F1.2)."""
        p = self.params
        pairs, _ = pair_legs(legs_local, p.pair_distance)
        best = None
        for i, j in pairs:
            distance, bearing = polar(midpoint(legs_local[i], legs_local[j]))
            in_cone = (p.acquire_min_distance <= distance <= p.acquire_max_distance
                       and abs(bearing) <= p.acquire_half_angle)
            if in_cone and (best is None or distance < best[0]):
                best = (distance, midpoint(legs_odom[i], legs_odom[j]))
        if best is None:
            self._candidate, self._candidate_count = None, 0
            return
        position = best[1]
        if (self._candidate is not None
                and math.dist(position, self._candidate) <= p.acquire_confirm_distance):
            self._candidate_count += 1
        else:
            self._candidate_count = 1
        self._candidate = position
        if self._candidate_count >= p.acquire_confirm_scans:
            self.kf = ConstantVelocityKalman(position[0], position[1], t, p.measurement_std,
                                             p.acceleration_std, max_speed=p.max_target_speed)
            self.status = self.TRACKING
            self.last_seen = t
            self.last_measurement = position
            self._candidate, self._candidate_count = None, 0
