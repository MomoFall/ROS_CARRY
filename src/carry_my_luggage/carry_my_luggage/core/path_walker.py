"""Scripted walk of the simulated operator along a polyline (simulation only)."""

import math


class PathWalker:
    """Position along a polyline walked at constant speed, with an optional pause."""

    def __init__(self, waypoints, speed, pause_index=-1, pause_duration=0.0):
        if len(waypoints) < 2:
            raise ValueError('at least two waypoints are needed')
        self.waypoints = [(float(x), float(y)) for x, y in waypoints]
        self.speed = speed
        self.lengths = [0.0]
        for a, b in zip(self.waypoints, self.waypoints[1:]):
            self.lengths.append(self.lengths[-1] + math.dist(a, b))
        valid_pause = 0 <= pause_index < len(self.waypoints) and pause_duration > 0.0
        self.pause_at = self.lengths[pause_index] if valid_pause else None
        self.pause_duration = pause_duration

    @property
    def total_length(self):
        return self.lengths[-1]

    def distance_at(self, t):
        """Return the distance walked t seconds after the start."""
        if t <= 0.0:
            return 0.0
        walked = self.speed * t
        if self.pause_at is not None and walked > self.pause_at:
            walked = max(self.pause_at, self.speed * (t - self.pause_duration))
        return min(walked, self.total_length)

    def pose_at(self, t):
        """Return (x, y, yaw, speed) of the walker t seconds after the start."""
        walked = self.distance_at(t)
        moving = self.distance_at(t + 1e-3) > walked
        k = 0
        while k < len(self.lengths) - 2 and walked > self.lengths[k + 1]:
            k += 1
        (x0, y0), (x1, y1) = self.waypoints[k], self.waypoints[k + 1]
        segment = self.lengths[k + 1] - self.lengths[k]
        ratio = 0.0 if segment <= 0.0 else (walked - self.lengths[k]) / segment
        yaw = math.atan2(y1 - y0, x1 - x0)
        return (x0 + ratio * (x1 - x0), y0 + ratio * (y1 - y0), yaw,
                self.speed if moving else 0.0)
