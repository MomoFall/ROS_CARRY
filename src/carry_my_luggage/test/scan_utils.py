"""Synthetic LaserScan ranges for the tests: ray casting on circles."""

import math

import numpy as np

ANGLE_MIN = 0.0
INCREMENT = 2.0 * math.pi / 360.0
RANGE_MIN = 0.12
RANGE_MAX = 3.5


def ray_circle(angle, cx, cy, radius):
    """Return the distance from the origin to a circle along a ray, or inf."""
    ux, uy = math.cos(angle), math.sin(angle)
    along = ux * cx + uy * cy
    disc = along * along - (cx * cx + cy * cy - radius * radius)
    if disc < 0.0 or along <= 0.0:
        return math.inf
    return along - math.sqrt(disc)


def make_scan(circles, count=360):
    """Return 360 ranges (LDS-01 layout: 0 to 2 pi) seeing the given (x, y, r) circles."""
    ranges = np.full(count, math.inf)
    for k in range(count):
        angle = ANGLE_MIN + k * INCREMENT
        for cx, cy, radius in circles:
            ranges[k] = min(ranges[k], ray_circle(angle, cx, cy, radius))
    ranges[ranges > RANGE_MAX] = math.inf
    ranges[ranges < RANGE_MIN] = 0.0
    return ranges


def legs_at(x, y, spacing=0.2, radius=0.06):
    """Return the two circles of a person standing at (x, y), legs side by side along y."""
    return [(x, y + spacing / 2.0, radius), (x, y - spacing / 2.0, radius)]
