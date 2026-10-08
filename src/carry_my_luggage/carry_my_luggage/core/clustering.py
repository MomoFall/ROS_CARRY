"""
Leg detection in a 2D laser scan (requirement F2.1).

The scan is cut into clusters of neighbouring points; the clusters that have
the width of a leg are kept as leg candidates.
"""

from dataclasses import dataclass
import math

import numpy as np

from .geometry import scan_angles, valid_ranges


@dataclass
class Cluster:
    """Group of neighbouring scan points, in the sensor frame."""

    points: np.ndarray
    center: tuple
    width: float
    is_leg: bool


def scan_to_points(ranges, angle_min, angle_increment, range_min, range_max):
    """
    Convert the valid measurements of a scan to points.

    Return (points, rays): an (N, 2) array of points in the sensor frame, in the
    angular order of the rays, and the index of the ray of every point.
    """
    r = np.asarray(ranges, dtype=float)
    angles = scan_angles(r.size, angle_min, angle_increment)
    mask = valid_ranges(r, range_min, range_max)
    r, angles = r[mask], angles[mask]
    return np.column_stack((r * np.cos(angles), r * np.sin(angles))), np.nonzero(mask)[0]


def split_clusters(points, jump_distance, rays=None, max_missing_rays=2, ray_count=None):
    """
    Split points given in angular order into clusters.

    A new cluster starts as soon as two consecutive points are more than
    jump_distance apart, or are separated by more than max_missing_rays rays
    without echo: two legs seen against an out-of-range background would
    merge otherwise. rays holds the ray index of every point.
    With ray_count (360 degree scan), the last and the first clusters are
    merged when they touch: an object right in front of the robot straddles
    the start and the end of the scan.
    """
    if len(points) == 0:
        return []
    if rays is None:
        rays = np.arange(len(points))
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    breaks = (gaps > jump_distance) | (np.diff(rays) > max_missing_rays + 1)
    clusters = np.split(points, np.nonzero(breaks)[0] + 1)
    if ray_count is not None and len(clusters) > 1:
        seam_gap = rays[0] + ray_count - rays[-1]
        if (seam_gap <= max_missing_rays + 1
                and np.linalg.norm(points[-1] - points[0]) <= jump_distance):
            clusters[0] = np.vstack((clusters.pop(), clusters[0]))
    return clusters


def cluster_width(points):
    """Return the width of a cluster: the distance between its end points."""
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(points[-1] - points[0]))


def is_full_circle(count, angle_increment):
    """Tell whether a scan of count rays covers the full 360 degrees."""
    step = abs(angle_increment)
    return count * step >= 2.0 * math.pi - 1.5 * step


def detect_legs(ranges, angle_min, angle_increment, range_min, range_max,
                jump_distance=0.10, min_points=3, width_min=0.05, width_max=0.25,
                max_distance=3.0, max_missing_rays=2):
    """
    Cut a scan into clusters and flag the leg candidates (F2.1).

    A leg candidate has at least min_points points, a width between width_min
    and width_max and its centre closer than max_distance.
    Return the list of all the clusters.
    """
    points, rays = scan_to_points(ranges, angle_min, angle_increment, range_min, range_max)
    full_circle = is_full_circle(len(ranges), angle_increment)
    clusters = []
    for pts in split_clusters(points, jump_distance, rays, max_missing_rays,
                              len(ranges) if full_circle else None):
        center = pts.mean(axis=0)
        width = cluster_width(pts)
        is_leg = (len(pts) >= min_points and width_min <= width <= width_max
                  and math.hypot(center[0], center[1]) <= max_distance)
        clusters.append(Cluster(pts, (float(center[0]), float(center[1])), width, is_leg))
    return clusters


def leg_centers(clusters):
    """Return the centre of every leg candidate."""
    return [c.center for c in clusters if c.is_leg]
