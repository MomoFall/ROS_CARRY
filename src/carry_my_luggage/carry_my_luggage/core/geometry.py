"""2D geometry helpers shared by the nodes, without any ROS dependency."""

import math

import numpy as np


def normalize_angle(angle):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    """Return the yaw (rotation around z) of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def to_local(point, pose):
    """Express a world point (x, y) in the frame of a pose (x, y, yaw)."""
    dx = point[0] - pose[0]
    dy = point[1] - pose[1]
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    return (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)


def to_world(point, pose):
    """Express a point (x, y) given in the frame of a pose (x, y, yaw) in the world frame."""
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + cos_yaw * point[0] - sin_yaw * point[1],
            pose[1] + sin_yaw * point[0] + cos_yaw * point[1])


def points_to_world(points, pose):
    """Express an (N, 2) array of points given in the frame of pose in the world frame."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((pose[0] + cos_yaw * pts[:, 0] - sin_yaw * pts[:, 1],
                            pose[1] + sin_yaw * pts[:, 0] + cos_yaw * pts[:, 1]))


def points_to_local(points, pose):
    """Express an (N, 2) array of world points in the frame of pose (x, y, yaw)."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    dx, dy = pts[:, 0] - pose[0], pts[:, 1] - pose[1]
    cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
    return np.column_stack((cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy))


def polar(point):
    """Return the distance and the bearing of a point (x, y)."""
    return math.hypot(point[0], point[1]), math.atan2(point[1], point[0])


def scan_angles(count, angle_min, angle_increment):
    """Return the angle of every ray of a scan, wrapped to [-pi, pi]."""
    angles = angle_min + angle_increment * np.arange(count)
    return np.arctan2(np.sin(angles), np.cos(angles))


def valid_ranges(ranges, range_min, range_max):
    """
    Return the mask of the usable measurements of a scan.

    Drivers report a missing echo as 0, inf or nan: these values are discarded,
    as well as everything outside [range_min, range_max].
    """
    r = np.asarray(ranges, dtype=float)
    with np.errstate(invalid='ignore'):
        return np.isfinite(r) & (r >= max(range_min, 1e-3)) & (r <= range_max)


def sector_ranges(ranges, angle_min, angle_increment, range_min, range_max, half_angle):
    """Return the valid ranges whose ray lies within +-half_angle of the front (x axis)."""
    r = np.asarray(ranges, dtype=float)
    angles = scan_angles(r.size, angle_min, angle_increment)
    mask = valid_ranges(r, range_min, range_max) & (np.abs(angles) <= half_angle)
    return r[mask]


def min_range_in_sector(ranges, angle_min, angle_increment, range_min, range_max,
                        half_angle):
    """Return the smallest valid range in the front sector, or inf if there is none."""
    near = sector_ranges(ranges, angle_min, angle_increment, range_min, range_max, half_angle)
    return float(near.min()) if near.size else math.inf
