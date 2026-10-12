"""Helpers to build the RViz markers published on /follow_markers (E3)."""

import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

GREEN = (0.2, 0.85, 0.2, 0.9)
GREY = (0.65, 0.65, 0.65, 0.8)
RED = (0.95, 0.1, 0.1, 0.9)
BLUE = (0.25, 0.45, 1.0, 0.8)
ORANGE = (1.0, 0.6, 0.0, 0.9)
YELLOW = (1.0, 0.9, 0.1, 0.8)
WHITE = (1.0, 1.0, 1.0, 1.0)


def make_marker(frame_id, ns, marker_id, marker_type, rgba, stamp=None, lifetime=0.5):
    """Create a marker with the common fields filled in."""
    marker = Marker()
    marker.header.frame_id = frame_id
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns = ns
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color = ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]),
                             a=float(rgba[3]))
    seconds = int(lifetime)
    marker.lifetime = Duration(sec=seconds, nanosec=int((lifetime - seconds) * 1e9))
    return marker


def cylinder(frame_id, ns, marker_id, x, y, diameter, height, rgba, **kwargs):
    """Return a vertical cylinder standing on the ground at (x, y)."""
    marker = make_marker(frame_id, ns, marker_id, Marker.CYLINDER, rgba, **kwargs)
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(height) / 2.0
    marker.scale.x = marker.scale.y = float(diameter)
    marker.scale.z = float(height)
    return marker


def sphere(frame_id, ns, marker_id, x, y, z, diameter, rgba, **kwargs):
    """Return a sphere centred on (x, y, z)."""
    marker = make_marker(frame_id, ns, marker_id, Marker.SPHERE, rgba, **kwargs)
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(z)
    marker.scale.x = marker.scale.y = marker.scale.z = float(diameter)
    return marker


def text(frame_id, ns, marker_id, x, y, z, label, rgba, size=0.12, **kwargs):
    """Return a text that always faces the camera."""
    marker = make_marker(frame_id, ns, marker_id, Marker.TEXT_VIEW_FACING, rgba, **kwargs)
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = float(z)
    marker.scale.z = float(size)
    marker.text = label
    return marker


def points(frame_id, ns, marker_id, xy, rgba, size=0.02, z=0.0, **kwargs):
    """Return a cloud of square points."""
    marker = make_marker(frame_id, ns, marker_id, Marker.POINTS, rgba, **kwargs)
    marker.scale.x = marker.scale.y = float(size)
    marker.points = [Point(x=float(px), y=float(py), z=float(z)) for px, py in xy]
    return marker


def line_strip(frame_id, ns, marker_id, xy, rgba, width=0.01, z=0.0, **kwargs):
    """Return a polyline."""
    marker = make_marker(frame_id, ns, marker_id, Marker.LINE_STRIP, rgba, **kwargs)
    marker.scale.x = float(width)
    marker.points = [Point(x=float(px), y=float(py), z=float(z)) for px, py in xy]
    return marker


def circle(cx, cy, radius, segments=32):
    """Return the vertices of a closed circle."""
    return [(cx + radius * math.cos(2.0 * math.pi * k / segments),
             cy + radius * math.sin(2.0 * math.pi * k / segments))
            for k in range(segments + 1)]


def sector(radius, half_angle, segments=12):
    """Return the vertices of a closed circular sector centred on the x axis."""
    arc = [(radius * math.cos(a), radius * math.sin(a))
           for a in (-half_angle + 2.0 * half_angle * k / segments
                     for k in range(segments + 1))]
    return [(0.0, 0.0)] + arc + [(0.0, 0.0)]
