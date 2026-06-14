# Copyright (c) 2026 Alberto J. Tudela Roldán
# Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Footprint helpers for nav2_costmap_2d_py.

It mirrors the nav2_costmap_2d footprint.hpp from the C++ implementation.

Footprints are represented as a list of ``(x, y)`` tuples (the robot-frame
polygon), matching the convention used throughout the Python port.
"""

import ast
import math
from typing import List, Tuple

from geometry_msgs.msg import Point, Point32, Polygon
from nav2_costmap_2d_py.core.costmap_math import distance, distance_to_line, sign0

Footprint = List[Tuple[float, float]]


def calculate_min_and_max_distances(footprint: Footprint) -> Tuple[float, float]:
    """
    Calculate the extreme distances for the footprint.

    Parameters
    ----------
    footprint : list of tuple of float
        The footprint to examine, as ``(x, y)`` points.

    Returns
    -------
    (min_dist, max_dist)
        The minimum and maximum distances from the robot centre to the
        footprint edges/vertices. For a footprint with two or fewer points,
        returns ``(inf, 0.0)``.

    """
    min_dist = float('inf')
    max_dist = 0.0

    if len(footprint) <= 2:
        return min_dist, max_dist

    n = len(footprint)
    for i in range(n - 1):
        vertex_dist = distance(0.0, 0.0, footprint[i][0], footprint[i][1])
        edge_dist = distance_to_line(
            0.0, 0.0, footprint[i][0], footprint[i][1],
            footprint[i + 1][0], footprint[i + 1][1])
        min_dist = min(min_dist, min(vertex_dist, edge_dist))
        max_dist = max(max_dist, max(vertex_dist, edge_dist))

    vertex_dist = distance(0.0, 0.0, footprint[-1][0], footprint[-1][1])
    edge_dist = distance_to_line(
        0.0, 0.0, footprint[-1][0], footprint[-1][1],
        footprint[0][0], footprint[0][1])
    min_dist = min(min_dist, min(vertex_dist, edge_dist))
    max_dist = max(max_dist, max(vertex_dist, edge_dist))

    return min_dist, max_dist


def to_point(pt: Point32) -> Point:
    """
    Convert a ``Point32`` to a ``Point``.

    Parameters
    ----------
    pt : geometry_msgs.msg.Point32
        The point to convert.

    Returns
    -------
    geometry_msgs.msg.Point
        The converted point.

    """
    point = Point()
    point.x = float(pt.x)
    point.y = float(pt.y)
    point.z = float(pt.z)
    return point


def to_point32(pt: Point) -> Point32:
    """
    Convert a ``Point`` to a ``Point32``.

    Parameters
    ----------
    pt : geometry_msgs.msg.Point
        The point to convert.

    Returns
    -------
    geometry_msgs.msg.Point32
        The converted point.

    """
    point32 = Point32()
    point32.x = float(pt.x)
    point32.y = float(pt.y)
    point32.z = float(pt.z)
    return point32


def to_polygon(pts: Footprint) -> Polygon:
    """
    Convert a list of ``(x, y)`` points to a ``Polygon`` message.

    Parameters
    ----------
    pts : list of tuple of float
        The footprint points.

    Returns
    -------
    geometry_msgs.msg.Polygon
        The polygon message.

    """
    polygon = Polygon()
    for px, py in pts:
        p = Point32()
        p.x = float(px)
        p.y = float(py)
        p.z = 0.0
        polygon.points.append(p)
    return polygon


def to_point_vector(polygon: Polygon) -> Footprint:
    """
    Convert a ``Polygon`` message to a list of ``(x, y)`` points.

    Parameters
    ----------
    polygon : geometry_msgs.msg.Polygon
        The polygon message.

    Returns
    -------
    list of tuple of float
        The footprint points.

    """
    return [(float(p.x), float(p.y)) for p in polygon.points]


def transform_footprint(
    x: float, y: float, theta: float, footprint_spec: Footprint
) -> Footprint:
    """
    Build the oriented footprint of the robot at a given pose.

    Parameters
    ----------
    x, y : float
        The position of the robot.
    theta : float
        The orientation of the robot, in radians.
    footprint_spec : list of tuple of float
        The basic shape of the footprint, in the robot frame.

    Returns
    -------
    list of tuple of float
        The oriented footprint in the world frame.

    """
    cos_th = math.cos(theta)
    sin_th = math.sin(theta)
    return [
        (x + (px * cos_th - py * sin_th), y + (px * sin_th + py * cos_th))
        for px, py in footprint_spec
    ]


def pad_footprint(footprint: Footprint, padding: float) -> Footprint:
    """
    Add the specified amount of padding to the footprint.

    Each coordinate is pushed outward by ``padding`` in the direction of its
    sign (axis-aligned).

    Parameters
    ----------
    footprint : list of tuple of float
        The footprint to pad.
    padding : float
        The amount of padding to add, in metres.

    Returns
    -------
    list of tuple of float
        The padded footprint.

    """
    return [
        (px + sign0(px) * padding, py + sign0(py) * padding)
        for px, py in footprint
    ]


def make_footprint_from_radius(radius: float) -> Footprint:
    """
    Create a circular footprint (16-point polygon) from a given radius.

    Parameters
    ----------
    radius : float
        The radius of the circular footprint, in metres.

    Returns
    -------
    list of tuple of float
        The footprint as a list of ``(x, y)`` points.

    """
    n = 16
    return [
        (math.cos(i * 2 * math.pi / n) * radius,
         math.sin(i * 2 * math.pi / n) * radius)
        for i in range(n)
    ]


def make_footprint_from_string(footprint_string: str) -> Footprint:
    """
    Make a footprint from a string of the form ``"[[1.0, 2.2], [3.3, 4.2], ...]"``.

    At least three points are required; on any parse error an empty list is
    returned.

    Parameters
    ----------
    footprint_string : str
        The footprint as a bracketed array of arrays of floats.

    Returns
    -------
    list of tuple of float
        The parsed footprint, or an empty list on error.

    """
    try:
        data = ast.literal_eval(footprint_string.strip())
    except (ValueError, SyntaxError):
        return []
    if not isinstance(data, (list, tuple)) or len(data) < 3:
        return []
    result: Footprint = []
    for pt in data:
        if isinstance(pt, (list, tuple)) and len(pt) == 2:
            result.append((float(pt[0]), float(pt[1])))
        else:
            return []
    return result
