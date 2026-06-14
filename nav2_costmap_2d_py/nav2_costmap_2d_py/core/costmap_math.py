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
Math helpers for nav2_costmap_2d_py.

It mirrors the nav2_costmap_2d costmap_math.hpp from the C++ implementation.
"""

import math


def sign(x: float) -> float:
    """
    Return -1 if x < 0, +1 otherwise.

    Parameters
    ----------
    x : float
        The value to take the sign of.

    Returns
    -------
    float
        ``-1.0`` if ``x < 0``, ``1.0`` otherwise.

    """
    return -1.0 if x < 0.0 else 1.0


def sign0(x: float) -> float:
    """
    Return the sign of x, with 0 mapped to 0.

    Parameters
    ----------
    x : float
        The value to take the sign of.

    Returns
    -------
    float
        ``-1.0`` if ``x < 0``, ``1.0`` if ``x > 0``, ``0.0`` if ``x == 0``.

    """
    return -1.0 if x < 0.0 else (1.0 if x > 0.0 else 0.0)


def distance(x0: float, y0: float, x1: float, y1: float) -> float:
    """
    Return the L2 (Euclidean) distance between two points.

    Parameters
    ----------
    x0, y0 : float
        The first point.
    x1, y1 : float
        The second point.

    Returns
    -------
    float
        The Euclidean distance between the two points.

    """
    return math.hypot(x1 - x0, y1 - y0)


def distance_to_line(
    p_x: float, p_y: float,
    x0: float, y0: float,
    x1: float, y1: float,
) -> float:
    """
    Return the distance from a point to a line segment.

    Parameters
    ----------
    p_x, p_y : float
        The point to measure from.
    x0, y0 : float
        The first endpoint of the segment.
    x1, y1 : float
        The second endpoint of the segment.

    Returns
    -------
    float
        The shortest distance from the point to the segment.

    """
    a = p_x - x0
    b = p_y - y0
    c = x1 - x0
    d = y1 - y0

    dot = a * c + b * d
    len_sq = c * c + d * d
    param = dot / len_sq

    if param < 0:
        xx = x0
        yy = y0
    elif param > 1:
        xx = x1
        yy = y1
    else:
        xx = x0 + param * c
        yy = y0 + param * d

    return distance(p_x, p_y, xx, yy)
