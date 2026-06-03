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
Geometry helpers for nav2_controller_py.

It mirrors the subset of nav2_util::geometry_utils used by the controller
server and its plugins.
"""

import math

from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Path


def get_yaw(q: Quaternion) -> float:
    """Convert a geometry_msgs/Quaternion to a yaw angle (rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def orientation_around_z_axis(angle: float) -> Quaternion:
    """Get a geometry_msgs/Quaternion from a yaw angle."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(angle / 2.0)
    q.w = math.cos(angle / 2.0)
    return q


def shortest_angular_distance(a: float, b: float) -> float:
    """Return the shortest signed angular distance from angle a to angle b."""
    return normalize_angle(b - a)


def normalize_angle(angle: float) -> float:
    """Normalize an angle to the range [-pi, pi]."""
    result = math.fmod(angle + math.pi, 2.0 * math.pi)
    if result <= 0.0:
        return result + math.pi
    return result - math.pi


def euclidean_distance(pos1: Pose, pos2: Pose, is_3d: bool = False) -> float:
    """Return the L2 distance between 2 geometry_msgs/Pose positions."""
    dx = pos1.position.x - pos2.position.x
    dy = pos1.position.y - pos2.position.y
    if is_3d:
        dz = pos1.position.z - pos2.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)
    return math.hypot(dx, dy)


def calculate_path_length(path: Path, start_index: int = 0) -> float:
    """
    Calculate the length of the provided path, starting at the provided index.

    Parameters
    ----------
    path : Path
        Path containing the poses that are planned.
    start_index : int
        Optional starting index for the calculation of path length.

    Returns
    -------
    float
        Path length.

    """
    poses = path.poses
    if start_index + 1 >= len(poses):
        return 0.0
    path_length = 0.0
    for idx in range(start_index, len(poses) - 1):
        path_length += euclidean_distance(poses[idx].pose, poses[idx + 1].pose)
    return path_length


def min_by(values, get_compare_val):
    """
    Find the index of the element with the minimum calculated value.

    Parameters
    ----------
    values : sequence
        Sequence of elements to compare.
    get_compare_val : callable
        Callable returning a comparable value for an element.

    Returns
    -------
    int
        Index of the element with the minimum value, or ``len(values)`` if the
        sequence is empty (mirroring the C++ ``end`` iterator semantics).

    """
    n = len(values)
    if n == 0:
        return 0
    lowest = get_compare_val(values[0])
    lowest_idx = 0
    for i in range(1, n):
        comp = get_compare_val(values[i])
        if comp <= lowest:
            lowest = comp
            lowest_idx = i
    return lowest_idx


def first_after_integrated_distance(values, begin: int, end: int, compare_val: float) -> int:
    """
    Find the first index whose integrated distance exceeds ``compare_val``.

    Parameters
    ----------
    values : sequence of PoseStamped
        Poses to integrate the distance over.
    begin : int
        Index to start integrating from.
    end : int
        Index to stop integrating at (exclusive).
    compare_val : float
        Distance threshold.

    Returns
    -------
    int
        The first index after the integrated distance threshold, or ``end``.

    """
    if begin >= end:
        return end
    dist = 0.0
    for it in range(begin, end - 1):
        dist += euclidean_distance(values[it].pose, values[it + 1].pose)
        if dist > compare_val:
            return it + 1
    return end


def distance_to_path_segment(point, start: Pose, end: Pose) -> float:
    """Return the distance from a point to a path segment [start, end]."""
    ax, ay = start.position.x, start.position.y
    bx, by = end.position.x, end.position.y
    px, py = point.x, point.y

    dx_seg = bx - ax
    dy_seg = by - ay
    seg_len_sq = dx_seg * dx_seg + dy_seg * dy_seg

    if seg_len_sq <= 1e-9:
        return math.hypot(px - ax, py - ay)

    dot = (px - ax) * dx_seg + (py - ay) * dy_seg
    t = max(0.0, min(1.0, dot / seg_len_sq))
    proj_x = ax + t * dx_seg
    proj_y = ay + t * dy_seg
    return math.hypot(px - proj_x, py - proj_y)


def cross_product_2d(point, start: Pose, end: Pose) -> float:
    """Compute the signed 2D cross product to determine the side of a segment."""
    ax, ay = start.position.x, start.position.y
    bx, by = end.position.x, end.position.y
    path_vec_x = bx - ax
    path_vec_y = by - ay
    robot_vec_x = point.x - ax
    robot_vec_y = point.y - ay
    return path_vec_x * robot_vec_y - path_vec_y * robot_vec_x
