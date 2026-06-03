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
Path helpers for nav2_controller_py.

It mirrors the subset of nav2_util::path_utils and nav2_util::robot_utils used
by the controller server and its plugins.
"""

from dataclasses import dataclass
import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav2_controller_py.geometry_utils import (cross_product_2d, distance_to_path_segment,
                                               euclidean_distance, get_yaw,
                                               shortest_angular_distance)
from nav_msgs.msg import Path
from rclpy.duration import Duration
import tf2_geometry_msgs  # type: ignore[import-untyped]  # noqa: F401
import tf2_ros  # type: ignore[import-untyped]


@dataclass
class PathSearchResult:
    """Result of searching for the closest segment on a path."""

    distance: float
    closest_segment_index: int


def transform_pose_in_target_frame(
    pose: PoseStamped,
    tf_buffer: tf2_ros.Buffer,
    target_frame: str,
    transform_timeout: float = 0.1,
) -> Optional[PoseStamped]:
    """
    Transform a pose into a target frame.

    Parameters
    ----------
    pose : PoseStamped
        Pose to transform.
    tf_buffer : tf2_ros.Buffer
        TF buffer to use for the transformation.
    target_frame : str
        Frame to transform into.
    transform_timeout : float
        TF timeout to use for the transformation, in seconds.

    Returns
    -------
    PoseStamped or None
        The transformed pose, or ``None`` if the transform failed.

    """
    if pose.header.frame_id == target_frame:
        return pose
    try:
        return tf_buffer.transform(
            pose, target_frame, timeout=Duration(seconds=transform_timeout))
    except Exception:
        return None


def distance_from_path(
    path: Path,
    robot_pose,
    start_index: int = 0,
    search_window_length: float = float('inf'),
) -> PathSearchResult:
    """
    Find the minimum distance from the robot's pose to the closest path segment.

    Parameters
    ----------
    path : Path
        The path to search (sequence of poses).
    robot_pose : geometry_msgs.msg.Pose
        The robot's current pose.
    start_index : int
        The index in the path to start searching from.
    search_window_length : float
        The maximum length (in meters) to search along the path.

    Returns
    -------
    PathSearchResult
        Struct containing the minimum (signed) distance and the index of the
        closest segment.

    """
    result = PathSearchResult(distance=float('inf'), closest_segment_index=start_index)

    poses = path.poses
    if not poses:
        return result

    if len(poses) == 1:
        result.distance = euclidean_distance(robot_pose, poses[0].pose)
        result.closest_segment_index = 0
        return result

    if start_index >= len(poses):
        raise RuntimeError(
            f'Requested start index ({start_index}) is greater than or equal '
            f'to path size ({len(poses)}).'
        )

    distance_traversed = 0.0
    for i in range(start_index, len(poses) - 1):
        if distance_traversed > search_window_length:
            break

        current_distance = distance_to_path_segment(
            robot_pose.position, poses[i].pose, poses[i + 1].pose)

        if current_distance < result.distance:
            result.distance = current_distance
            result.closest_segment_index = i

        distance_traversed += euclidean_distance(poses[i].pose, poses[i + 1].pose)

    segment_start = poses[result.closest_segment_index]
    segment_end = poses[result.closest_segment_index + 1]

    # Obtain the signed direction of the cross track error
    cross_product = cross_product_2d(
        robot_pose.position, segment_start.pose, segment_end.pose)
    result.distance *= 1.0 if cross_product >= 0.0 else -1.0

    return result


def find_first_path_constraint(
    path: Path,
    enforce_path_inversion: bool,
    rotation_threshold: float,
) -> int:
    """
    Find the index of the first pose at which there is an inversion or rotation.

    Parameters
    ----------
    path : Path
        Path to check for inversion or rotation.
    enforce_path_inversion : bool
        Whether to enable the check for inversion.
    rotation_threshold : float
        Minimum rotation angle to consider an in-place rotation
        (0 to disable the rotation check).

    Returns
    -------
    int
        The first index after the inversion or in-place rotation found in the
        path, or ``len(path.poses)`` if none exist.

    """
    poses = path.poses
    n = len(poses)
    # At least 3 poses for a possible inversion
    if n < 3:
        return n

    check_rotation = abs(rotation_threshold) >= 1e-6
    rotation_idx = n
    inversion_idx = n
    prev_dx = 0.0
    prev_dy = 0.0

    for idx in range(0, n - 1):
        dx = poses[idx + 1].pose.position.x - poses[idx].pose.position.x
        dy = poses[idx + 1].pose.position.y - poses[idx].pose.position.y
        trans = math.hypot(dx, dy)

        # No smaller index can exist beyond this point, terminate early
        if rotation_idx <= idx + 1:
            break

        # Check inversion
        if enforce_path_inversion and trans > 1e-4:
            if idx >= 1:
                dot_product = prev_dx * dx + prev_dy * dy
                if dot_product < 0.0:
                    inversion_idx = idx + 1
                    break
            prev_dx = dx
            prev_dy = dy

        # Check in place rotation
        if check_rotation and trans < 1e-4 and rotation_idx == n:
            accumulated_rotation = 0.0
            end_idx = idx
            while end_idx < n - 1:
                current_yaw = get_yaw(poses[end_idx].pose.orientation)
                next_yaw = get_yaw(poses[end_idx + 1].pose.orientation)
                accumulated_rotation += abs(
                    shortest_angular_distance(current_yaw, next_yaw))
                if accumulated_rotation > rotation_threshold:
                    rotation_idx = end_idx + 1
                    break
                if end_idx + 2 < n:
                    ndx = poses[end_idx + 2].pose.position.x - \
                        poses[end_idx + 1].pose.position.x
                    ndy = poses[end_idx + 2].pose.position.y - \
                        poses[end_idx + 1].pose.position.y
                    if math.hypot(ndx, ndy) > 1e-4:
                        break
                else:
                    break
                end_idx += 1

    return min(rotation_idx, inversion_idx)


def remove_poses_after_first_constraint(
    path: Path,
    enforce_path_inversion: bool,
    rotation_threshold: float,
) -> int:
    """
    Find and remove poses after the first constraint in the path (in place).

    Parameters
    ----------
    path : Path
        Path to check for inversion or rotation (modified in place).
    enforce_path_inversion : bool
        Whether to enable the check for inversion.
    rotation_threshold : float
        Minimum rotation angle to consider an in-place rotation.

    Returns
    -------
    int
        The location of the inversion or rotation, or 0 if none exists.

    """
    first_after_constraint = find_first_path_constraint(
        path, enforce_path_inversion, rotation_threshold)
    if first_after_constraint == len(path.poses):
        return 0
    del path.poses[first_after_constraint:]
    return first_after_constraint
