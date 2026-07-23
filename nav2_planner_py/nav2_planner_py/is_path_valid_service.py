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
IsPathValidService for Nav2 Planner Server.

It mirrors the nav2_planner::IsPathValidService from the C++ implementation.
Hosts an IsPathValid service that determines whether a path is still valid
given the current costmap state.
"""

import math
import re
from typing import Any, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav2_costmap_2d_py import Costmap2DROS
from nav2_costmap_2d_py.core.cost_values import (FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE,
                                                 LETHAL_OBSTACLE, NO_INFORMATION)
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path
import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode


class IsPathValidService:
    """
    Service to determine if a path is still valid given the current costmap state.

    Creates and manages an IsPathValid ROS2 service, validates paths against
    the current costmap using footprint- or radius-based collision detection,
    and supports layer-specific costmap checking.
    """

    def __init__(
        self,
        node: LifecycleNode,
        costmap_ros: Costmap2DROS,
        costmap_update_timeout: Duration,
    ) -> None:
        """
        Initialize an IsPathValidService instance.

        Parameters
        ----------
        node : LifecycleNode
            Lifecycle node pointer used for service creation.
        costmap_ros : Costmap2DROS
            Costmap ROS wrapper used for collision checking and robot pose.
        costmap_update_timeout : Duration
            Timeout for waiting for costmap updates before path validation.

        """
        self._node = node
        self._costmap_ros = costmap_ros
        self._costmap_update_timeout = costmap_update_timeout
        self._logger = rclpy.logging.get_logger('is_path_valid_service')
        self._costmap = None
        self._service: Optional[Any] = None

    def initialize(self) -> None:
        """Initialize the IsPathValid service."""
        if self._node is None:
            raise RuntimeError('Failed to lock node in initialize')

        self._costmap = self._costmap_ros.get_costmap()  # type: ignore[assignment]

        self._service = self._node.create_service(
            IsPathValid, 'is_path_valid', self._callback
        )
        self._logger.info('IsPathValidService initialized')

    def reset(self) -> None:
        """Reset the service."""
        if self._service is not None:
            self._service.destroy()
            self._service = None
        self._costmap = None

    # ---------------------------------------------------------------
    # Private methods
    # ---------------------------------------------------------------

    def _get_costmap_to_check(self, layer_name: str):
        """
        Get the costmap to check based on the layer name.

        When the layer name is empty the full (master) costmap is returned.
        Otherwise the layered costmap plugins are searched by name and the first
        matching layer that exposes a costmap is returned.

        Parameters
        ----------
        layer_name : str
            Name of the layer to check, or empty string for the full costmap.

        Returns
        -------
        Costmap2D or None
            The costmap to check, or ``None`` if the requested layer is not found
            or does not provide a costmap.

        """
        if not layer_name:
            return self._costmap

        try:
            layers = self._costmap_ros.get_layered_costmap().get_plugins()  # type: ignore
            for layer in layers:
                if layer.get_name() == layer_name:
                    # Check if layer provides a costmap
                    if hasattr(layer, 'get_costmap'):
                        return layer.get_costmap()
                    elif hasattr(layer, 'costmap'):
                        return layer.costmap
        except Exception as ex:
            self._logger.error(f"Error getting layer '{layer_name}': {ex}")

        self._logger.error(
            f"Requested layer '{layer_name}' not found or does not provide a costmap."
        )
        return None

    def _get_footprint_to_use(
        self, footprint_string: str
    ) -> Tuple[List[Tuple[float, float]], bool]:
        """
        Get the footprint to use for collision checking.

        If a custom footprint string is provided it is parsed and radius-based checking
        is disabled. Otherwise the robot footprint configured in the costmap is used,
        or radius-based checking is applied when the costmap is configured that way.

        Parameters
        ----------
        footprint_string : str
            Custom footprint string in ``[[x1,y1],[x2,y2],...]`` format, or empty
            string to use the robot footprint from the costmap configuration.

        Returns
        -------
        Tuple[List[Tuple[float, float]], bool]
            A tuple ``(footprint, use_radius)`` where *footprint* is a list of
            ``(x, y)`` points relative to the robot centre, and *use_radius*
            indicates whether radius-based collision checking should be used
            instead of the footprint.

        """
        use_radius = self._costmap_ros.get_use_radius()

        if footprint_string:
            try:
                footprint = self._parse_footprint_string(footprint_string)
                use_radius = False
                return footprint, use_radius
            except ValueError:
                self._logger.error(
                    f"Invalid footprint string '{footprint_string}'. Cannot validate path."
                )
                return [], False

        if not use_radius:
            footprint = self._costmap_ros.get_robot_footprint()
            return footprint, use_radius

        return [], use_radius

    # TODO(ajtudela): Rename to _make_footprint_from_string and move to Footprint class
    # in nav2_costmap_2d_py, so it can be reused by other packages.
    def _parse_footprint_string(
        self, footprint_string: str
    ) -> List[Tuple[float, float]]:
        """
        Parse a footprint string into a list of (x, y) points.

        Extracts numeric coordinate pairs from the standard Nav2 footprint string format.

        Parameters
        ----------
        footprint_string : str
            Footprint specification in ``[[x1,y1],[x2,y2],...]`` format.

        Returns
        -------
        List[Tuple[float, float]]
            Ordered list of ``(x, y)`` coordinate pairs defining the footprint
            polygon relative to the robot centre.

        Raises
        ------
        ValueError
            If the footprint string is malformed or contains an odd number of
            coordinates.

        """
        # Simple parser for footprint format
        footprint_string = footprint_string.strip()
        if not footprint_string.startswith('[[') or not footprint_string.endswith(']]'):
            raise ValueError('Invalid footprint format')

        # Extract numbers
        numbers = re.findall(r'-?\d+\.?\d*', footprint_string)
        if len(numbers) % 2 != 0:
            raise ValueError('Odd number of coordinates')

        footprint = []
        for i in range(0, len(numbers), 2):
            footprint.append((float(numbers[i]), float(numbers[i + 1])))

        return footprint

    def _find_closest_point_index(
        self, current_pose: PoseStamped, path: Path
    ) -> int:
        """
        Find the closest point on the path to the current pose.

        The lethal check starts at this index to avoid checking poses that have
        already been passed and may have become occupied.

        Parameters
        ----------
        current_pose : PoseStamped
            The current robot pose in the global frame.
        path : Path
            The path to search for the closest point.

        Returns
        -------
        int
            Index of the closest pose in the path to the current robot position.

        """
        closest_point_index = 0
        closest_distance = float('inf')
        current_point = current_pose.pose.position

        for i, pose in enumerate(path.poses):
            path_point = pose.pose.position
            distance = math.sqrt(
                (current_point.x - path_point.x) ** 2 + (current_point.y - path_point.y) ** 2
            )

            if distance < closest_distance:
                closest_point_index = i
                closest_distance = distance

        return closest_point_index

    def _callback(
        self, request: IsPathValid.Request, response: IsPathValid.Response
    ) -> IsPathValid.Response:
        """
        Service callback to determine if the path is still valid.

        Checks each path pose from the closest point to the robot up to
        ``max_lookahead_distance`` for collisions using either radius- or
        footprint-based cost evaluation on the requested costmap layer.

        Parameters
        ----------
        request : IsPathValid.Request
            The service request containing the path and validation parameters
            (layer name, footprint, lookahead distance, cost thresholds, etc.).
        response : IsPathValid.Response
            The service response populated with ``is_valid``, ``success``, and
            ``invalid_pose_indices``.

        Returns
        -------
        IsPathValid.Response
            The response with validation results.

        """
        response.success = True
        response.is_valid = True
        response.invalid_pose_indices = []

        if not request.path.poses:
            self._logger.error('Received empty path. Cannot validate path.')
            response.success = False
            response.is_valid = False
            return response

        # Wait for costmap to become current (skip if timeout is zero or negative)
        if self._costmap_update_timeout > Duration(seconds=0):
            try:
                self._costmap_ros.wait_until_current(self._costmap_update_timeout)
            except Exception as ex:
                self._logger.error(f'Failed to wait for costmap: {ex}')
                response.success = False
                response.is_valid = False
                return response

        # Get current robot pose
        try:
            current_pose = self._costmap_ros.get_robot_pose()
            if current_pose is None:
                self._logger.error('Failed to get robot pose. Cannot validate path.')
                response.success = False
                response.is_valid = False
                return response
        except Exception as ex:
            self._logger.error(f'Failed to get robot pose: {ex}')
            response.success = False
            response.is_valid = False
            return response

        # The lethal check starts at the closest point to avoid points that have already been
        # passed and may have become occupied. The method for collision detection is based
        # on the shape of the footprint.
        closest_point_index = self._find_closest_point_index(current_pose, request.path)

        # Determine which costmap to use based on layer_name parameter
        costmap_to_check = self._get_costmap_to_check(request.layer_name)

        if not costmap_to_check:
            response.success = False
            response.is_valid = False
            return response

        # TODO(ajtudela): Add lock on costmap_to_check?

        # Determine footprint to use
        footprint, use_radius = self._get_footprint_to_use(request.footprint)

        if not use_radius and not footprint:
            response.success = False
            response.is_valid = False
            return response

        # TODO(ajtudela): Create collision checker as in C++ implementation

        # Determine the end index for validation based on lookahead distance
        end_index = len(request.path.poses)
        if request.max_lookahead_distance > 0.0:
            end_index = self._first_after_integrated_distance(
                request.path.poses,
                closest_point_index,
                request.max_lookahead_distance,
            )

        # Check each pose in the path
        for i in range(closest_point_index, end_index):
            position = request.path.poses[i].pose.position
            cost = FREE_SPACE

            if use_radius:
                # Use simple radius-based collision checking
                cost = self._get_cost_at_pose_radius(
                    costmap_to_check, position.x, position.y
                )
            else:
                # Use footprint-based collision checking
                theta = self._get_yaw_from_orientation(
                    request.path.poses[i].pose.orientation
                )
                # TODO(ajtudela): Move this function to collision checker class
                # and rename to footprint_cost_at_pose
                cost = self._get_cost_at_pose_footprint(
                    costmap_to_check, position.x, position.y, theta, footprint
                )

            # Handle unknown information
            if (cost == NO_INFORMATION and request.consider_unknown_as_obstacle):
                cost = LETHAL_OBSTACLE
            elif cost == NO_INFORMATION:
                cost = FREE_SPACE

            # Check collision conditions
            is_collision = False
            if use_radius:
                is_collision = (
                    cost >= request.max_cost
                    or cost == LETHAL_OBSTACLE
                    or cost == INSCRIBED_INFLATED_OBSTACLE
                )
            else:
                is_collision = (
                    cost == LETHAL_OBSTACLE or cost >= request.max_cost
                )

            if is_collision:
                response.is_valid = False
                response.invalid_pose_indices.append(i)
                if request.stop_at_first_collision:
                    break

        return response

    def _get_cost_at_pose_radius(
        self, costmap, x: float, y: float
    ) -> int:
        """
        Get cost at a pose using radius-based collision checking.

        Maps world coordinates to costmap cell indices and returns the cell cost,
        falling back to ``LETHAL_OBSTACLE`` when the pose is outside the costmap bounds.

        Parameters
        ----------
        costmap : Costmap2D
            The costmap to query.
        x : float
            X coordinate in the world (global) frame.
        y : float
            Y coordinate in the world (global) frame.

        Returns
        -------
        int
            Cost value at the given pose, or ``LETHAL_OBSTACLE`` if the pose is
            outside the costmap bounds or a mapping error occurs.

        """
        try:
            mx, my = costmap.worldToMap(x, y)
            return costmap.getCost(mx, my)
        except Exception:
            return LETHAL_OBSTACLE

    def _get_cost_at_pose_footprint(
        self,
        costmap,
        x: float,
        y: float,
        theta: float,
        footprint: List[Tuple[float, float]],
    ) -> int:
        """
        Get cost at a pose using footprint-based collision checking.

        Each footprint vertex is rotated and translated to the world frame and
        the maximum cell cost is returned, with an early exit on ``LETHAL_OBSTACLE``.

        Parameters
        ----------
        costmap : Costmap2D
            The costmap to query.
        x : float
            X coordinate of the robot centre in the world (global) frame.
        y : float
            Y coordinate of the robot centre in the world (global) frame.
        theta : float
            Robot heading (yaw) in radians.
        footprint : List[Tuple[float, float]]
            List of ``(x, y)`` footprint vertices relative to the robot centre.

        Returns
        -------
        int
            Maximum cost found across all footprint vertices, or
            ``LETHAL_OBSTACLE`` if any vertex is outside the costmap bounds or
            an error occurs.

        """
        max_cost = FREE_SPACE

        try:
            # Transform footprint points to world frame and check each one
            for fx, fy in footprint:
                # Rotate and translate footprint point
                world_x = (
                    x + fx * math.cos(theta) - fy * math.sin(theta)
                )
                world_y = (
                    y + fx * math.sin(theta) + fy * math.cos(theta)
                )

                # Get cost at this point
                mx, my = costmap.worldToMap(world_x, world_y)
                cost = costmap.getCost(mx, my)

                # Track maximum cost
                if cost > max_cost:
                    max_cost = cost

                # Early exit if lethal cost found
                if cost == LETHAL_OBSTACLE:
                    return cost

        except Exception:
            return LETHAL_OBSTACLE

        return max_cost

    def _first_after_integrated_distance(
        self,
        poses: List[PoseStamped],
        start_index: int,
        max_distance: float,
    ) -> int:
        """
        Find the end index based on integrated distance along the path.

        Limits path validation to a lookahead window when ``max_lookahead_distance > 0``.

        Parameters
        ----------
        poses : List[PoseStamped]
            Ordered list of poses constituting the path.
        start_index : int
            Index of the first pose to include in the distance integration
            (typically the closest point to the robot).
        max_distance : float
            Maximum cumulative Euclidean distance (metres) to look ahead from
            ``start_index``.

        Returns
        -------
        int
            Exclusive end index such that the arc length from ``start_index`` to
            ``end_index - 1`` does not exceed ``max_distance``. Returns
            ``len(poses)`` when the full remaining path is shorter than
            ``max_distance``.

        """
        accumulated_distance = 0.0

        for i in range(start_index, len(poses) - 1):
            p1 = poses[i].pose.position
            p2 = poses[i + 1].pose.position

            distance = math.sqrt(
                (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2
            )
            accumulated_distance += distance

            if accumulated_distance >= max_distance:
                return i + 1

        return len(poses)

    @staticmethod
    def _get_yaw_from_orientation(orientation) -> float:
        """
        Extract yaw angle from quaternion orientation.

        Parameters
        ----------
        orientation : Quaternion
            Quaternion orientation message with ``x``, ``y``, ``z``, and ``w``
            fields (e.g. ``geometry_msgs.msg.Quaternion``).

        Returns
        -------
        float
            Yaw angle extracted from the quaternion, in radians.

        """
        # Using the formula for extracting yaw from quaternion
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Calculate yaw
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        yaw = math.atan2(sin_yaw, cos_yaw)
        return yaw
