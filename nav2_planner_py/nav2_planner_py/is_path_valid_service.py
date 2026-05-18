# Copyright (c) 2025 Nav2 Python Port
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
Service to determine if a path is still valid given the current costmap state.
"""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.srv import IsPathValid

# Cost values from nav2_costmap_2d
FREE_SPACE = 0
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
NO_INFORMATION = 255


class IsPathValidService:
    """
    Service to determine if a path is still valid given the current costmap state.

    Responsibilities:
        - Create and manage IsPathValid ROS2 service
        - Validate paths against the costmap
        - Check for collisions using footprint or radius-based detection
        - Handle layer-specific costmap checking
    """

    def __init__(
        self,
        node: LifecycleNode,
        costmap_ros,
        costmap_update_timeout: Duration,
    ) -> None:
        """
        Initialize the IsPathValidService.

        Parameters
        ----------
        node : LifecycleNode
            The ROS2 lifecycle node for service creation.
        costmap_ros : Costmap2DROS
            The costmap ROS wrapper for collision checking.
        costmap_update_timeout : Duration
            Timeout for waiting for costmap updates.
        """
        self._node = node
        self._costmap_ros = costmap_ros
        self._costmap_update_timeout = costmap_update_timeout
        self._logger = rclpy.logging.get_logger("is_path_valid_service")
        self._costmap = None
        self._service = None

    def initialize(self) -> None:
        """Initialize the IsPathValid service."""
        if self._node is None:
            raise RuntimeError('Failed to lock node in initialize')

        self._costmap = self._costmap_ros.get_costmap()

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

        Parameters
        ----------
        layer_name : str
            Name of the layer to check, or empty for full costmap.

        Returns
        -------
        Costmap2D or None
            Pointer to the costmap to check, or None if layer not found.
        """
        if not layer_name:
            return self._costmap

        try:
            layers = self._costmap_ros.get_layered_costmap().get_plugins()
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

        Parameters
        ----------
        footprint_string : str
            Custom footprint string, or empty to use robot's footprint.

        Returns
        -------
        Tuple[List[Tuple[float, float]], bool]
            (footprint, use_radius) where footprint is a list of (x, y) points
            and use_radius indicates if radius-based checking should be used.
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

    def _parse_footprint_string(
        self, footprint_string: str
    ) -> List[Tuple[float, float]]:
        """
        Parse a footprint string into a list of (x, y) points.

        Parameters
        ----------
        footprint_string : str
            String in format "[[x1, y1], [x2, y2], ...]"

        Returns
        -------
        List[Tuple[float, float]]
            List of (x, y) coordinate pairs.

        Raises
        ------
        ValueError
            If the footprint string is invalid.
        """
        import re

        # Simple parser for footprint format
        footprint_string = footprint_string.strip()
        if not footprint_string.startswith('[[') or not footprint_string.endswith(']]'):
            raise ValueError('Invalid footprint format')

        # Extract numbers
        numbers = re.findall(r"-?\d+\.?\d*", footprint_string)
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

        Parameters
        ----------
        current_pose : PoseStamped
            The current robot pose.
        path : Path
            The path to search.

        Returns
        -------
        int
            Index of the closest pose in the path.
        """
        closest_point_index = 0
        closest_distance = float('inf')
        current_point = current_pose.pose.position

        for i, pose in enumerate(path.poses):
            path_point = pose.pose.position
            distance = math.sqrt(
                (current_point.x - path_point.x) ** 2
                + (current_point.y - path_point.y) ** 2
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

        Parameters
        ----------
        request : IsPathValid.Request
            The service request containing the path to validate.
        response : IsPathValid.Response
            The service response to be filled.

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
                self._costmap_ros.wait_until_current(
                    self._costmap_update_timeout
                )
            except Exception as ex:
                self._logger.error(f'Failed to wait for costmap: {ex}')
                response.success = False
                response.is_valid = False
                return response

        # Get current robot pose
        try:
            current_pose = self._costmap_ros.get_robot_pose()
            if current_pose is None:
                self._logger.error(
                    'Failed to get robot pose. Cannot validate path.')
                response.success = False
                response.is_valid = False
                return response
        except Exception as ex:
            self._logger.error(f'Failed to get robot pose: {ex}')
            response.success = False
            response.is_valid = False
            return response

        # Find closest point on the path to avoid checking already-passed points
        closest_point_index = self._find_closest_point_index(
            current_pose, request.path
        )

        # Determine which costmap to use based on layer_name parameter
        costmap_to_check = self._get_costmap_to_check(request.layer_name)

        if not costmap_to_check:
            response.success = False
            response.is_valid = False
            return response

        # Determine footprint to use
        footprint, use_radius = self._get_footprint_to_use(request.footprint)

        if not use_radius and not footprint:
            response.success = False
            response.is_valid = False
            return response

        # Determine the end index for validation based on lookahead distance
        end_index = len(request.path.poses)
        if request.max_lookahead_distance > 0.0:
            end_index = self._find_end_index_by_distance(
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
                cost = self._get_cost_at_pose_footprint(
                    costmap_to_check, position.x, position.y, theta, footprint
                )

            # Handle unknown information
            if (
                cost == NO_INFORMATION
                and request.consider_unknown_as_obstacle
            ):
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

        Parameters
        ----------
        costmap
            The costmap to check.
        x : float
            X coordinate in world frame.
        y : float
            Y coordinate in world frame.

        Returns
        -------
        int
            Cost value at the given pose.
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

        Parameters
        ----------
        costmap
            The costmap to check.
        x : float
            X coordinate in world frame.
        y : float
            Y coordinate in world frame.
        theta : float
            Orientation angle in radians.
        footprint : List[Tuple[float, float]]
            List of (x, y) footprint points relative to robot center.

        Returns
        -------
        int
            Cost value (max cost of any footprint point).
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

    def _find_end_index_by_distance(
        self,
        poses: List[PoseStamped],
        start_index: int,
        max_distance: float,
    ) -> int:
        """
        Find the end index based on integrated distance along the path.

        Parameters
        ----------
        poses : List[PoseStamped]
            List of poses in the path.
        start_index : int
            Starting index.
        max_distance : float
            Maximum distance to look ahead.

        Returns
        -------
        int
            Index after traveling max_distance from start_index.
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
        orientation
            Quaternion orientation (with x, y, z, w fields).

        Returns
        -------
        float
            Yaw angle in radians.
        """
        # Using the formula for extracting yaw from quaternion
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # Calculate yaw (rotation around z-axis)
        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        sin_pitch = 2.0 * (w * y - z * x)
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)

        yaw = math.atan2(sin_yaw, cos_yaw)
        return yaw
