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
ClearCostmapService for nav2_costmap_2d_py.

It mirrors the nav2_costmap_2d::ClearCostmapService from the C++ implementation.
Exposes services to clear costmap objects in inclusive/exclusive regions or completely.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple, TYPE_CHECKING

from nav2_msgs.srv import (ClearCostmapAroundPose, ClearCostmapAroundRobot,
                           ClearCostmapExceptRegion, ClearEntireCostmap)

if TYPE_CHECKING:
    from nav2_costmap_2d_py.costmap_2d_ros import Costmap2DROS


class ClearCostmapService:
    """
    Expose services to clear costmap objects in inclusive/exclusive regions or completely.

    Parameters
    ----------
    node :
        The lifecycle node that owns the services (Costmap2DROS).
    costmap_ros :
        The Costmap2DROS instance to clear (provides ``get_costmap()``,
        ``get_robot_pose()``, ``get_name()``).

    """

    def __init__(self, node: 'Costmap2DROS', costmap_ros: 'Costmap2DROS') -> None:
        """
        Initialize the service, creating the four clearing service servers.

        Parameters
        ----------
        node : Costmap2DROS
            The lifecycle node that owns the services.
        costmap_ros : Costmap2DROS
            The Costmap2DROS instance to clear (provides ``get_costmap()``,
            ``get_robot_pose()``, ``get_name()``).

        """
        self._node = node
        self._costmap_ros = costmap_ros
        self._reset_value = costmap_ros.get_costmap().default_value
        name = costmap_ros.get_name()

        self._clear_except_srv = node.create_service(
            ClearCostmapExceptRegion, f'clear_except_{name}', self._clear_except_callback,
        )

        self._clear_around_srv = node.create_service(
            ClearCostmapAroundRobot, f'clear_around_{name}', self._clear_around_callback,
        )

        self._clear_entirely_srv = node.create_service(
            ClearEntireCostmap, f'clear_entirely_{name}', self._clear_entirely_callback,
        )

        self._clear_around_pose_srv = node.create_service(
            ClearCostmapAroundPose, f'clear_around_pose_{name}', self._clear_around_pose_callback,
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _clear_except_callback(
        self,
        request: ClearCostmapExceptRegion.Request,
        response: ClearCostmapExceptRegion.Response,
    ) -> ClearCostmapExceptRegion.Response:
        """
        Clear costmap except in a given region (everything outside the robot area).

        Parameters
        ----------
        request : ClearCostmapExceptRegion.Request
            The service request holding the ``reset_distance``.
        response : ClearCostmapExceptRegion.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearCostmapExceptRegion.Response
            The service response.

        """
        reset_distance = request.reset_distance
        if not self._clear_except_region(reset_distance):
            self._node.get_logger().error(
                '[ClearCostmapService] Could not get robot pose for clear_except service'
            )
        return response

    def _clear_around_callback(
        self,
        request: ClearCostmapAroundRobot.Request,
        response: ClearCostmapAroundRobot.Response,
    ) -> ClearCostmapAroundRobot.Response:
        """
        Clear costmap in a given region (the area around the robot).

        Parameters
        ----------
        request : ClearCostmapAroundRobot.Request
            The service request holding the ``reset_distance``.
        response : ClearCostmapAroundRobot.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearCostmapAroundRobot.Response
            The service response.

        """
        reset_distance = request.reset_distance
        if not self._clear_around(reset_distance):
            self._node.get_logger().error(
                '[ClearCostmapService] Could not get robot pose for clear_around service'
            )
        return response

    def _clear_entirely_callback(
        self,
        request: ClearEntireCostmap.Request,
        response: ClearEntireCostmap.Response,
    ) -> ClearEntireCostmap.Response:
        """
        Clear the entire costmap and all its layers.

        Parameters
        ----------
        request : ClearEntireCostmap.Request
            The (empty) service request.
        response : ClearEntireCostmap.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearEntireCostmap.Response
            The service response.

        """
        self._clear_entirely()
        return response

    def _clear_around_pose_callback(
        self,
        request: ClearCostmapAroundPose.Request,
        response: ClearCostmapAroundPose.Response,
    ) -> ClearCostmapAroundPose.Response:
        """
        Clear costmap around a given pose.

        Parameters
        ----------
        request : ClearCostmapAroundPose.Request
            The service request holding the target ``pose`` and ``reset_distance``.
        response : ClearCostmapAroundPose.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearCostmapAroundPose.Response
            The service response.

        """
        self._clear_around_world_pose(
            request.pose.pose.position.x,
            request.pose.pose.position.y,
            request.reset_distance,
            invert=False,
        )
        return response

    # ------------------------------------------------------------------
    # Implementation helpers
    # ------------------------------------------------------------------

    def _get_robot_position(self) -> Optional[Tuple[float, float]]:
        """
        Get the robot's position in the costmap using the master costmap.

        Returns
        -------
        tuple of float or None
            The ``(x, y)`` world position of the robot, or ``None`` if the
            robot pose could not be obtained.

        """
        pose = self._costmap_ros.get_robot_pose()
        if pose is None:
            return None
        return pose.pose.position.x, pose.pose.position.y

    def _clear_except_region(self, reset_distance: float) -> bool:
        """
        Clear the region outside of the user-specified area reverting to the static map.

        Parameters
        ----------
        reset_distance : float
            Half-side of the square area around the robot to preserve; everything
            beyond it is reverted to the default value.

        Returns
        -------
        bool
            ``True`` on success, ``False`` if the robot pose was unavailable.

        """
        pos = self._get_robot_position()
        if pos is None:
            return False
        x, y = pos
        self._clear_around_world_pose(x, y, reset_distance, invert=True)
        return True

    def _clear_around(self, reset_distance: float) -> bool:
        """
        Clear the region inside the user-specified area around the robot.

        Parameters
        ----------
        reset_distance : float
            Distance around the robot inside which cells are reverted to the
            default value.

        Returns
        -------
        bool
            ``True`` on success, ``False`` if the robot pose was unavailable.

        """
        pos = self._get_robot_position()
        if pos is None:
            return False
        x, y = pos
        self._clear_around_world_pose(x, y, reset_distance, invert=False)
        return True

    def _clear_around_world_pose(
        self,
        wx: float,
        wy: float,
        reset_distance: float,
        invert: bool,
    ) -> None:
        """
        Clear a given costmap layer around a world pose.

        Parameters
        ----------
        wx, wy : float
            World coordinates of the pose to clear around.
        reset_distance : float
            Distance from ``(wx, wy)`` that delimits the region to clear.
        invert : bool
            If ``False``, clears cells *within* ``reset_distance``; if ``True``,
            clears cells *outside* it (keeping the inner region intact).

        """
        costmap = self._costmap_ros.get_costmap()
        with costmap.get_mutex():
            sx = costmap.size_x
            sy = costmap.size_y
            res = costmap.resolution
            ox = costmap.origin_x
            oy = costmap.origin_y
            arr = costmap.get_char_map()

            for my in range(sy):
                for mx in range(sx):
                    world_x = ox + (mx + 0.5) * res
                    world_y = oy + (my + 0.5) * res
                    dist = math.sqrt((world_x - wx) ** 2 + (world_y - wy) ** 2)
                    in_region = dist <= reset_distance
                    if in_region != invert:
                        idx = my * sx + mx
                        arr[idx] = self._reset_value

    def _clear_entirely(self) -> None:
        """Clear the entire costmap, resetting the master grid and all clearable layers."""
        costmap = self._costmap_ros.get_costmap()
        with costmap.get_mutex():
            costmap.reset_maps()
        # Propagate to all layers
        lc = self._costmap_ros.get_layered_costmap()
        if lc:
            for layer in lc.get_plugins():
                if layer.is_clearable():
                    layer.reset()
            for f in lc.get_filters():
                if f.is_clearable():
                    f.reset()
