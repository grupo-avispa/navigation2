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
CostmapFilter base class for nav2_costmap_2d_py.

It mirrors the nav2_costmap_2d::CostmapFilter base from the C++ implementation:
filters subscribe to a ``CostmapFilterInfo`` topic to discover their mask topic
(and base/multiplier/type), buffer the robot pose during ``update_bounds`` and
apply themselves to the master grid during ``update_costs`` via ``process``.
"""

from typing import Any, List, Tuple

from geometry_msgs.msg import PointStamped, Pose
from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import SetBool

# --- filter_values.hpp constants ---------------------------------------------
KEEPOUT_FILTER = 0
SPEED_FILTER_PERCENT = 1
SPEED_FILTER_ABSOLUTE = 2
BINARY_FILTER = 3

BASE_DEFAULT = 0.0
MULTIPLIER_DEFAULT = 1.0

SPEED_MASK_UNKNOWN = -1
SPEED_MASK_NO_LIMIT = 0
NO_SPEED_LIMIT = 0.0

# --- occ_grid_values.hpp constants -------------------------------------------
OCC_GRID_UNKNOWN = -1
OCC_GRID_FREE = 0
OCC_GRID_OCCUPIED = 100


class CostmapFilter(Layer):
    """Base class for costmap filters."""

    def __init__(self) -> None:
        """Initialize the filter base state."""
        super().__init__()
        self._filter_info_topic = 'costmap_filter_info'
        self._mask_topic = ''
        self._transform_tolerance = 0.1
        self._global_frame = ''
        self._filter_info_sub: Any = None
        self._mask_sub: Any = None
        self._enable_service: Any = None
        # Robot pose buffered in update_bounds, consumed in update_costs.
        self._latest_pose = Pose()

    # ------------------------------------------------------------------
    # Layer interface (base implementation)
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Read the common filter parameters and create the enable/disable service."""
        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._filter_info_topic = self._declare_parameter_if_not_declared(
            'filter_info_topic', 'costmap_filter_info')
        self._transform_tolerance = self._declare_parameter_if_not_declared(
            'transform_tolerance', 0.1)
        self._global_frame = self._layered_costmap.get_global_frame_id()

        # Costmap filter enabling/disabling service.
        self._enable_service = self._node.create_service(
            SetBool, f'{self._name}/toggle_filter', self.enable_callback)

        self.initialize_filter(self._filter_info_topic)

    def enable_callback(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        """
        Enable/disable the costmap filter via a ``std_srvs/SetBool`` service.

        Parameters
        ----------
        request : SetBool.Request
            The request whose ``data`` field toggles the filter.
        response : SetBool.Response
            The response, filled with the success flag and a status message.

        Returns
        -------
        SetBool.Response
            ``success`` is True; ``message`` is ``"Enabled"`` or ``"Disabled"``.

        """
        self._enabled = request.data
        response.success = True
        response.message = 'Enabled' if self._enabled else 'Disabled'
        return response

    def update_bounds(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """Buffer the robot pose (filters do not expand the bounds by default)."""
        if not self._enabled:
            return
        self._latest_pose.position.x = robot_x
        self._latest_pose.position.y = robot_y
        self._latest_pose.position.z = 0.0
        # Only yaw is needed downstream; store it on the (unused) z of orientation
        # as a convenience for subclasses that need the heading.
        self._latest_yaw = robot_yaw

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """Run the filter ``process`` on the master grid for the buffered pose."""
        if not self._enabled:
            return
        self.process(master_grid, min_i, min_j, max_i, max_j, self._latest_pose)
        self.set_current(True)

    # ------------------------------------------------------------------
    # Hooks to be implemented by subclasses
    # ------------------------------------------------------------------

    def initialize_filter(self, filter_info_topic: str) -> None:
        """Subscribe to the filter info topic (override in subclasses)."""
        raise NotImplementedError

    def process(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int, max_i: int, max_j: int,
        pose: Pose,
    ) -> None:
        """Apply the filter to the master grid (override in subclasses)."""
        raise NotImplementedError

    def reset_filter(self) -> None:
        """Reset the filter subscriptions (override in subclasses)."""
        raise NotImplementedError

    def is_active(self) -> bool:
        """Return whether the filter has received its mask (override in subclasses)."""
        return False

    # ------------------------------------------------------------------
    # Shared helpers (mirror the C++ CostmapFilter methods)
    # ------------------------------------------------------------------

    def transform_pose(
        self,
        global_frame: str,
        wx: float, wy: float,
        mask_frame: str,
    ) -> Tuple[bool, float, float]:
        """
        Transform a world point from the costmap global frame into the mask frame.

        Parameters
        ----------
        global_frame : str
            The costmap global frame.
        wx, wy : float
            The point coordinates in ``global_frame``.
        mask_frame : str
            The filter mask frame.

        Returns
        -------
        (ok, mx_wx, mx_wy)
            ``ok`` is False if the transform failed; ``mx_wx``/``mx_wy`` are the
            point coordinates in ``mask_frame``.

        """
        if mask_frame == global_frame:
            return True, wx, wy
        if self._tf_buffer is None:
            return False, 0.0, 0.0
        point = PointStamped()
        point.header.frame_id = global_frame
        point.point.x = wx
        point.point.y = wy
        try:
            out = self._tf_buffer.transform(point, mask_frame)
        except Exception as ex:  # noqa: BLE001
            self._node.get_logger().error(
                f'CostmapFilter: failed to transform from {global_frame} to '
                f'{mask_frame}: {ex}', throttle_duration_sec=2.0)
            return False, 0.0, 0.0
        return True, out.point.x, out.point.y

    @staticmethod
    def mask_world_to_map(
        mask: OccupancyGrid, wx: float, wy: float
    ) -> Tuple[bool, int, int]:
        """
        Convert world coordinates to mask cell indices.

        Parameters
        ----------
        mask : OccupancyGrid
            The filter mask.
        wx, wy : float
            World coordinates in the mask frame.

        Returns
        -------
        (ok, mx, my)
            ``ok`` is False if the point lies outside the mask.

        """
        ox = mask.info.origin.position.x
        oy = mask.info.origin.position.y
        res = mask.info.resolution
        if wx < ox or wy < oy:
            return False, 0, 0
        mx = int((wx - ox) / res)
        my = int((wy - oy) / res)
        if mx < mask.info.width and my < mask.info.height:
            return True, mx, my
        return False, 0, 0

    @staticmethod
    def get_mask_cost(mask: OccupancyGrid, mx: int, my: int) -> int:
        """
        Read a mask cell and convert its occupancy to a cost.

        Parameters
        ----------
        mask : OccupancyGrid
            The filter mask.
        mx, my : int
            The mask cell indices.

        Returns
        -------
        int
            ``NO_INFORMATION`` for unknown cells, otherwise the occupancy linearly
            mapped to ``[FREE_SPACE..LETHAL_OBSTACLE]``.

        """
        data = mask.data[my * mask.info.width + mx]
        if data == OCC_GRID_UNKNOWN:
            return NO_INFORMATION
        return round(
            float(data) * (LETHAL_OBSTACLE - FREE_SPACE)
            / (OCC_GRID_OCCUPIED - OCC_GRID_FREE))

    @staticmethod
    def get_mask_data(mask: OccupancyGrid, mx: int, my: int) -> int:
        """
        Read the raw int8 value of a mask cell.

        Parameters
        ----------
        mask : OccupancyGrid
            The filter mask.
        mx, my : int
            The mask cell indices.

        Returns
        -------
        int
            The raw mask value (``-1`` for unknown, otherwise 0-100).

        """
        return mask.data[my * mask.info.width + mx]
