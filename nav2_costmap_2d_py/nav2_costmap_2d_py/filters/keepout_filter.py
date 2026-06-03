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
KeepoutFilter for nav2_costmap_2d_py.

Costmap filter that marks keepout zones from an OccupancyGrid mask.

It mirrors the nav2_costmap_2d::KeepoutFilter from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/KeepoutFilter"``
"""

from typing import Any, List, Optional

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# OccupancyGrid threshold: cells >= this value are kept-out
_KEEPOUT_THRESHOLD = 99

_EPSILON = 1e-6


class KeepoutFilter(CostmapLayer):
    """
    Read in a keepout mask and mark keepout regions in the map.

    Prevents planning or control in restricted areas by applying a binary
    keepout mask (from an OccupancyGrid) onto the master costmap.
    """

    def __init__(self) -> None:
        """Initialize keepout filter defaults."""
        super().__init__()
        self._filter_info_topic = 'costmap_filter_info'
        self._mask_topic = 'keepout_filter_mask'
        self._mask_sub: Optional[Any] = None
        self._filter_info_sub: Optional[Any] = None
        self._mask_received = False
        self._has_new_data = False

    def on_initialize(self) -> None:
        """Initialize the filter on startup: read parameters and subscribe to the mask topic."""
        node = self._node
        name = self._name

        def _p(param: str, default: Any) -> Any:
            full = f'{name}.{param}'
            if not node.has_parameter(full):
                node.declare_parameter(full, default)
            return node.get_parameter(full).value

        self._enabled = _p('enabled', True)
        # Resolve against the parent namespace (see Layer.join_with_parent_namespace):
        # the costmap node sits in the '/global_costmap' sub-namespace.
        self._mask_topic = self.join_with_parent_namespace(
            _p('mask_topic', 'keepout_filter_mask'))

        # The filter mask is latched
        mask_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mask_sub = node.create_subscription(
            OccupancyGrid,
            self._mask_topic,
            self._mask_callback,
            mask_qos,
        )

        node.get_logger().info(
            f'[KeepoutFilter] "{name}" subscribing to mask "{self._mask_topic}"'
        )

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
        """
        Update the bounds of the master costmap by this filter's update dimensions.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if not self._enabled or not self._has_new_data:
            return
        master = self._layered_costmap.get_costmap()
        ox = master.origin_x
        oy = master.origin_y
        min_x[0] = min(min_x[0], ox)
        min_y[0] = min(min_y[0], oy)
        max_x[0] = max(max_x[0], ox + master.size_x_meters)
        max_y[0] = max(max_y[0], oy + master.size_y_meters)

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Process the keepout layer, applying the mask onto the master costmap window.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the window to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to update, in cells.

        """
        if not self._enabled or not self._mask_received:
            return
        self.update_with_max(master_grid, min_i, min_j, max_i, max_j)
        self._has_new_data = False
        self._current = True

    def reset(self) -> None:
        """Reset the costmap filter, forcing a re-apply of the mask on the next cycle."""
        self._has_new_data = True
        self._current = False

    def is_clearable(self) -> bool:
        """
        Return whether clearing operations should be processed on this filter.

        Returns
        -------
        bool
            Always ``False``; keepout regions are never cleared.

        """
        return False

    def _mask_callback(self, msg: OccupancyGrid) -> None:
        """
        Handle the filter mask: copy keepout (lethal) cells from the mask into this layer.

        Parameters
        ----------
        msg : OccupancyGrid
            The incoming keepout mask, resizing this layer if its geometry
            changed.

        """
        new_w = msg.info.width
        new_h = msg.info.height
        new_res = msg.info.resolution
        new_ox = msg.info.origin.position.x
        new_oy = msg.info.origin.position.y

        if not self._mask_received:
            self.resize_map(new_w, new_h, new_res, new_ox, new_oy)
        elif (new_w != self._size_x or new_h != self._size_y
              or abs(new_res - self._resolution) > _EPSILON
              or abs(new_ox - self._origin_x) > _EPSILON
              or abs(new_oy - self._origin_y) > _EPSILON):
            self.resize_map(new_w, new_h, new_res, new_ox, new_oy)

        for idx, val in enumerate(msg.data):
            if val == -1:
                self._costmap[idx] = NO_INFORMATION
            elif val >= _KEEPOUT_THRESHOLD:
                self._costmap[idx] = LETHAL_OBSTACLE
            else:
                self._costmap[idx] = FREE_SPACE

        self._mask_received = True
        self._has_new_data = True
        self._current = True
