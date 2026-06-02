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

from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from typing import Any, Optional

from nav_msgs.msg import OccupancyGrid

from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION,
)

# OccupancyGrid threshold: cells >= this value are kept-out
_KEEPOUT_THRESHOLD = 99

_EPSILON = 1e-6


class KeepoutFilter(CostmapLayer):
    """
    Applies a binary keepout mask onto the master costmap.

    Parameters (under ``<name>.``):
      enabled          (bool, default True)
      filter_info_topic (str, default 'costmap_filter_info')
      mask_topic        (str, default 'keepout_filter_mask')
    """

    def __init__(self) -> None:
        super().__init__()
        self._filter_info_topic = 'costmap_filter_info'
        self._mask_topic = 'keepout_filter_mask'
        self._mask_sub: Optional[Any] = None
        self._filter_info_sub: Optional[Any] = None
        self._mask_received = False
        self._has_new_data = False

    def on_initialize(self) -> None:
        node = self._node
        name = self._name

        def _p(param, default):
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
        robot_x, robot_y, robot_yaw,
        min_x, min_y, max_x, max_y,
    ) -> None:
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
        min_i: int, min_j: int,
        max_i: int, max_j: int,
    ) -> None:
        if not self._enabled or not self._mask_received:
            return
        self.update_with_max(master_grid, min_i, min_j, max_i, max_j)
        self._has_new_data = False
        self._current = True

    def reset(self) -> None:
        self._has_new_data = True
        self._current = False

    def is_clearable(self) -> bool:
        return False

    def _mask_callback(self, msg: OccupancyGrid) -> None:
        """
        Receive the keepout mask and copy lethal cells into this layer.
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
