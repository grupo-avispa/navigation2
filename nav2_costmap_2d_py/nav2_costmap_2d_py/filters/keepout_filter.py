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

Costmap filter that marks keepout zones from an OccupancyGrid mask discovered
through a ``CostmapFilterInfo`` message.

It mirrors the nav2_costmap_2d::KeepoutFilter from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/KeepoutFilter"``
"""

from typing import Any, List, Optional

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    MAX_NON_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.filters.costmap_filter import (
    BASE_DEFAULT,
    CostmapFilter,
    MULTIPLIER_DEFAULT,
)
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

_INF = float('inf')


def _latched_qos(depth: int = 1) -> QoSProfile:
    """Build a latched (transient-local, reliable) QoS profile."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class KeepoutFilter(CostmapFilter):
    """
    Read in a keepout mask and mark keepout regions in the map.

    Prevents planning or control in restricted areas by applying a keepout mask
    (from an OccupancyGrid) onto the master costmap.
    """

    def __init__(self) -> None:
        """Initialize keepout filter defaults."""
        super().__init__()
        self._filter_mask: Optional[OccupancyGrid] = None
        self._override_lethal_cost = False
        self._lethal_override_cost = MAX_NON_OBSTACLE
        self._is_pose_lethal = False
        self._last_pose_lethal = False
        self._has_updated_data = False
        self._x = 0
        self._y = 0
        self._width = 0
        self._height = 0
        self._lethal_state_update_min_x = _INF
        self._lethal_state_update_min_y = _INF
        self._lethal_state_update_max_x = -_INF
        self._lethal_state_update_max_y = -_INF

    def initialize_filter(self, filter_info_topic: str) -> None:
        """
        Subscribe to the filter info topic and read keepout-specific parameters.

        Parameters
        ----------
        filter_info_topic : str
            The topic to subscribe for ``CostmapFilterInfo`` messages.

        """
        node = self._node
        self._filter_info_topic = self.join_with_parent_namespace(filter_info_topic)
        self._override_lethal_cost = self._declare_parameter_if_not_declared(
            'override_lethal_cost', False)
        lethal_override = self._declare_parameter_if_not_declared(
            'lethal_override_cost', MAX_NON_OBSTACLE)
        self._lethal_override_cost = max(FREE_SPACE, min(int(lethal_override), MAX_NON_OBSTACLE))

        self._filter_info_sub = node.create_subscription(
            CostmapFilterInfo, self._filter_info_topic,
            self._filter_info_callback, _latched_qos())
        node.get_logger().info(
            f'[KeepoutFilter] "{self._name}" subscribing to filter info '
            f'"{self._filter_info_topic}"')

    def _filter_info_callback(self, msg: CostmapFilterInfo) -> None:
        """
        Read the filter info, then subscribe to the announced mask topic.

        Parameters
        ----------
        msg : CostmapFilterInfo
            The filter info message.

        """
        if msg.base != BASE_DEFAULT or msg.multiplier != MULTIPLIER_DEFAULT:
            self._node.get_logger().error(
                'KeepoutFilter: for proper use, base and multiplier in '
                f'CostmapFilterInfo should be {BASE_DEFAULT} and {MULTIPLIER_DEFAULT}')

        self._mask_topic = self.join_with_parent_namespace(msg.filter_mask_topic)
        self._mask_sub = self._node.create_subscription(
            OccupancyGrid, self._mask_topic, self._mask_callback, _latched_qos(3))
        self._node.get_logger().info(
            f'[KeepoutFilter] "{self._name}" subscribing to mask "{self._mask_topic}"')

    def _mask_callback(self, msg: OccupancyGrid) -> None:
        """
        Store the received filter mask.

        Parameters
        ----------
        msg : OccupancyGrid
            The filter mask message.

        """
        self._filter_mask = msg
        self._has_updated_data = True
        self._x = 0
        self._y = 0
        self._width = msg.info.width
        self._height = msg.info.height

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
        Buffer the robot pose and, on new mask data, expand the bounds.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if not self._enabled:
            return
        super().update_bounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y)

        if self._filter_mask is None:
            return

        master = self._layered_costmap.get_costmap()

        if self._has_updated_data:
            wx, wy = master.map_to_world(self._x, self._y)
            min_x[0] = min(wx, min_x[0])
            min_y[0] = min(wy, min_y[0])
            wx, wy = master.map_to_world(self._x + self._width, self._y + self._height)
            max_x[0] = max(wx, max_x[0])
            max_y[0] = max(wy, max_y[0])
            self._has_updated_data = False
            return

        # override_lethal_cost: find the cost at the robot's pose.
        self._is_pose_lethal = False
        if self._override_lethal_cost:
            ok, msk_wx, msk_wy = self.transform_pose(
                self._global_frame, robot_x, robot_y, self._filter_mask.header.frame_id)
            if ok:
                in_mask, mx, my = self.mask_world_to_map(self._filter_mask, msk_wx, msk_wy)
                if in_mask:
                    data = self.get_mask_cost(self._filter_mask, mx, my)
                    self._is_pose_lethal = data in (
                        INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE)

            if self._is_pose_lethal or (self._last_pose_lethal and not self._is_pose_lethal):
                self._lethal_state_update_min_x = min(min_x[0], self._lethal_state_update_min_x)
                min_x[0] = self._lethal_state_update_min_x
                self._lethal_state_update_min_y = min(min_y[0], self._lethal_state_update_min_y)
                min_y[0] = self._lethal_state_update_min_y
                self._lethal_state_update_max_x = max(max_x[0], self._lethal_state_update_max_x)
                max_x[0] = self._lethal_state_update_max_x
                self._lethal_state_update_max_y = max(max_y[0], self._lethal_state_update_max_y)
                max_y[0] = self._lethal_state_update_max_y
            else:
                self._lethal_state_update_min_x = _INF
                self._lethal_state_update_min_y = _INF
                self._lethal_state_update_max_x = -_INF
                self._lethal_state_update_max_y = -_INF

    def process(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
        pose: Any,
    ) -> None:
        """
        Apply the keepout mask onto the master grid window.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the window to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to update, in cells.
        pose : Any
            The robot's pose, for potential use in lethal override.

        """
        if self._filter_mask is None:
            self._node.get_logger().warning(
                '[KeepoutFilter] Filter mask was not received',
                throttle_duration_sec=2.0)
            return

        mask = self._filter_mask
        mask_frame = mask.header.frame_id
        same_frame = (mask_frame == self._global_frame)

        # When frames differ, look up the transform once and apply it per cell.
        r00 = r11 = 1.0
        r01 = r10 = tx = ty = 0.0
        if not same_frame:
            if self._tf_buffer is None:
                return
            try:
                transform = self._tf_buffer.lookup_transform(
                    mask_frame, self._global_frame, _tf_time(),
                    timeout=_tf_duration(self._transform_tolerance))
            except Exception as ex:  # noqa: BLE001
                self._node.get_logger().error(
                    f'[KeepoutFilter] failed to transform {self._global_frame} '
                    f'to {mask_frame}: {ex}', throttle_duration_sec=2.0)
                return
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
            xy = q.x * q.y
            wz_ = q.w * q.z
            r00, r01 = 1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz_)
            r10, r11 = 2.0 * (xy + wz_), 1.0 - 2.0 * (xx + zz)

        for i in range(min_i, max_i):
            for j in range(min_j, max_j):
                gl_wx, gl_wy = master_grid.map_to_world(i, j)
                if same_frame:
                    msk_wx, msk_wy = gl_wx, gl_wy
                else:
                    msk_wx = tx + r00 * gl_wx + r01 * gl_wy
                    msk_wy = ty + r10 * gl_wx + r11 * gl_wy

                in_mask, mx, my = self.mask_world_to_map(mask, msk_wx, msk_wy)
                if not in_mask:
                    continue
                data = self.get_mask_cost(mask, mx, my)
                if data == NO_INFORMATION:
                    continue
                old_data = master_grid.get_cost(i, j)
                if data > old_data or old_data == NO_INFORMATION:
                    if self._override_lethal_cost and self._is_pose_lethal:
                        master_grid.set_cost(i, j, self._lethal_override_cost)
                    else:
                        master_grid.set_cost(i, j, data)

        self._last_pose_lethal = self._is_pose_lethal

    def reset(self) -> None:
        """Reset the filter, forcing a re-apply of the mask on the next cycle."""
        self._has_updated_data = True
        self.set_current(False)

    def reset_filter(self) -> None:
        """Drop the filter info and mask subscriptions."""
        self._filter_info_sub = None
        self._mask_sub = None

    def is_active(self) -> bool:
        """Return whether the mask has been received."""
        return self._filter_mask is not None

    def is_clearable(self) -> bool:
        """Keepout regions are never cleared."""
        return False


def _tf_time() -> Any:
    """Return a zero rclpy Time (latest available transform)."""
    from rclpy.time import Time
    return Time()


def _tf_duration(seconds: float) -> Any:
    """
    Return an rclpy Duration for the given seconds.

    Parameters
    ----------
    seconds : float
        The duration in seconds.

    """
    from rclpy.duration import Duration
    return Duration(seconds=seconds)
