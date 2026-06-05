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
SpeedFilter for nav2_costmap_2d_py.

Costmap filter that reads a speed-limit mask (discovered through a
``CostmapFilterInfo`` message) and publishes ``nav2_msgs/msg/SpeedLimit``
messages when the robot enters / leaves a speed-limited zone.

It mirrors the nav2_costmap_2d::SpeedFilter from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/SpeedFilter"``
"""

from typing import Any, Optional

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.filters.costmap_filter import (
    CostmapFilter,
    NO_SPEED_LIMIT,
    SPEED_FILTER_ABSOLUTE,
    SPEED_FILTER_PERCENT,
    SPEED_MASK_NO_LIMIT,
    SPEED_MASK_UNKNOWN,
)
from nav2_msgs.msg import CostmapFilterInfo, SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def _latched_qos(depth: int = 1) -> QoSProfile:
    """
    Build a latched (transient-local, reliable) QoS profile.

    Parameters
    ----------
    depth : int
        The depth of the QoS history (default: 1).

    """
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class SpeedFilter(CostmapFilter):
    """
    Read in a speed restriction mask to dynamically adjust the robot's speed.

    Publishes ``nav2_msgs/msg/SpeedLimit`` messages when the robot enters or
    leaves a speed-limited zone, based on its pose in the mask.
    """

    def __init__(self) -> None:
        """Initialize speed filter defaults."""
        super().__init__()
        self._speed_limit_topic = 'speed_limit'
        self._filter_mask: Optional[OccupancyGrid] = None
        self._percentage = False
        self._base = 0.0
        self._multiplier = 1.0
        self._speed_limit = NO_SPEED_LIMIT
        self._speed_limit_prev = NO_SPEED_LIMIT
        self._speed_limit_pub: Any = None

    def initialize_filter(self, filter_info_topic: str) -> None:
        """
        Subscribe to the filter info topic and create the speed limit publisher.

        Parameters
        ----------
        filter_info_topic : str
            The topic to subscribe for ``CostmapFilterInfo`` messages.

        """
        node = self._node
        self._filter_info_topic = self.join_with_parent_namespace(filter_info_topic)
        self._speed_limit_topic = self._declare_parameter_if_not_declared(
            'speed_limit_topic', 'speed_limit')

        self._filter_info_sub = node.create_subscription(
            CostmapFilterInfo, self._filter_info_topic,
            self._filter_info_callback, _latched_qos())
        self._speed_limit_pub = node.create_publisher(
            SpeedLimit, self._speed_limit_topic, _latched_qos())
        node.get_logger().info(
            f'[SpeedFilter] "{self._name}" subscribing to filter info '
            f'"{self._filter_info_topic}", publishing to "{self._speed_limit_topic}"')

    def _filter_info_callback(self, msg: CostmapFilterInfo) -> None:
        """
        Read base/multiplier/type and subscribe to the announced mask topic.

        Parameters
        ----------
        msg : CostmapFilterInfo
            The filter info message.

        """
        self._base = msg.base
        self._multiplier = msg.multiplier
        if msg.type == SPEED_FILTER_PERCENT:
            self._percentage = True
        elif msg.type == SPEED_FILTER_ABSOLUTE:
            self._percentage = False
        else:
            self._node.get_logger().error('SpeedFilter: mode is not supported')
            return

        self._mask_topic = self.join_with_parent_namespace(msg.filter_mask_topic)
        self._mask_sub = self._node.create_subscription(
            OccupancyGrid, self._mask_topic, self._mask_callback, _latched_qos(3))
        self._node.get_logger().info(
            f'[SpeedFilter] "{self._name}" subscribing to mask "{self._mask_topic}" '
            f'(speed_limit = {self._base} + mask_data * {self._multiplier})')

    def _mask_callback(self, msg: OccupancyGrid) -> None:
        """
        Store the received filter mask.

        Parameters
        ----------
        msg : OccupancyGrid
            The filter mask message.

        """
        self._filter_mask = msg

    def process(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int, max_i: int, max_j: int,
        pose: Any,
    ) -> None:
        """
        Look up the speed limit at the robot pose and publish it on change.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap.
        min_i, min_j, max_i, max_j : int
            The bounds of the region to process.
        pose : Any
            The robot's pose.

        """
        if self._filter_mask is None:
            self._node.get_logger().warning(
                '[SpeedFilter] Filter mask was not received',
                throttle_duration_sec=2.0)
            return

        mask = self._filter_mask
        ok, msk_wx, msk_wy = self.transform_pose(
            self._global_frame, pose.position.x, pose.position.y,
            mask.header.frame_id)
        if not ok:
            return

        in_mask, mx, my = self.mask_world_to_map(mask, msk_wx, msk_wy)
        if not in_mask:
            return

        speed_mask_data = self.get_mask_data(mask, mx, my)
        if speed_mask_data == SPEED_MASK_NO_LIMIT:
            # Free cell: no speed limit.
            self._speed_limit = NO_SPEED_LIMIT
        elif speed_mask_data == SPEED_MASK_UNKNOWN:
            # Unknown cell: invalid for this filter, do nothing.
            self._node.get_logger().error(
                f'[SpeedFilter] Found unknown cell in filter_mask[{mx}, {my}], '
                'which is invalid for this kind of filter')
            return
        else:
            # Normal case: speed_mask_data in [1..100].
            self._speed_limit = speed_mask_data * self._multiplier + self._base
            if self._percentage:
                if self._speed_limit < 0.0 or self._speed_limit > 100.0:
                    self._node.get_logger().warning(
                        f'[SpeedFilter] Speed limit {self._speed_limit}% out of '
                        '[0, 100]; setting it to no-limit.')
                    self._speed_limit = NO_SPEED_LIMIT
            elif self._speed_limit < 0.0:
                self._node.get_logger().warning(
                    f'[SpeedFilter] Speed limit {self._speed_limit} m/s < 0; '
                    'setting it to no-limit.')
                self._speed_limit = NO_SPEED_LIMIT

        if self._speed_limit != self._speed_limit_prev:
            self._publish_speed_limit(self._speed_limit)
            self._speed_limit_prev = self._speed_limit

    def _publish_speed_limit(self, speed_limit: float) -> None:
        """
        Publish a ``nav2_msgs/msg/SpeedLimit`` message with the given limit.

        Parameters
        ----------
        speed_limit : float
            The speed limit to publish (percentage of max speed or absolute
            speed, depending on the ``percentage`` parameter).

        """
        msg = SpeedLimit()
        msg.header.frame_id = self._global_frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.percentage = self._percentage
        msg.speed_limit = float(speed_limit)
        self._speed_limit_pub.publish(msg)

    def reset(self) -> None:
        """Reset the last published speed limit."""
        self._speed_limit_prev = NO_SPEED_LIMIT
        self.set_current(False)

    def reset_filter(self) -> None:
        """Drop the filter info and mask subscriptions."""
        self._filter_info_sub = None
        self._mask_sub = None

    def is_active(self) -> bool:
        """Return whether the mask has been received."""
        return self._filter_mask is not None

    def is_current(self) -> bool:
        """Return True: the speed filter is always considered current."""
        return True
