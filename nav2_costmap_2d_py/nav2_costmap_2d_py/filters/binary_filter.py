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
BinaryFilter for nav2_costmap_2d_py.

Costmap filter that reads a binary mask (discovered through a
``CostmapFilterInfo`` message) and flips a binary state, published as a
``std_msgs/msg/Bool`` message, when the robot enters / leaves a flagged zone.

It mirrors the nav2_costmap_2d::BinaryFilter from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/BinaryFilter"``
"""

from typing import Any, Optional

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.filters.costmap_filter import (
    BASE_DEFAULT,
    BINARY_FILTER,
    CostmapFilter,
    MULTIPLIER_DEFAULT,
    OCC_GRID_UNKNOWN,
)
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool


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


class BinaryFilter(CostmapFilter):
    """
    Read in a binary mask and flip a binary state based on the robot's pose.

    Publishes a ``std_msgs/msg/Bool`` message whenever the robot enters or
    leaves a flagged zone in the mask.
    """

    def __init__(self) -> None:
        """Initialize binary filter defaults."""
        super().__init__()
        self._binary_state_topic = 'binary_state'
        self._filter_mask: Optional[OccupancyGrid] = None
        self._base = 0.0
        self._multiplier = 1.0
        self._flip_threshold = 50.0
        self._default_state = False
        self._binary_state = self._default_state
        self._binary_state_pub: Any = None

    def initialize_filter(self, filter_info_topic: str) -> None:
        """
        Subscribe to the filter info topic and create the binary state publisher.

        Parameters
        ----------
        filter_info_topic : str
            The topic to subscribe for ``CostmapFilterInfo`` messages.

        """
        node = self._node
        self._filter_info_topic = self.join_with_parent_namespace(filter_info_topic)
        self._default_state = self._declare_parameter_if_not_declared('default_state', False)
        self._binary_state_topic = self._declare_parameter_if_not_declared(
            'binary_state_topic', 'binary_state')
        self._flip_threshold = self._declare_parameter_if_not_declared('flip_threshold', 50.0)

        self._filter_info_sub = node.create_subscription(
            CostmapFilterInfo, self._filter_info_topic,
            self._filter_info_callback, _latched_qos())
        self._binary_state_pub = node.create_publisher(
            Bool, self._binary_state_topic, _latched_qos())

        self._base = BASE_DEFAULT
        self._multiplier = MULTIPLIER_DEFAULT
        self._binary_state = self._default_state
        self.change_state(self._binary_state)
        node.get_logger().info(
            f'[BinaryFilter] "{self._name}" subscribing to filter info '
            f'"{self._filter_info_topic}", publishing to "{self._binary_state_topic}"')

    def _filter_info_callback(self, msg: CostmapFilterInfo) -> None:
        """
        Read base/multiplier and subscribe to the announced mask topic.

        Parameters
        ----------
        msg : CostmapFilterInfo
            The filter info message.

        """
        if msg.type != BINARY_FILTER:
            self._node.get_logger().error(
                f'[BinaryFilter] Mode {msg.type} is not supported')
            return
        self._base = msg.base
        self._multiplier = msg.multiplier
        self._mask_topic = self.join_with_parent_namespace(msg.filter_mask_topic)
        self._mask_sub = self._node.create_subscription(
            OccupancyGrid, self._mask_topic, self._mask_callback, _latched_qos(3))
        self._node.get_logger().info(
            f'[BinaryFilter] "{self._name}" subscribing to mask "{self._mask_topic}"')

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
        Look up the mask value at the robot pose and flip the binary state.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap (unused, binary filters do not alter costs).
        min_i, min_j, max_i, max_j : int
            The bounds of the region to process (unused).
        pose : Any
            The robot's pose.

        """
        if self._filter_mask is None:
            self._node.get_logger().warning(
                '[BinaryFilter] Filter mask was not received',
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
            # Robot went out of mask range. Set "false" state by default.
            self._node.get_logger().warning(
                '[BinaryFilter] Robot is outside of filter mask. '
                'Resetting binary state to default.')
            self.change_state(self._default_state)
            return

        mask_data = self.get_mask_data(mask, mx, my)
        if mask_data == OCC_GRID_UNKNOWN:
            self._node.get_logger().warning(
                f'[BinaryFilter] Filter mask [{mx}, {my}] data is unknown. Do nothing.',
                throttle_duration_sec=2.0)
            return

        if self._base + mask_data * self._multiplier > self._flip_threshold:
            if self._binary_state == self._default_state:
                self.change_state(not self._default_state)
        else:
            if self._binary_state != self._default_state:
                self.change_state(self._default_state)

    def change_state(self, state: bool) -> None:
        """
        Change the binary state of the filter and publish the new state.

        Parameters
        ----------
        state : bool
            The new binary state.

        """
        self._binary_state = state
        self._node.get_logger().info(
            f'[BinaryFilter] Switched {"on" if state else "off"}')
        msg = Bool()
        msg.data = state
        if self._binary_state_pub is not None:
            self._binary_state_pub.publish(msg)

    def reset_filter(self) -> None:
        """Publish the current state and drop the filter subscriptions/publisher."""
        if self._binary_state_pub is not None:
            msg = Bool()
            msg.data = self._binary_state
            self._binary_state_pub.publish(msg)
        self._filter_info_sub = None
        self._mask_sub = None
        self._binary_state_pub = None

    def reset(self) -> None:
        """Reset the filter (mark it not current)."""
        self.set_current(False)

    def is_active(self) -> bool:
        """Return whether the mask has been received."""
        return self._filter_mask is not None
