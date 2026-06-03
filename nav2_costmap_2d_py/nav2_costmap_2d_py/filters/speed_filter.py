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

Costmap filter that reads a speed-limit mask and publishes
``nav2_msgs/msg/SpeedLimit`` messages when the robot enters / leaves
a speed-limited zone.

It mirrors the nav2_costmap_2d::SpeedFilter from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/SpeedFilter"``
"""

from typing import Any, List, Optional

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid

# OccupancyGrid value that means "full speed"
_FULL_SPEED_ZONE = 0


class SpeedFilter(Layer):
    """
    Read in a speed restriction mask to dynamically adjust the robot's speed.

    Enables the robot to slow down in dangerous areas based on its pose in the
    map, publishing ``nav2_msgs/msg/SpeedLimit`` messages when the robot enters
    or leaves a speed-limited zone.
    """

    def __init__(self) -> None:
        """Initialize speed filter defaults."""
        super().__init__()
        self._speed_limit_topic = 'speed_limit'
        self._mask_topic = 'speed_filter_mask'
        self._percentage = True
        self._base_speed = 0.5

        self._mask: Optional[list] = None   # flat list of speed values (0-100 or -1)
        self._mask_width: int = 0
        self._mask_height: int = 0
        self._mask_resolution: float = 0.0
        self._mask_origin_x: float = 0.0
        self._mask_origin_y: float = 0.0

        self._mask_sub: Any = None
        # Any (not Optional): assigned a real publisher in on_initialize before
        # use; avoids a spurious mypy union-attr on .publish().
        self._speed_limit_pub: Any = None
        self._last_speed_limit: Optional[float] = None

    def on_initialize(self) -> None:
        """Initialize the filter: read params, subscribe to the mask and create the publisher."""
        node = self._node
        name = self._name

        def _p(param: str, default: Any) -> Any:
            full = f'{name}.{param}'
            if not node.has_parameter(full):
                node.declare_parameter(full, default)
            return node.get_parameter(full).value

        self._enabled = _p('enabled', True)
        self._speed_limit_topic = _p('speed_limit_topic', 'speed_limit')
        self._mask_topic = _p('mask_topic', 'speed_filter_mask')
        self._percentage = _p('percentage', True)
        self._base_speed = _p('base_speed', 0.5)

        self._mask_sub = node.create_subscription(
            OccupancyGrid,
            self._mask_topic,
            self._mask_callback,
            1,
        )

        self._speed_limit_pub = node.create_publisher(
            SpeedLimit,
            self._speed_limit_topic,
            1,
        )

        node.get_logger().info(
            f'[SpeedFilter] "{name}" subscribing to mask "{self._mask_topic}", '
            f'publishing to "{self._speed_limit_topic}"'
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
        Update the master costmap bounds (no-op: the speed filter does not change bounds).

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds (left
            unchanged by this filter).

        """
        # SpeedFilter does not modify the costmap bounds
        pass

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Process the speed filter at the robot's pose, publishing a speed limit on change.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap (not modified by this filter).
        min_i, min_j : int
            Lower x/y boundary of the window, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window, in cells.

        """
        if not self._enabled or self._mask is None:
            return

        # Get the robot pose from the LayeredCostmap → Costmap2DROS chain.
        # We use the master costmap origin + size to estimate robot position
        # as the centre of the map (fallback: we can't get robot pose here
        # without the node reference, so we use master costmap's origin).
        # Plugins that need robot pose should use self._node directly.
        try:
            robot_pose = self._node.get_robot_pose()  # type: ignore[union-attr]
        except AttributeError:
            return

        if robot_pose is None:
            return

        rx = robot_pose.pose.position.x
        ry = robot_pose.pose.position.y

        # Look up the mask value at (rx, ry)
        speed_limit = self._get_speed_at(rx, ry)
        if speed_limit is None:
            return

        # Publish only if changed
        if speed_limit != self._last_speed_limit:
            self._publish_speed_limit(speed_limit)
            self._last_speed_limit = speed_limit

        self._current = True

    def reset(self) -> None:
        """Reset the costmap filter, clearing the last published speed limit."""
        self._last_speed_limit = None
        self._current = False

    def is_current(self) -> bool:
        """
        Check whether the data is up to date.

        Returns
        -------
        bool
            Always ``True``; the speed filter is always considered current.

        """
        return True  # SpeedFilter is always considered current

    def _get_speed_at(self, wx: float, wy: float) -> Optional[float]:
        """
        Return the speed limit (fraction or absolute) at world position (wx, wy).

        Parameters
        ----------
        wx, wy : float
            World coordinates to query in the speed mask.

        Returns
        -------
        float or None
            The speed limit value, ``0.0`` when in a full-speed zone, or
            ``None`` if ``(wx, wy)`` is outside the mask.

        """
        if self._mask is None:
            return None

        mx = int((wx - self._mask_origin_x) / self._mask_resolution)
        my = int((wy - self._mask_origin_y) / self._mask_resolution)

        if not (0 <= mx < self._mask_width and 0 <= my < self._mask_height):
            return None

        val = self._mask[my * self._mask_width + mx]

        if val == -1 or val == _FULL_SPEED_ZONE:
            # Outside speed zone: restore full speed
            return 0.0

        # OccupancyGrid value 1-100 → speed limit
        if self._percentage:
            return float(val)          # percentage of max speed
        else:
            return self._base_speed * (1.0 - val / 100.0)

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
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._layered_costmap.get_global_frame_id()
        msg.percentage = self._percentage
        msg.speed_limit = speed_limit
        self._speed_limit_pub.publish(msg)

    def _mask_callback(self, msg: OccupancyGrid) -> None:
        """
        Handle the filter mask: store the incoming speed-restriction OccupancyGrid.

        Parameters
        ----------
        msg : OccupancyGrid
            The incoming speed-restriction mask.

        """
        self._mask_width = msg.info.width
        self._mask_height = msg.info.height
        self._mask_resolution = msg.info.resolution
        self._mask_origin_x = msg.info.origin.position.x
        self._mask_origin_y = msg.info.origin.position.y
        self._mask = list(msg.data)

        self._node.get_logger().debug(
            f'[SpeedFilter] "{self._name}" received mask '
            f'{self._mask_width}×{self._mask_height} @ '
            f'{self._mask_resolution:.4f} m/px'
        )
