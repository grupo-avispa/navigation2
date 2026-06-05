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
StaticLayer for nav2_costmap_2d_py.

Reads a static occupancy grid from the ``/map`` topic and writes it
into the master costmap.

It mirrors the nav2_costmap_2d::StaticLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/StaticLayer"``
"""

from typing import Any, List, Optional

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.layered_costmap import transform_footprint
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

_EPSILON = 1e-6


class StaticLayer(CostmapLayer):
    """
    Take in a map generated from SLAM to add costs to the costmap.

    Reads a static occupancy grid from the map topic and writes it into the
    master costmap.
    """

    def __init__(self) -> None:
        """Initialize static layer defaults."""
        super().__init__()
        self._map_topic = 'map'
        self._subscribe_to_updates = False
        self._track_unknown_space = True
        self._use_maximum = False
        self._lethal_threshold = 100
        self._unknown_cost_value: int = 255   # 0xff
        self._inscribed_obstacle_cost_value: int = 253
        self._trinary_costmap = True
        self._transform_tolerance = 0.0
        self._footprint_clearing_enabled = False
        self._restore_cleared_footprint = True

        self._global_frame = ''
        self._map_frame = ''
        self._transformed_footprint: List[tuple] = []

        self._map_sub: Optional[Any] = None
        self._map_update_sub: Optional[Any] = None
        self._has_updated_data = False
        self._map_received = False

        # Dirty region (cell indices)
        self._x: int = 0
        self._y: int = 0
        self._width: int = 0
        self._height: int = 0

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer on startup: read parameters and subscribe to the map topic."""
        node = self._node
        self._global_frame = self._layered_costmap.get_global_frame_id()

        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._subscribe_to_updates = self._declare_parameter_if_not_declared(
            'subscribe_to_updates', False)
        self._footprint_clearing_enabled = self._declare_parameter_if_not_declared(
            'footprint_clearing_enabled', False)
        self._restore_cleared_footprint = self._declare_parameter_if_not_declared(
            'restore_cleared_footprint', True)
        # Resolve the map topic against the parent namespace: the costmap node
        # is in the '/global_costmap' sub-namespace, so a relative 'map' would
        # become '/global_costmap/map' instead of the '/map' map_server uses.
        self._map_topic = self.join_with_parent_namespace(
            self._declare_parameter_if_not_declared('map_topic', 'map'))
        self._track_unknown_space = self._declare_parameter_if_not_declared(
            'track_unknown_space', True)
        self._use_maximum = self._declare_parameter_if_not_declared('use_maximum', False)
        temp_lethal_threshold = self._declare_parameter_if_not_declared(
            'lethal_cost_threshold', 100)
        self._lethal_threshold = max(min(int(temp_lethal_threshold), 100), 0)
        self._unknown_cost_value = int(self._declare_parameter_if_not_declared(
            'unknown_cost_value', 255))
        self._inscribed_obstacle_cost_value = int(self._declare_parameter_if_not_declared(
            'inscribed_obstacle_cost_value', 253))
        self._trinary_costmap = self._declare_parameter_if_not_declared('trinary_costmap', True)
        self._transform_tolerance = self._declare_parameter_if_not_declared(
            'transform_tolerance', 0.0)
        self._map_subscribe_transient_local = self._declare_parameter_if_not_declared(
            'map_subscribe_transient_local', True)

        # The map_server latches the map (publishes once, TRANSIENT_LOCAL).
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=(
                QoSDurabilityPolicy.TRANSIENT_LOCAL
                if self._map_subscribe_transient_local
                else QoSDurabilityPolicy.VOLATILE
            ),
        )
        self._map_sub = node.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._map_callback,
            map_qos,
        )

        if self._subscribe_to_updates:
            try:
                from map_msgs.msg import OccupancyGridUpdate
                self._map_update_sub = node.create_subscription(
                    OccupancyGridUpdate,
                    self._map_topic + '_updates',
                    self._map_update_callback,
                    1,
                )
            except ImportError:
                node.get_logger().warning(
                    '[StaticLayer] map_msgs not available; '
                    'subscribe_to_updates disabled.'
                )

        node.get_logger().info(
            f'[StaticLayer] "{self._name}" subscribing to "{self._map_topic}"'
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
        Update the bounds of the master costmap by this layer's update dimensions.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if not self._map_received:
            return
        if not self._layered_costmap.is_rolling():
            if not (self._has_updated_data or self._has_extra_bounds):
                return

        self.use_extra_bounds(min_x, min_y, max_x, max_y)

        if self._layered_costmap.is_rolling():
            master = self._layered_costmap.get_costmap()
            half_w = master.size_x_meters / 2.0
            half_h = master.size_y_meters / 2.0
            min_x[0] = min(robot_x - half_w, min_x[0])
            min_y[0] = min(robot_y - half_h, min_y[0])
            max_x[0] = max(robot_x + half_w, max_x[0])
            max_y[0] = max(robot_y + half_h, max_y[0])
        else:
            wx, wy = self.map_to_world(self._x, self._y)
            min_x[0] = min(wx, min_x[0])
            min_y[0] = min(wy, min_y[0])
            wx, wy = self.map_to_world(self._x + self._width, self._y + self._height)
            max_x[0] = max(wx, max_x[0])
            max_y[0] = max(wy, max_y[0])

        self._has_updated_data = False

        self.update_footprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y)

    def update_footprint(
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
        Expand the bounds to include the robot footprint (for footprint clearing).

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if not self._footprint_clearing_enabled:
            return
        self._transformed_footprint = transform_footprint(
            robot_x, robot_y, robot_yaw, self._layered_costmap.get_footprint())
        for (fx, fy) in self._transformed_footprint:
            self.touch(fx, fy, min_x, min_y, max_x, max_y)

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int
    ) -> None:
        """
        Update the costs in the master costmap within the given window.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to write the static map into.
        min_i, min_j : int
            Lower x/y boundary of the window to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to update, in cells.

        """
        if not self._enabled or not self._map_received:
            return

        # Clear the footprint region (cached for restoration after combining).
        map_region_to_restore: List[tuple] = []
        if self._footprint_clearing_enabled:
            ok, cells = self.get_map_region_occupied_by_polygon(self._transformed_footprint)
            if ok:
                map_region_to_restore = [
                    (mx, my, self.get_cost(mx, my)) for (mx, my) in cells]
                for (mx, my) in cells:
                    self.set_cost(mx, my, FREE_SPACE)

        if not self._layered_costmap.is_rolling():
            # Same coordinates as the master grid.
            if not self._use_maximum:
                self.update_with_true_overwrite(master_grid, min_i, min_j, max_i, max_j)
            else:
                self.update_with_max(master_grid, min_i, min_j, max_i, max_j)
        else:
            self._update_costs_rolling(master_grid, min_i, min_j, max_i, max_j)

        if self._footprint_clearing_enabled and self._restore_cleared_footprint:
            for (mx, my, cost) in map_region_to_restore:
                self.set_cost(mx, my, cost)

        self.set_current(True)

    def _update_costs_rolling(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int, max_i: int, max_j: int,
    ) -> None:
        """
        Combine the static map into a rolling master grid using the map<->global transform.

        Parameters
        ----------
        master_grid : Costmap2D
            The (rolling) master costmap to write into.
        min_i, min_j, max_i, max_j : int
            Cell-index bounds of the window to update.

        """
        if self._tf_buffer is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame, self._global_frame, Time(),
                timeout=Duration(seconds=self._transform_tolerance))
        except Exception as ex:  # noqa: BLE001
            self._node.get_logger().error(
                f'[StaticLayer] "{self._name}": {ex}',
                throttle_duration_sec=2.0)
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
                # master cell (i, j) -> global frame coordinates
                wx, wy = master_grid.map_to_world(i, j)
                # global frame -> map frame
                px = tx + r00 * wx + r01 * wy
                py = ty + r10 * wx + r11 * wy
                ok, mx, my = self.world_to_map(px, py)
                if not ok:
                    continue
                if not self._use_maximum:
                    master_grid.set_cost(i, j, self.get_cost(mx, my))
                else:
                    master_grid.set_cost(
                        i, j, max(self.get_cost(mx, my), master_grid.get_cost(i, j)))

    def reset(self) -> None:
        """Reset this costmap layer, forcing a re-publish on the next cycle."""
        self._has_updated_data = True   # Force re-publish on next cycle
        self.set_current(False)

    def is_clearable(self) -> bool:
        """
        Return whether clearing operations should be processed on this layer.

        Returns
        -------
        bool
            Always ``False``; the static layer is never cleared.

        """
        return False

    # ------------------------------------------------------------------
    # Map callbacks
    # ------------------------------------------------------------------

    def _map_callback(self, msg: OccupancyGrid) -> None:
        """
        Update the costmap's map from the map_server (full map callback).

        Parameters
        ----------
        msg : OccupancyGrid
            The full static map, resizing the layer (or the whole layered
            costmap) if its geometry changed.

        """
        node = self._node
        new_w = msg.info.width
        new_h = msg.info.height
        new_res = msg.info.resolution
        new_ox = msg.info.origin.position.x
        new_oy = msg.info.origin.position.y

        master = self._layered_costmap.get_costmap()
        rolling = self._layered_costmap.is_rolling()

        if not rolling and (
            master.size_x != new_w or master.size_y != new_h
            or abs(master.resolution - new_res) > _EPSILON
            or abs(master.origin_x - new_ox) > _EPSILON
            or abs(master.origin_y - new_oy) > _EPSILON
            or not self._layered_costmap.is_size_locked()
        ):
            # Non-rolling: resize the whole layered costmap (and all layers).
            node.get_logger().info(
                f'[StaticLayer] "{self._name}" resizing costmap '
                f'to {new_w}×{new_h} at {new_res:.4f} m/px'
            )
            self._layered_costmap.resize_map(
                new_w, new_h, new_res, new_ox, new_oy, size_locked=True
            )

        elif (new_w != self._size_x or new_h != self._size_y
              or abs(new_res - self._resolution) > _EPSILON
              or abs(new_ox - self._origin_x) > _EPSILON
              or abs(new_oy - self._origin_y) > _EPSILON):
            # Only update the size of the costmap stored locally in this layer.
            node.get_logger().info(
                f'[StaticLayer] "{self._name}" resizing static layer '
                f'to {new_w}×{new_h} at {new_res:.4f} m/px'
            )
            self.resize_map(new_w, new_h, new_res, new_ox, new_oy)

        # Copy map data
        for idx, val in enumerate(msg.data):
            self._costmap[idx] = self.interpret_value(val)

        self._map_frame = msg.header.frame_id
        self._x = 0
        self._y = 0
        self._width = self._size_x
        self._height = self._size_y
        self._map_received = True
        self._has_updated_data = True
        self.set_current(True)

    def _map_update_callback(self, msg: Any) -> None:
        """
        Update the costmap's map with an update in a particular area (partial update).

        Parameters
        ----------
        msg : map_msgs.msg.OccupancyGridUpdate
            The incremental map update message.

        """
        if not self._map_received:
            return
        min_x_c = msg.x
        min_y_c = msg.y
        for j in range(msg.height):
            for i in range(msg.width):
                mx = min_x_c + i
                my = min_y_c + j
                if 0 <= mx < self._size_x and 0 <= my < self._size_y:
                    idx = self.get_index(mx, my)
                    val = msg.data[j * msg.width + i]
                    self._costmap[idx] = self.interpret_value(val)
        self._x = min(self._x, min_x_c)
        self._y = min(self._y, min_y_c)
        self._width = max(
            self._x + self._width, min_x_c + msg.width
        ) - self._x
        self._height = max(
            self._y + self._height, min_y_c + msg.height
        ) - self._y
        self._has_updated_data = True

    # ------------------------------------------------------------------
    # Cost interpretation
    # ------------------------------------------------------------------

    def interpret_value(self, value: int) -> int:
        """
        Interpret a value from the static map and convert it into a cost.

        Parameters
        ----------
        value : int
            The raw OccupancyGrid value (``-1`` for unknown, otherwise 0-100).

        Returns
        -------
        int
            The corresponding costmap cost value.

        """
        value = value & 0xFF

        # check if the static value is above the unknown or lethal thresholds
        if self._track_unknown_space and value == self._unknown_cost_value:
            return NO_INFORMATION
        elif not self._track_unknown_space and value == self._unknown_cost_value:
            return FREE_SPACE
        elif value == self._inscribed_obstacle_cost_value:
            return INSCRIBED_INFLATED_OBSTACLE
        elif value >= self._lethal_threshold:
            return LETHAL_OBSTACLE
        elif self._trinary_costmap:
            return FREE_SPACE

        scale = float(value) / self._lethal_threshold
        return int(scale * LETHAL_OBSTACLE)
