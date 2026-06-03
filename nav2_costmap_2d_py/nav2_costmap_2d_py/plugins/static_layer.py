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

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

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
        self._trinary_costmap = True
        self._transform_tolerance = 0.0

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
        name = self._name

        def _p(param: str, default: Any) -> Any:
            full = f'{name}.{param}'
            if not node.has_parameter(full):
                node.declare_parameter(full, default)
            return node.get_parameter(full).value

        self._enabled = _p('enabled', True)
        # Resolve the map topic against the parent namespace: the costmap node
        # is in the '/global_costmap' sub-namespace, so a relative 'map' would
        # become '/global_costmap/map' instead of the '/map' map_server uses.
        self._map_topic = self.join_with_parent_namespace(_p('map_topic', 'map'))
        self._subscribe_to_updates = _p('subscribe_to_updates', False)
        self._track_unknown_space = _p('track_unknown_space', True)
        self._use_maximum = _p('use_maximum', False)
        self._lethal_threshold = _p('lethal_cost_threshold', 100)
        unknown_raw = _p('unknown_cost_value', -1)
        self._unknown_cost_value = 255 if unknown_raw == -1 else int(unknown_raw)
        self._trinary_costmap = _p('trinary_costmap', True)
        self._transform_tolerance = _p('transform_tolerance', 0.0)
        self._map_subscribe_transient_local = _p(
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
            f'[StaticLayer] "{name}" subscribing to "{self._map_topic}"'
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
        if not self._enabled:
            return
        if not self._has_updated_data:
            return

        master = self._layered_costmap.get_costmap()
        wx0, wy0 = master.map_to_world(self._x, self._y)
        wx1, wy1 = master.map_to_world(
            self._x + self._width - 1,
            self._y + self._height - 1,
        )
        self.touch(wx0, wy0, min_x, min_y, max_x, max_y)
        self.touch(wx1, wy1, min_x, min_y, max_x, max_y)

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

        if self._use_maximum:
            self.update_with_max(master_grid, min_i, min_j, max_i, max_j)
        else:
            self.update_with_true_overwrite(master_grid, min_i, min_j, max_i, max_j)

        self._has_updated_data = False

    def reset(self) -> None:
        """Reset this costmap layer, forcing a re-publish on the next cycle."""
        self._has_updated_data = True   # Force re-publish on next cycle

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

        if (not self._map_received
                or abs(new_res - master.resolution) > _EPSILON):
            # First map or resolution changed → resize the whole layered costmap
            self._layered_costmap.resize_map(
                new_w, new_h, new_res, new_ox, new_oy, size_locked=True
            )

        elif (new_w != self._size_x or new_h != self._size_y
              or abs(new_ox - self._origin_x) > _EPSILON
              or abs(new_oy - self._origin_y) > _EPSILON):
            # Size/origin changed → resize just this layer
            node.get_logger().info(
                f'[StaticLayer] "{self._name}" resizing static layer '
                f'to {new_w}×{new_h} at {new_res:.4f} m/px'
            )
            self.resize_map(new_w, new_h, new_res, new_ox, new_oy)

        # Copy map data
        for idx, val in enumerate(msg.data):
            self._costmap[idx] = self._interpret_value(val)

        self._x = 0
        self._y = 0
        self._width = new_w
        self._height = new_h
        self._map_received = True
        self._has_updated_data = True
        self._current = True

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
                    self._costmap[idx] = self._interpret_value(val)
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

    def _interpret_value(self, value: int) -> int:
        """
        Interpret a value from the static map (0..100, -1) to convert it into a cost.

        Parameters
        ----------
        value : int
            The raw OccupancyGrid value (``-1`` for unknown, otherwise 0-100).

        Returns
        -------
        int
            The corresponding costmap cost value.

        """
        if value == -1:
            # Unknown
            return NO_INFORMATION if self._track_unknown_space else FREE_SPACE
        if value >= self._lethal_threshold:
            return LETHAL_OBSTACLE
        if self._trinary_costmap:
            return FREE_SPACE
        # Analogue mapping
        scale = float(value) / self._lethal_threshold
        return int(scale * LETHAL_OBSTACLE)
