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
LayeredCostmap for nav2_costmap_2d_py.
Aggregates a list of :class:`Layer` plugins into a single combined ``Costmap2D``.
It mirrors the nav2_costmap_2d::LayeredCostmap from the C++ implementation.
"""

import math
import threading
from typing import List, Optional

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.cost_values import NO_INFORMATION, FREE_SPACE


class LayeredCostmap:
    """
    Instantiates layer plugins and aggregates them into one combined grid.
    """

    def __init__(
        self,
        global_frame: str,
        rolling_window: bool,
        track_unknown_space: bool,
    ) -> None:
        self._global_frame = global_frame
        self._rolling_window = rolling_window
        self._track_unknown_space = track_unknown_space

        default_value = NO_INFORMATION if track_unknown_space else FREE_SPACE
        self._combined_costmap = Costmap2D(default_value=default_value)

        self._plugins: List = []   # List[Layer]
        self._filters: List = []   # List[Layer] (costmap filters)

        self._initialized = False
        self._current = False
        self._size_locked = False

        # Bounding box of the last update cycle
        self._bx0 = 0
        self._bxn = 0
        self._by0 = 0
        self._byn = 0

        # Circumscribed / inscribed radii (set by set_footprint)
        self.circumscribed_radius: float = 0.0
        self.inscribed_radius: float = 0.0

        self._footprint: list = []

    # ------------------------------------------------------------------
    # Resize
    # ------------------------------------------------------------------

    def resize_map(
        self,
        size_x: int,
        size_y: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        size_locked: bool = False,
    ) -> None:
        """Resize the combined costmap and propagate to all layers."""
        with self._combined_costmap.get_mutex():
            self._size_locked = size_locked
            self._combined_costmap.resize_map(
                size_x, size_y, resolution, origin_x, origin_y
            )
            for plugin in self._plugins:
                plugin.match_size()
            for f in self._filters:
                f.match_size()

    # ------------------------------------------------------------------
    # Plugin / filter management
    # ------------------------------------------------------------------

    def add_plugin(self, plugin) -> None:
        """Add a layer plugin."""
        self._plugins.append(plugin)

    def add_filter(self, f) -> None:
        """Add a filter layer."""
        self._filters.append(f)

    def get_plugins(self) -> list:
        return self._plugins

    def get_filters(self) -> list:
        return self._filters

    # ------------------------------------------------------------------
    # updateMap
    # ------------------------------------------------------------------

    def update_map(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> None:
        """
        Run one costmap update cycle.

        1. If rolling window, shift origin to keep robot centred.
        2. Reset combined costmap to default value.
        3. Call ``update_bounds`` on every plugin.
        4. Call ``update_costs`` on every plugin within bounds.
        5. Apply filters on top.
        """
        with self._combined_costmap.get_mutex():
            # ----- Rolling window origin shift -----
            if self._rolling_window:
                new_origin_x = (
                    robot_x
                    - self._combined_costmap.size_x_meters / 2.0
                )
                new_origin_y = (
                    robot_y
                    - self._combined_costmap.size_y_meters / 2.0
                )
                self._combined_costmap.move_map(new_origin_x, new_origin_y)
                for plugin in self._plugins:
                    plugin.match_size()
                for f in self._filters:
                    f.match_size()

            # ----- Initialise bounding box to "nothing" -----
            _INF = float('inf')
            min_x = [_INF]
            min_y = [_INF]
            max_x = [-_INF]
            max_y = [-_INF]

            # ----- update_bounds on each plugin -----
            for plugin in self._plugins:
                plugin.update_bounds(robot_x, robot_y, robot_yaw,
                                     min_x, min_y, max_x, max_y)

            # ----- Clamp bounds to map extents and convert to cell indices -----
            if min_x[0] > max_x[0] or min_y[0] > max_y[0]:
                # No layer expanded the bounds → nothing to update
                self._initialized = True
                return

            x0, y0 = self._combined_costmap.world_to_map_enforced_bounds(
                min_x[0], min_y[0]
            )
            xn, yn = self._combined_costmap.world_to_map_enforced_bounds(
                max_x[0], max_y[0]
            )
            xn = min(xn + 1, self._combined_costmap.size_x)
            yn = min(yn + 1, self._combined_costmap.size_y)

            # ----- Reset dirty region to default value -----
            self._combined_costmap.reset_map(x0, y0, xn, yn)

            # ----- update_costs on each plugin -----
            for plugin in self._plugins:
                plugin.update_costs(self._combined_costmap, x0, y0, xn, yn)

            # ----- Apply filters after plugins -----
            for f in self._filters:
                f.update_costs(self._combined_costmap, x0, y0, xn, yn)

            self._bx0 = x0
            self._bxn = xn
            self._by0 = y0
            self._byn = yn
            self._initialized = True

    # ------------------------------------------------------------------
    # isCurrent
    # ------------------------------------------------------------------

    def is_current(self) -> bool:
        """
        Return True if ALL enabled layers report current data.
        """
        self._current = True
        for plugin in self._plugins:
            self._current = self._current and (
                plugin.is_current() or not plugin.is_enabled()
            )
        for f in self._filters:
            self._current = self._current and (
                f.is_current() or not f.is_enabled()
            )
        return self._current

    # ------------------------------------------------------------------
    # isOutofBounds
    # ------------------------------------------------------------------

    def is_out_of_bounds(self, robot_x: float, robot_y: float) -> bool:
        ok, _, _ = self._combined_costmap.world_to_map(robot_x, robot_y)
        return not ok

    # ------------------------------------------------------------------
    # Footprint
    # ------------------------------------------------------------------

    def set_footprint(self, footprint: list) -> None:
        """
        Set the robot footprint and propagate to all layers.

        Computes circumscribed and inscribed radii.
        """
        self._footprint = footprint
        self.circumscribed_radius, self.inscribed_radius = (
            _compute_footprint_radii(footprint)
        )
        for plugin in self._plugins:
            plugin.set_footprint(footprint)
        for f in self._filters:
            f.set_footprint(footprint)

    def get_footprint(self) -> list:
        return self._footprint

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_costmap(self) -> Costmap2D:
        return self._combined_costmap

    def get_global_frame_id(self) -> str:
        return self._global_frame

    def is_rolling_window(self) -> bool:
        return self._rolling_window

    def is_size_locked(self) -> bool:
        return self._size_locked

    def is_initialized(self) -> bool:
        return self._initialized

    def get_updated_bounds(self):
        return self._bx0, self._by0, self._bxn, self._byn


# ---------------------------------------------------------------------------
# Footprint helpers
# ---------------------------------------------------------------------------

def _compute_footprint_radii(footprint: list):
    """Return (circumscribed_radius, inscribed_radius) for a footprint polygon."""
    import math
    if not footprint:
        return 0.0, 0.0

    circumscribed = 0.0
    inscribed = float('inf')

    n = len(footprint)
    for i, (px, py) in enumerate(footprint):
        r = math.sqrt(px * px + py * py)
        circumscribed = max(circumscribed, r)

        # Inscribed = min distance from origin to each edge
        x2, y2 = footprint[(i + 1) % n]
        # distance from origin to segment (px,py)-(x2,y2)
        dx = x2 - px
        dy = y2 - py
        len_sq = dx * dx + dy * dy
        if len_sq > 0:
            t = max(0.0, min(1.0, (-px * dx - py * dy) / len_sq))
            cx = px + t * dx
            cy = py + t * dy
            d = math.sqrt(cx * cx + cy * cy)
            inscribed = min(inscribed, d)

    if inscribed == float('inf'):
        inscribed = circumscribed

    return circumscribed, inscribed


def make_footprint_from_radius(radius: float) -> list:
    """
    Build a circular footprint (16-point polygon) of given radius.
    """
    import math
    n = 16
    return [
        (radius * math.cos(2 * math.pi * i / n),
         radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def make_footprint_from_string(footprint_str: str) -> list:
    """
    Parse a footprint string like ``"[[0.1,0.2],[-0.1,0.2],[-0.1,-0.2],[0.1,-0.2]]"``.

    Returns a list of (x, y) tuples, or an empty list on parse error.
    """
    import ast
    try:
        data = ast.literal_eval(footprint_str.strip())
        if not isinstance(data, (list, tuple)):
            return []
        result = []
        for pt in data:
            if isinstance(pt, (list, tuple)) and len(pt) == 2:
                result.append((float(pt[0]), float(pt[1])))
        return result
    except Exception:
        return []


def pad_footprint(footprint: list, padding: float) -> list:
    """
    Expand footprint outward by *padding* metres.
    """
    import math
    if not footprint or padding <= 0:
        return list(footprint)
    cx = sum(p[0] for p in footprint) / len(footprint)
    cy = sum(p[1] for p in footprint) / len(footprint)
    result = []
    for px, py in footprint:
        dx = px - cx
        dy = py - cy
        length = math.sqrt(dx * dx + dy * dy)
        if length > 0:
            result.append((px + padding * dx / length,
                           py + padding * dy / length))
        else:
            result.append((px, py))
    return result


def transform_footprint(
    x: float, y: float, theta: float, footprint: list
) -> list:
    """
    Rotate and translate a footprint to the robot pose (x, y, theta).

    Returns list of (wx, wy) world-frame points.
    """
    import math
    cos_th = math.cos(theta)
    sin_th = math.sin(theta)
    return [
        (x + px * cos_th - py * sin_th,
         y + px * sin_th + py * cos_th)
        for px, py in footprint
    ]
