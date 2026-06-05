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
from __future__ import annotations

import ast
import math
from typing import List, Tuple, TYPE_CHECKING

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D

if TYPE_CHECKING:
    # Imported only for type hints; importing at runtime would create a
    # circular import (layer imports LayeredCostmap under TYPE_CHECKING too).
    from nav2_costmap_2d_py.core.layer import Layer


class LayeredCostmap:
    """Instantiates layer plugins and aggregates them into one combined grid."""

    def __init__(
        self,
        global_frame: str,
        rolling_window: bool,
        track_unknown_space: bool,
    ) -> None:
        """
        Construct a layered costmap.

        Parameters
        ----------
        global_frame : str
            The global frame the costmap is expressed in.
        rolling_window : bool
            Whether the costmap is a rolling window that follows the robot.
        track_unknown_space : bool
            Whether unknown space is tracked (default value ``NO_INFORMATION``)
            or treated as free space (``FREE_SPACE``).

        """
        self._global_frame = global_frame
        self._rolling_window = rolling_window
        self._track_unknown_space = track_unknown_space

        default_value = NO_INFORMATION if track_unknown_space else FREE_SPACE
        self._combined_costmap = Costmap2D(default_value=default_value)

        self._plugins: List[Layer] = []
        self._filters: List[Layer] = []   # costmap filters

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

        self._footprint: List[Tuple[float, float]] = []

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
        """
        Resize the map to a new size, resolution, or origin and propagate to all layers.

        Parameters
        ----------
        size_x : int
            The new x size of the map in cells.
        size_y : int
            The new y size of the map in cells.
        resolution : float
            The new resolution of the map in meters/cell.
        origin_x : float
            The new x origin of the map.
        origin_y : float
            The new y origin of the map.
        size_locked : bool
            Whether the size of the costmap is locked against further resizing.

        """
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

    def add_plugin(self, plugin: 'Layer') -> None:
        """
        Add a new plugin to the plugins list to process.

        Parameters
        ----------
        plugin : Layer
            The layer plugin to add.

        """
        self._plugins.append(plugin)

    def add_filter(self, f: 'Layer') -> None:
        """
        Add a new costmap filter plugin to the filters list to process.

        Parameters
        ----------
        f : Layer
            The costmap filter layer to add.

        """
        self._filters.append(f)

    def get_plugins(self) -> List['Layer']:
        """Get the list of costmap plugins."""
        return self._plugins

    def get_filters(self) -> List['Layer']:
        """Get the list of costmap filters."""
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
        Update the underlying costmap with new data (run one costmap update cycle).

        Can be called outside the periodic update loop to force a refresh.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.

        Notes
        -----
        1. If rolling window, shift origin to keep robot centred.
        2. Reset combined costmap to default value.
        3. Call ``update_bounds`` on every plugin.
        4. Call ``update_costs`` on every plugin within bounds.
        5. Apply filters on top.

        """
        with self._combined_costmap.get_mutex():
            # ----- Rolling window origin shift -----
            # Only the combined costmap's
            # origin is rolled here. Each layer rolls its OWN costmap inside its
            # update_bounds (match_size is NOT called every cycle, which would
            # wipe the accumulated obstacle data).
            if self._rolling_window:
                new_origin_x = (
                    robot_x
                    - self._combined_costmap.size_x_meters / 2.0
                )
                new_origin_y = (
                    robot_y
                    - self._combined_costmap.size_y_meters / 2.0
                )
                self._combined_costmap.update_origin(new_origin_x, new_origin_y)

            # ----- Initialise bounding box to "nothing" -----
            _INF = float('inf')
            min_x = [_INF]
            min_y = [_INF]
            max_x = [-_INF]
            max_y = [-_INF]

            # ----- update_bounds on each plugin -----
            for plugin in self._plugins:
                plugin.update_bounds(
                    robot_x, robot_y, robot_yaw,
                    min_x, min_y, max_x, max_y
                )

            # ----- Clamp bounds to map extents and convert to cell indices -----
            if min_x[0] > max_x[0] or min_y[0] > max_y[0]:
                # No layer expanded the bounds → nothing to update
                self._initialized = True
                return

            x0, y0 = self._combined_costmap.world_to_map_enforce_bounds(
                min_x[0], min_y[0]
            )
            xn, yn = self._combined_costmap.world_to_map_enforce_bounds(
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
        Return whether the costmap is current.

        The costmap is current when all enabled layers are processing recent
        data and not stale information.
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
        """
        Check whether the robot is outside the bounds of its costmap.

        Useful for detecting poorly configured setups.

        Parameters
        ----------
        robot_x, robot_y : float
            The robot position in the global frame.

        Returns
        -------
        bool
            True if the robot lies outside the costmap bounds.

        """
        ok, _, _ = self._combined_costmap.world_to_map(robot_x, robot_y)
        return not ok

    # ------------------------------------------------------------------
    # Footprint
    # ------------------------------------------------------------------

    def set_footprint(self, footprint: List[Tuple[float, float]]) -> None:
        """
        Update the stored footprint, the circumscribed/inscribed radii, and all layers.

        Calls ``on_footprint_changed()`` (via ``set_footprint``) on every plugin
        and filter.

        Parameters
        ----------
        footprint : list of tuple of float
            The robot footprint as a list of ``(x, y)`` points.

        """
        self._footprint = footprint
        self.circumscribed_radius, self.inscribed_radius = (
            _compute_footprint_radii(footprint)
        )
        for plugin in self._plugins:
            plugin.set_footprint(footprint)
        for f in self._filters:
            f.set_footprint(footprint)

    def get_footprint(self) -> List[Tuple[float, float]]:
        """Return the latest footprint stored with ``set_footprint()``."""
        return self._footprint

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_costmap(self) -> Costmap2D:
        """Get the master (combined) costmap."""
        return self._combined_costmap

    def get_global_frame_id(self) -> str:
        """Get the global frame the costmap is expressed in."""
        return self._global_frame

    def is_rolling(self) -> bool:
        """Return whether this costmap is a rolling window."""
        return self._rolling_window

    def is_size_locked(self) -> bool:
        """Return whether the size of the costmap is locked."""
        return self._size_locked

    def is_initialized(self) -> bool:
        """Return whether the costmap is initialized."""
        return self._initialized

    def get_updated_bounds(self) -> Tuple[int, int, int, int]:
        """
        Get the bounds of the costmap region updated in the last cycle.

        Returns
        -------
        tuple of int
            ``(x0, y0, xn, yn)`` cell-index bounds of the last update.

        """
        return self._bx0, self._by0, self._bxn, self._byn


# ---------------------------------------------------------------------------
# Footprint helpers
# ---------------------------------------------------------------------------

def _compute_footprint_radii(
    footprint: List[Tuple[float, float]]
) -> Tuple[float, float]:
    """
    Compute the circumscribed and inscribed radii of a footprint polygon.

    Parameters
    ----------
    footprint : list of tuple of float
        The robot footprint as a list of ``(x, y)`` points.

    Returns
    -------
    tuple of float
        The ``(circumscribed_radius, inscribed_radius)`` of the footprint.

    """
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


def make_footprint_from_radius(radius: float) -> List[Tuple[float, float]]:
    """
    Build a circular footprint (16-point polygon) of the given radius.

    Parameters
    ----------
    radius : float
        The radius of the circular footprint, in metres.

    Returns
    -------
    list of tuple of float
        The footprint as a list of ``(x, y)`` points.

    """
    n = 16
    return [
        (radius * math.cos(2 * math.pi * i / n),
         radius * math.sin(2 * math.pi * i / n))
        for i in range(n)
    ]


def make_footprint_from_string(footprint_str: str) -> List[Tuple[float, float]]:
    """
    Parse a footprint string like ``"[[0.1,0.2],[-0.1,0.2],[-0.1,-0.2],[0.1,-0.2]]"``.

    Parameters
    ----------
    footprint_str : str
        The footprint expressed as a string of ``[[x, y], ...]`` points.

    Returns
    -------
    list of tuple of float
        The parsed footprint as ``(x, y)`` tuples, or an empty list on parse
        error.

    """
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


def _sign0(x: float) -> int:
    """Return -1, 0 or +1 with sign0(0) == 0."""
    if x < 0:
        return -1
    if x > 0:
        return 1
    return 0


def pad_footprint(
    footprint: List[Tuple[float, float]], padding: float
) -> List[Tuple[float, float]]:
    """
    Pad a footprint outward by *padding* metres along each axis.

    Each coordinate is pushed
    out by ``padding`` in the direction of its sign (axis-aligned), so a corner
    ``(x, y)`` becomes ``(x + sign0(x) * padding, y + sign0(y) * padding)``.

    Parameters
    ----------
    footprint : list of tuple of float
        The footprint to pad, as a list of ``(x, y)`` points.
    padding : float
        The amount to expand each point outward, in metres.

    Returns
    -------
    list of tuple of float
        The padded footprint.

    """
    return [
        (px + _sign0(px) * padding, py + _sign0(py) * padding)
        for px, py in footprint
    ]


def transform_footprint(
    x: float, y: float, theta: float,
    footprint: List[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    """
    Rotate and translate a footprint to the robot pose (x, y, theta).

    Parameters
    ----------
    x, y : float
        World position of the robot.
    theta : float
        Orientation of the robot, in radians.
    footprint : list of tuple of float
        The footprint in the robot frame, as ``(x, y)`` points.

    Returns
    -------
    list of tuple of float
        The footprint transformed into the world frame as ``(wx, wy)`` points.

    """
    cos_th = math.cos(theta)
    sin_th = math.sin(theta)
    return [
        (x + px * cos_th - py * sin_th,
         y + px * sin_th + py * cos_th)
        for px, py in footprint
    ]
