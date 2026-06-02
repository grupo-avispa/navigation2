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
Costmap2D for nav2_costmap_2d_py.

Pure-Python 2D costmap grid.
The grid is stored as a flat ``bytearray`` of length ``size_x * size_y``.
Cell (mx, my) maps to index ``my * size_x + mx``.

Coordinates
-----------
* World (wx, wy)  - metres in the global frame.
* Map   (mx, my)  - integer cell indices (0 … size_x/y - 1).
* Index           - flat array offset = my * size_x + mx.

It mirrors the nav2_costmap_2d::Costmap2D from the C++ implementation.
"""

import math
import threading
from typing import Tuple

from nav2_costmap_2d_py.cost_values import FREE_SPACE


class Costmap2D:
    """A 2D costmap provides a mapping between points in the world and their associated "costs"."""

    def __init__(
        self,
        size_x: int = 0,
        size_y: int = 0,
        resolution: float = 0.05,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        default_value: int = FREE_SPACE,
    ) -> None:
        self._mutex = threading.RLock()
        self._resolution = resolution
        self._origin_x = origin_x
        self._origin_y = origin_y
        self._default_value = default_value
        self._size_x = 0
        self._size_y = 0
        self._costmap: bytearray = bytearray()
        if size_x > 0 and size_y > 0:
            self._init_maps(size_x, size_y)
            self.reset_maps()

    # ------------------------------------------------------------------
    # Initialization helpers
    # ------------------------------------------------------------------

    def _init_maps(self, size_x: int, size_y: int) -> None:
        self._size_x = size_x
        self._size_y = size_y
        self._costmap = bytearray(size_x * size_y)

    def reset_maps(self) -> None:
        """Fill every cell with the default value."""
        for i in range(len(self._costmap)):
            self._costmap[i] = self._default_value

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
    ) -> None:
        """Resize (and reinitialize) the costmap."""
        with self._mutex:
            self._resolution = resolution
            self._origin_x = origin_x
            self._origin_y = origin_y
            self._init_maps(size_x, size_y)
            self.reset_maps()

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def world_to_map(self, wx: float, wy: float) -> Tuple[bool, int, int]:
        """
        Convert world coordinates to map cell indices.

        Returns
        -------
        (ok, mx, my)
            ok is False if the world point is outside the map bounds.
        """
        if wx < self._origin_x or wy < self._origin_y:
            return False, 0, 0
        mx = int((wx - self._origin_x) / self._resolution)
        my = int((wy - self._origin_y) / self._resolution)
        if mx >= self._size_x or my >= self._size_y:
            return False, 0, 0
        return True, mx, my

    def world_to_map_enforced_bounds(self, wx: float, wy: float) -> Tuple[int, int]:
        """
        Convert world → map, clamping to valid bounds.
        """
        mx = int((wx - self._origin_x) / self._resolution)
        my = int((wy - self._origin_y) / self._resolution)
        mx = max(0, min(mx, self._size_x - 1))
        my = max(0, min(my, self._size_y - 1))
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map cell centre to world coordinates."""
        wx = self._origin_x + (mx + 0.5) * self._resolution
        wy = self._origin_y + (my + 0.5) * self._resolution
        return wx, wy

    # ------------------------------------------------------------------
    # Index helpers
    # ------------------------------------------------------------------

    def get_index(self, mx: int, my: int) -> int:
        return my * self._size_x + mx

    def index_to_cells(self, index: int) -> Tuple[int, int]:
        my = index // self._size_x
        mx = index % self._size_x
        return mx, my

    # ------------------------------------------------------------------
    # Cell access
    # ------------------------------------------------------------------

    def get_cost(self, mx: int, my: int) -> int:
        return self._costmap[self.get_index(mx, my)]

    def set_cost(self, mx: int, my: int, cost: int) -> None:
        self._costmap[self.get_index(mx, my)] = cost

    def get_char_map(self) -> bytearray:
        """Return a reference to the raw costmap array."""
        return self._costmap

    # ------------------------------------------------------------------
    # Map-wide operations
    # ------------------------------------------------------------------

    def copy_window(
        self,
        source: 'Costmap2D',
        sx0: int, sy0: int, sxn: int, syn: int,
        dx0: int, dy0: int,
    ) -> bool:
        """
        Copy a rectangular window from *source* into this costmap.
        """
        win_w = sxn - sx0
        win_h = syn - sy0
        if (sx0 < 0 or sy0 < 0 or sxn > source._size_x or syn > source._size_y
                or dx0 < 0 or dy0 < 0
                or dx0 + win_w > self._size_x
                or dy0 + win_h > self._size_y):
            return False
        src = source._costmap
        dst = self._costmap
        for row in range(win_h):
            s_off = (sy0 + row) * source._size_x + sx0
            d_off = (dy0 + row) * self._size_x + dx0
            dst[d_off:d_off + win_w] = src[s_off:s_off + win_w]
        return True

    def reset_map(self, x0: int, y0: int, xn: int, yn: int) -> None:
        """Reset a rectangular region to the default value."""
        for y in range(y0, yn):
            for x in range(x0, xn):
                self._costmap[self.get_index(x, y)] = self._default_value

    def move_map(self, new_origin_x: float, new_origin_y: float) -> None:
        """
        Shift the map origin, keeping valid data where possible.
        """
        cell_ox = int((new_origin_x - self._origin_x) / self._resolution)
        cell_oy = int((new_origin_y - self._origin_y) / self._resolution)

        if cell_ox == 0 and cell_oy == 0:
            return

        old_map = bytearray(self._costmap)
        self.reset_maps()

        for ny in range(self._size_y):
            oy = ny - cell_oy
            if oy < 0 or oy >= self._size_y:
                continue
            for nx in range(self._size_x):
                ox = nx - cell_ox
                if ox < 0 or ox >= self._size_x:
                    continue
                self._costmap[self.get_index(nx, ny)] = old_map[self.get_index(ox, oy)]

        self._origin_x = new_origin_x
        self._origin_y = new_origin_y

    def set_convex_polygon_cost(
        self,
        polygon: list,   # list of (x, y) world tuples
        cost: int,
    ) -> bool:
        """
        Set all cells inside a convex polygon to *cost*.
        """
        if len(polygon) < 3:
            return False
        cells = self._rasterize_polygon(polygon)
        for mx, my in cells:
            if 0 <= mx < self._size_x and 0 <= my < self._size_y:
                self._costmap[self.get_index(mx, my)] = cost
        return True

    def _rasterize_polygon(self, polygon: list) -> list:
        """Return all map cells inside (and on the boundary of) a polygon."""
        pts = []
        for wx, wy in polygon:
            ok, mx, my = self.world_to_map(wx, wy)
            if ok:
                pts.append((mx, my))
        if not pts:
            return []

        min_y = min(p[1] for p in pts)
        max_y = max(p[1] for p in pts)
        cells = []
        n = len(pts)
        for y in range(min_y, max_y + 1):
            xs = []
            for i in range(n):
                x1, y1 = pts[i]
                x2, y2 = pts[(i + 1) % n]
                if (y1 <= y < y2) or (y2 <= y < y1):
                    if y2 != y1:
                        xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                        xs.append(int(xi))
            xs.sort()
            for k in range(0, len(xs) - 1, 2):
                for x in range(xs[k], xs[k + 1] + 1):
                    cells.append((x, y))
        return cells

    def footprint_cost(self, x: float, y: float, theta: float, footprint: list) -> float:
        """
        Compute the maximum cost under a rotated/translated footprint.

        Returns -1.0 if any cell is out of bounds.
        """
        cos_th = math.cos(theta)
        sin_th = math.sin(theta)
        max_cost = 0.0
        for px, py in footprint:
            wx = x + px * cos_th - py * sin_th
            wy = y + px * sin_th + py * cos_th
            ok, mx, my = self.world_to_map(wx, wy)
            if not ok:
                return -1.0
            c = self.get_cost(mx, my)
            if c > max_cost:
                max_cost = c
        return max_cost

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def size_x(self) -> int:
        return self._size_x

    @property
    def size_y(self) -> int:
        return self._size_y

    @property
    def resolution(self) -> float:
        return self._resolution

    @property
    def origin_x(self) -> float:
        return self._origin_x

    @property
    def origin_y(self) -> float:
        return self._origin_y

    @property
    def size_x_meters(self) -> float:
        return self._size_x * self._resolution

    @property
    def size_y_meters(self) -> float:
        return self._size_y * self._resolution

    @property
    def default_value(self) -> int:
        return self._default_value

    @default_value.setter
    def default_value(self, val: int) -> None:
        self._default_value = val

    def get_mutex(self) -> threading.Lock:
        return self._mutex

    def is_in_bounds(self, mx: int, my: int) -> bool:
        return 0 <= mx < self._size_x and 0 <= my < self._size_y
