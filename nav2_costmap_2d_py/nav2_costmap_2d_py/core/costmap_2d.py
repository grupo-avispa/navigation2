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
It mirrors the nav2_costmap_2d::Costmap2D from the C++ implementation.
"""

import math
import threading
from typing import List, Tuple

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE


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
        """
        Construct a costmap.

        Parameters
        ----------
        size_x : int
            The x size of the map in cells.
        size_y : int
            The y size of the map in cells.
        resolution : float
            The resolution of the map in meters/cell.
        origin_x : float
            The x origin of the map.
        origin_y : float
            The y origin of the map.
        default_value : int
            Default background value used to fill the grid.

        """
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
        """
        Initialize the costmap data structure.

        Parameters
        ----------
        size_x : int
            The x size to use for map initialization.
        size_y : int
            The y size to use for map initialization.

        """
        self._size_x = size_x
        self._size_y = size_y
        self._costmap = bytearray(size_x * size_y)

    def reset_maps(self) -> None:
        """Reset the costmap, filling every cell with the default (unknown) value."""
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
        """
        Resize the costmap.

        Reinitializes the grid with the new dimensions and resets it to the
        default value.

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

        """
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
        Convert from world coordinates to map coordinates.

        Parameters
        ----------
        wx : float
            The x world coordinate.
        wy : float
            The y world coordinate.

        Returns
        -------
        (ok, mx, my)
            ``ok`` is True if the conversion was successful (legal bounds),
            False otherwise; ``mx``/``my`` are the associated map coordinates.

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
        Convert from world coordinates to map coordinates, constraining results to legal bounds.

        Parameters
        ----------
        wx : float
            The x world coordinate.
        wy : float
            The y world coordinate.

        Returns
        -------
        (mx, my)
            The associated map coordinates, guaranteed to lie within the map.

        """
        mx = int((wx - self._origin_x) / self._resolution)
        my = int((wy - self._origin_y) / self._resolution)
        mx = max(0, min(mx, self._size_x - 1))
        my = max(0, min(my, self._size_y - 1))
        return mx, my

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """
        Convert from map coordinates to world coordinates (cell centre).

        Parameters
        ----------
        mx : int
            The x map coordinate.
        my : int
            The y map coordinate.

        Returns
        -------
        (wx, wy)
            The associated world coordinates.

        """
        wx = self._origin_x + (mx + 0.5) * self._resolution
        wy = self._origin_y + (my + 0.5) * self._resolution
        return wx, wy

    # ------------------------------------------------------------------
    # Index helpers
    # ------------------------------------------------------------------

    def get_index(self, mx: int, my: int) -> int:
        """
        Given two map coordinates, compute the associated flat array index.

        Parameters
        ----------
        mx : int
            The x coordinate.
        my : int
            The y coordinate.

        Returns
        -------
        int
            The associated index.

        """
        return my * self._size_x + mx

    def index_to_cells(self, index: int) -> Tuple[int, int]:
        """
        Given an index, compute the associated map coordinates.

        Parameters
        ----------
        index : int
            The flat array index.

        Returns
        -------
        (mx, my)
            The associated map coordinates.

        """
        my = index // self._size_x
        mx = index % self._size_x
        return mx, my

    # ------------------------------------------------------------------
    # Cell access
    # ------------------------------------------------------------------

    def get_cost(self, mx: int, my: int) -> int:
        """
        Get the cost of a cell in the costmap.

        Parameters
        ----------
        mx : int
            The x coordinate of the cell.
        my : int
            The y coordinate of the cell.

        Returns
        -------
        int
            The cost of the cell.

        """
        return self._costmap[self.get_index(mx, my)]

    def set_cost(self, mx: int, my: int, cost: int) -> None:
        """
        Set the cost of a cell in the costmap.

        Parameters
        ----------
        mx : int
            The x coordinate of the cell.
        my : int
            The y coordinate of the cell.
        cost : int
            The cost to set the cell to.

        """
        self._costmap[self.get_index(mx, my)] = cost

    def get_char_map(self) -> bytearray:
        """
        Return the underlying array used as the costmap.

        Returns
        -------
        bytearray
            The underlying array storing the cost values.

        """
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
        Copy the ``(sx0, sy0)..(sxn, syn)`` window from a source costmap into this one.

        Parameters
        ----------
        source : Costmap2D
            Source costmap where the window will be copied from.
        sx0, sy0 : int
            Lower x/y boundary of the source window to copy, in cells.
        sxn, syn : int
            Upper x/y boundary of the source window to copy, in cells.
        dx0, dy0 : int
            Lower x/y boundary of the destination window to copy into, in cells.

        Returns
        -------
        bool
            True if the copy succeeded, False otherwise.

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
        """
        Reset the costmap within the given bounds to the default value.

        Parameters
        ----------
        x0, y0 : int
            Lower x/y boundary of the region to reset, in cells.
        xn, yn : int
            Upper x/y boundary of the region to reset, in cells.

        """
        for y in range(y0, yn):
            for x in range(x0, xn):
                self._costmap[self.get_index(x, y)] = self._default_value

    def move_map(self, new_origin_x: float, new_origin_y: float) -> None:
        """
        Move the origin of the costmap to a new location, keeping data where it can.

        Parameters
        ----------
        new_origin_x : float
            The x coordinate of the new origin.
        new_origin_y : float
            The y coordinate of the new origin.

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
        polygon: List[Tuple[float, float]],
        cost: int,
    ) -> bool:
        """
        Set the cost of a convex polygon to a desired value.

        Parameters
        ----------
        polygon : list of tuple of float
            The polygon to perform the operation on, as a list of ``(x, y)``
            world-coordinate tuples.
        cost : int
            The value to set the enclosed cells to.

        Returns
        -------
        bool
            True if the polygon was filled, False if it could not be filled.

        """
        if len(polygon) < 3:
            return False
        cells = self._rasterize_polygon(polygon)
        for mx, my in cells:
            if 0 <= mx < self._size_x and 0 <= my < self._size_y:
                self._costmap[self.get_index(mx, my)] = cost
        return True

    def _rasterize_polygon(
        self, polygon: List[Tuple[float, float]]
    ) -> List[Tuple[int, int]]:
        """
        Get the map cells that fill a convex polygon.

        Parameters
        ----------
        polygon : list of tuple of float
            The polygon to rasterize, as a list of ``(x, y)`` world-coordinate
            tuples.

        Returns
        -------
        list of tuple of int
            The map cells (``(mx, my)``) inside (and on the boundary of) the
            polygon.

        """
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

    def footprint_cost(
        self, x: float, y: float, theta: float,
        footprint: List[Tuple[float, float]],
    ) -> float:
        """
        Compute the maximum cost under a rotated/translated footprint.

        Parameters
        ----------
        x, y : float
            World position of the robot to place the footprint at.
        theta : float
            Orientation of the robot, in radians.
        footprint : list of tuple of float
            The robot footprint as a list of ``(x, y)`` points in the robot
            frame.

        Returns
        -------
        float
            The maximum cost found under the footprint, or ``-1.0`` if any cell
            is out of bounds.

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
        """Accessor for the x size of the costmap in cells."""
        return self._size_x

    @property
    def size_y(self) -> int:
        """Accessor for the y size of the costmap in cells."""
        return self._size_y

    @property
    def resolution(self) -> float:
        """Accessor for the resolution of the costmap in meters/cell."""
        return self._resolution

    @property
    def origin_x(self) -> float:
        """Accessor for the x origin of the costmap."""
        return self._origin_x

    @property
    def origin_y(self) -> float:
        """Accessor for the y origin of the costmap."""
        return self._origin_y

    @property
    def size_x_meters(self) -> float:
        """Accessor for the x size of the costmap in meters."""
        return self._size_x * self._resolution

    @property
    def size_y_meters(self) -> float:
        """Accessor for the y size of the costmap in meters."""
        return self._size_y * self._resolution

    @property
    def default_value(self) -> int:
        """Get the default background value of the costmap."""
        return self._default_value

    @default_value.setter
    def default_value(self, val: int) -> None:
        """Set the default background value of the costmap."""
        self._default_value = val

    def get_mutex(self) -> threading.RLock:
        """Return the recursive mutex guarding thread-safe access to the costmap."""
        return self._mutex

    def is_in_bounds(self, mx: int, my: int) -> bool:
        """Return whether the map coordinates ``(mx, my)`` lie within the costmap bounds."""
        return 0 <= mx < self._size_x and 0 <= my < self._size_y
