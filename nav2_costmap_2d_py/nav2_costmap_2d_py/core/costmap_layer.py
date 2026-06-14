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
CostmapLayer for nav2_costmap_2d_py.

Intermediate abstract class for layers that maintain their own Costmap2D
internal grid.
It mirrors the nav2_costmap_2d::CostmapLayer from the C++ implementation.
"""

from typing import List, Tuple

from nav2_costmap_2d_py.core.cost_values import (
    CombinationMethod,
    INSCRIBED_INFLATED_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer
import numpy as np


class CostmapLayer(Layer, Costmap2D):
    """Layer that owns an internal Costmap2D grid."""

    def __init__(self) -> None:
        """Initialize both parent Layer and Costmap2D instances."""
        Layer.__init__(self)
        Costmap2D.__init__(self)
        self._has_extra_bounds = False
        self._extra_min_x = 1e6
        self._extra_min_y = 1e6
        self._extra_max_x = -1e6
        self._extra_max_y = -1e6

    def is_discretized(self) -> bool:
        """Return whether the layer is discretized (always True for costmap layers)."""
        return True

    def match_size(self) -> None:
        """Match the size of the master costmap, resizing this layer's internal grid."""
        master = self._layered_costmap.get_costmap()
        self.resize_map(
            master.size_x,
            master.size_y,
            master.resolution,
            master.origin_x,
            master.origin_y,
        )

    @staticmethod
    def touch(
        x: float, y: float,
        min_x: List[float], min_y: List[float],
        max_x: List[float], max_y: List[float],
    ) -> None:
        """
        Update the bounding box specified by the parameters to include the location (x, y).

        Parameters
        ----------
        x, y : float
            World location to include in the bounding box.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the bounding box corners, updated
            in place (used as mutable out-parameters).

        """
        min_x[0] = min(x, min_x[0])
        min_y[0] = min(y, min_y[0])
        max_x[0] = max(x, max_x[0])
        max_y[0] = max(y, max_y[0])

    @staticmethod
    def combination_method_from_int(value: int) -> CombinationMethod:
        """
        Convert an integer parameter value into a :class:`CombinationMethod`.

        Falls back to ``CombinationMethod.Max`` for unsupported values.

        Parameters
        ----------
        value : int
            The raw ``combination_method`` parameter value (0, 1 or 2).

        Returns
        -------
        CombinationMethod
            The matching combination method.

        """
        if value == 0:
            return CombinationMethod.Overwrite
        if value == 1:
            return CombinationMethod.Max
        if value == 2:
            return CombinationMethod.MaxWithoutUnknownOverwrite
        return CombinationMethod.Max

    def clear_area(
        self,
        start_x: int, start_y: int,
        end_x: int, end_y: int,
        invert: bool = False,
    ) -> None:
        """
        Clear an area in the costmap with the given dimensions.

        Cells inside the rectangle are reset to ``NO_INFORMATION``. If *invert*
        is True, everything *except* the given dimensions is cleared instead.

        Parameters
        ----------
        start_x, start_y : int
            Lower x/y boundary of the rectangle, in cells.
        end_x, end_y : int
            Upper x/y boundary of the rectangle, in cells.
        invert : bool
            If True, clear everything outside the rectangle instead of inside.

        """
        self.set_current(False)
        grid = self.get_char_map()

        size_x = self.size_x
        size_y = self.size_y

        start_x = min(max(start_x, 0), size_x)
        start_y = min(max(start_y, 0), size_y)
        end_x = min(max(end_x, 0), size_x)
        end_y = min(max(end_y, 0), size_y)

        for x in range(size_x):
            xrange = start_x < x < end_x
            for y in range(size_y):
                if (xrange and start_y < y < end_y) == invert:
                    continue
                index = self.get_index(x, y)
                if grid[index] != NO_INFORMATION:
                    grid[index] = NO_INFORMATION

    def add_extra_bounds(self, mx0: float, my0: float, mx1: float, my1: float) -> None:
        """
        Add an additional bounding box to be merged in on the next ``use_extra_bounds``.

        Parameters
        ----------
        mx0, my0 : float
            Lower x/y corner of the extra bounding box, in world coordinates.
        mx1, my1 : float
            Upper x/y corner of the extra bounding box, in world coordinates.

        """
        self._extra_min_x = min(mx0, self._extra_min_x)
        self._extra_max_x = max(mx1, self._extra_max_x)
        self._extra_min_y = min(my0, self._extra_min_y)
        self._extra_max_y = max(my1, self._extra_max_y)
        self._has_extra_bounds = True

    def use_extra_bounds(
        self,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """
        Merge any pending extra bounds into the given bounding box and clear them.

        Parameters
        ----------
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the bounding box corners, expanded in
            place (used as mutable out-parameters).

        """
        if not self._has_extra_bounds:
            return

        min_x[0] = min(self._extra_min_x, min_x[0])
        min_y[0] = min(self._extra_min_y, min_y[0])
        max_x[0] = max(self._extra_max_x, max_x[0])
        max_y[0] = max(self._extra_max_y, max_y[0])
        self._extra_min_x = 1e6
        self._extra_min_y = 1e6
        self._extra_max_x = -1e6
        self._extra_max_y = -1e6
        self._has_extra_bounds = False

    # ------------------------------------------------------------------
    # Merge helpers
    # ------------------------------------------------------------------

    def update_with_max(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int,
        max_i: int, max_j: int,
    ) -> None:
        """
        Update the master grid, taking the maximum of the master and layer values.

        Sets each master cell to the maximum of its current value and this
        layer's value. If the master value is ``NO_INFORMATION`` it is
        overwritten; if this layer's value is ``NO_INFORMATION`` the master
        value is left unchanged.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the bounding box to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the bounding box to update, in cells.

        """
        if not self._enabled:
            return
        master, mine = self._region_views(master_grid, min_i, min_j, max_i, max_j)
        valid = mine != NO_INFORMATION
        take = valid & ((master == NO_INFORMATION) | (master < mine))
        master[take] = mine[take]

    def update_with_max_without_unknown_overwrite(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Update the master grid with the maximum, but never overwrite unknown master cells.

        Like :meth:`update_with_max`, but if the master value is
        ``NO_INFORMATION`` it is left unchanged.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the bounding box to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the bounding box to update, in cells.

        """
        if not self._enabled:
            return
        master, mine = self._region_views(master_grid, min_i, min_j, max_i, max_j)
        valid = mine != NO_INFORMATION
        take = valid & (master != NO_INFORMATION) & (master < mine)
        master[take] = mine[take]

    def update_with_addition(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Update the master grid by adding this layer's values to it.

        Cells are summed and capped at ``INSCRIBED_INFLATED_OBSTACLE - 1``;
        ``NO_INFORMATION`` master cells are overwritten and ``NO_INFORMATION``
        layer cells are skipped.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the bounding box to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the bounding box to update, in cells.

        """
        if not self._enabled:
            return
        master, mine = self._region_views(master_grid, min_i, min_j, max_i, max_j)
        valid = mine != NO_INFORMATION
        # NO_INFORMATION master cells are overwritten with the layer value.
        overwrite = valid & (master == NO_INFORMATION)
        master[overwrite] = mine[overwrite]
        # Otherwise sum and clamp at INSCRIBED_INFLATED_OBSTACLE - 1.
        add = valid & (master != NO_INFORMATION)
        summed = master[add].astype(np.int32) + mine[add].astype(np.int32)
        summed = np.where(
            summed >= INSCRIBED_INFLATED_OBSTACLE,
            INSCRIBED_INFLATED_OBSTACLE - 1,
            summed,
        )
        master[add] = summed.astype(np.uint8)

    def update_with_overwrite(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int,
        max_i: int, max_j: int,
    ) -> None:
        """
        Update the master grid by overwriting it with this layer's valid values.

        Every valid value from this layer is written into the master grid;
        ``NO_INFORMATION`` cells are not copied.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the bounding box to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the bounding box to update, in cells.

        """
        if not self._enabled:
            return
        master, mine = self._region_views(master_grid, min_i, min_j, max_i, max_j)
        mask = mine != NO_INFORMATION
        master[mask] = mine[mask]

    def update_with_true_overwrite(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int,
        max_i: int, max_j: int,
    ) -> None:
        """
        Update the master grid, writing every value from this layer into it.

        TrueOverwrite means every value from this layer is written into the
        master grid, including ``NO_INFORMATION`` cells.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to update.
        min_i, min_j : int
            Lower x/y boundary of the bounding box to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the bounding box to update, in cells.

        """
        if not self._enabled:
            return
        if self._costmap is None:
            raise RuntimeError(
                "Can't update costmap layer: it hasn't been initialized yet!"
            )
        master, mine = self._region_views(master_grid, min_i, min_j, max_i, max_j)
        master[:] = mine

    def _region_views(
        self,
        master_grid: Costmap2D,
        min_i: int, min_j: int,
        max_i: int, max_j: int,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Return numpy views of the master and layer grids over the given bounding box.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap whose buffer is viewed.
        min_i, min_j : int
            Lower x/y boundary of the bounding box, in cells.
        max_i, max_j : int
            Upper x/y boundary of the bounding box, in cells.

        Returns
        -------
        tuple of numpy.ndarray
            The ``(master, layer)`` 2D views restricted to the bounding box.

        """
        span = master_grid.size_x
        rows = master_grid.size_y
        master = np.frombuffer(
            master_grid.get_char_map(), dtype=np.uint8).reshape(rows, span)
        mine = np.frombuffer(self._costmap, dtype=np.uint8).reshape(rows, span)
        region = (slice(min_j, max_j), slice(min_i, max_i))
        return master[region], mine[region]
