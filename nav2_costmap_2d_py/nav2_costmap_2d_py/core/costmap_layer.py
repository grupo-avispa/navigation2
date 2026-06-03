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

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer
import numpy as np


class CostmapLayer(Layer, Costmap2D):
    """Layer that owns an internal Costmap2D grid."""

    def __init__(self) -> None:
        """Initialize both parent Layer and Costmap2D instances."""
        Layer.__init__(self)
        Costmap2D.__init__(self)

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

    def clear_area(
        self,
        start_x: int, start_y: int,
        end_x: int, end_y: int,
        invert: bool = False,
    ) -> None:
        """
        Clear an area in the costmap with the given dimensions.

        Cells inside the rectangle are set to ``FREE_SPACE`` (cells holding
        ``NO_INFORMATION`` are left untouched). If *invert* is True, everything
        *except* the given dimensions is cleared instead.

        Parameters
        ----------
        start_x, start_y : int
            Lower x/y boundary of the rectangle, in cells.
        end_x, end_y : int
            Upper x/y boundary of the rectangle, in cells.
        invert : bool
            If True, clear everything outside the rectangle instead of inside.

        """
        self._current = False
        for x in range(self.size_x):
            xrange = start_x < x < end_x
            for y in range(self.size_y):
                in_region = xrange and start_y < y < end_y
                if in_region == invert:
                    continue
                idx = self.get_index(x, y)
                if self._costmap[idx] != NO_INFORMATION:
                    self._costmap[idx] = FREE_SPACE

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
