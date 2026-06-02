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
internal grid.  Adds ``match_size()``, ``touch()``, ``clear_area()``, and
the two ``update_with_*`` merge helpers.
It mirrors the nav2_costmap_2d::CostmapLayer from the C++ implementation.
"""

import numpy as np

from nav2_costmap_2d_py.core.layer import Layer
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, NO_INFORMATION


class CostmapLayer(Layer, Costmap2D):
    """
    Layer that owns an internal Costmap2D grid.
    """

    def __init__(self) -> None:
        Layer.__init__(self)
        Costmap2D.__init__(self)

    def match_size(self) -> None:
        """Resize this layer's internal grid to match the master costmap."""
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
        min_x: list, min_y: list, max_x: list, max_y: list,
    ) -> None:
        """Expand bounding box to include world point (x, y)."""
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
        Clear (set to FREE_SPACE) all cells in the given rectangle.

        If *invert* is True, clear everything OUTSIDE the rectangle instead.
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
        Copy cells to master only where this layer's cost > master's cost.

        NO_INFORMATION cells are skipped.
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
        Copy cells to master, skipping NO_INFORMATION.
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
        Copy ALL cells to master unconditionally (including NO_INFORMATION).
        """
        if not self._enabled:
            return
        if self._costmap is None:
            raise RuntimeError(
                "Can't update costmap layer: it hasn't been initialized yet!"
            )
        master, mine = self._region_views(master_grid, min_i, min_j, max_i, max_j)
        master[:] = mine

    def _region_views(self, master_grid, min_i, min_j, max_i, max_j):
        """
        Return writable NumPy views of the [min_j:max_j, min_i:max_i] window of
        the master grid and this layer's grid (which share the master's shape).

        Vectorising these merges is essential: the old per-cell Python loops ran
        over the whole map every update cycle and held the GIL long enough to
        freeze the whole process, starving the planner's action server (BT
        "timed out waiting ... to acknowledge goal request"). NumPy fancy-indexing
        runs in C and releases the GIL.
        """
        span = master_grid.size_x
        rows = master_grid.size_y
        master = np.frombuffer(
            master_grid.get_char_map(), dtype=np.uint8).reshape(rows, span)
        mine = np.frombuffer(self._costmap, dtype=np.uint8).reshape(rows, span)
        region = (slice(min_j, max_j), slice(min_i, max_i))
        return master[region], mine[region]
