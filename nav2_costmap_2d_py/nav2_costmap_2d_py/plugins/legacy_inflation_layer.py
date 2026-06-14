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
LegacyInflationLayer for nav2_costmap_2d_py.

Layer to convolve the costmap by the robot's radius or footprint using a
BFS-based distance/cost cache (the legacy inflation algorithm).

It mirrors the nav2_costmap_2d::LegacyInflationLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/LegacyInflationLayer"``
"""

import math
import threading
from typing import Any, List

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.plugins.inflation_layer_interface import InflationLayerInterface


class CellData:
    """Storage for cell information used during obstacle inflation."""

    def __init__(self, x: int, y: int, sx: int, sy: int) -> None:
        """
        Construct a CellData object.

        Parameters
        ----------
        x : int
            The x coordinate of the cell in the costmap.
        y : int
            The y coordinate of the cell in the costmap.
        sx : int
            The x coordinate of the closest obstacle cell in the costmap.
        sy : int
            The y coordinate of the closest obstacle cell in the costmap.

        """
        self.x = x
        self.y = y
        self.src_x = sx
        self.src_y = sy


class LegacyInflationLayer(InflationLayerInterface):
    """Layer to convolve the costmap by the robot's radius using a BFS cache."""

    def __init__(self) -> None:
        """Initialize legacy inflation layer defaults."""
        super().__init__()
        self._access = threading.RLock()
        self._inflation_radius = 0.0
        self._inscribed_radius = 0.0
        self._cost_scaling_factor = 0.0
        self._inflate_unknown = False
        self._inflate_around_unknown = False
        self._cell_inflation_radius = 0
        self._cached_cell_inflation_radius = 0
        self._inflation_cells: List[List[CellData]] = []
        self._resolution = 0.0
        self._seen: List[bool] = []
        self._cached_costs: List[int] = []
        self._cached_distances: List[float] = []
        self._distance_matrix: List[List[int]] = []
        self._cache_length = 0
        self._max_dist = 0
        self._last_min_x = -float('inf')
        self._last_min_y = -float('inf')
        self._last_max_x = float('inf')
        self._last_max_y = float('inf')
        self._need_reinflation = False

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer on startup: read the inflation parameters."""
        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._inflation_radius = self._declare_parameter_if_not_declared(
            'inflation_radius', 0.55)
        self._cost_scaling_factor = self._declare_parameter_if_not_declared(
            'cost_scaling_factor', 10.0)
        self._inflate_unknown = self._declare_parameter_if_not_declared(
            'inflate_unknown', False)
        self._inflate_around_unknown = self._declare_parameter_if_not_declared(
            'inflate_around_unknown', False)

        self._current = True
        self._seen = []
        self._cached_distances = []
        self._cached_costs = []
        self._need_reinflation = False
        self._cell_inflation_radius = self.cell_distance(self._inflation_radius)
        self.match_size()

    def match_size(self) -> None:
        """Match the size of the master costmap and recompute the caches."""
        with self._access:
            costmap = self._layered_costmap.get_costmap()
            self._resolution = costmap.resolution
            self._cell_inflation_radius = self.cell_distance(self._inflation_radius)
            self.compute_caches()
            self._seen = [False] * (costmap.size_x * costmap.size_y)

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
        with self._access:
            if self._need_reinflation:
                self._last_min_x = float('inf')
                self._last_min_y = float('inf')
                self._last_max_x = -float('inf')
                self._last_max_y = -float('inf')
                min_x[0] = -float('inf')
                min_y[0] = -float('inf')
                max_x[0] = float('inf')
                max_y[0] = float('inf')
                self._need_reinflation = False
            else:
                tmp_min_x = self._last_min_x
                tmp_min_y = self._last_min_y
                tmp_max_x = self._last_max_x
                tmp_max_y = self._last_max_y
                self._last_min_x = min_x[0]
                self._last_min_y = min_y[0]
                self._last_max_x = max_x[0]
                self._last_max_y = max_y[0]
                min_x[0] = min(tmp_min_x, min_x[0]) - self._inflation_radius
                min_y[0] = min(tmp_min_y, min_y[0]) - self._inflation_radius
                max_x[0] = max(tmp_max_x, max_x[0]) + self._inflation_radius
                max_y[0] = max(tmp_max_y, max_y[0]) + self._inflation_radius

    def on_footprint_changed(self) -> None:
        """Recompute the inscribed radius and caches when the footprint changes."""
        with self._access:
            self._inscribed_radius = self._layered_costmap.get_inscribed_radius()
            self._cell_inflation_radius = self.cell_distance(self._inflation_radius)
            self.compute_caches()
            self._need_reinflation = True

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Update the costs in the master costmap within the given window.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to inflate.
        min_i, min_j : int
            Lower x/y boundary of the window to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to update, in cells.

        """
        with self._access:
            if not self._enabled or self._cell_inflation_radius == 0:
                return

            master_array = master_grid.get_char_map()
            size_x = master_grid.size_x
            size_y = master_grid.size_y

            if len(self._seen) != size_x * size_y:
                self._seen = [False] * (size_x * size_y)
            else:
                for k in range(len(self._seen)):
                    self._seen[k] = False

            base_min_i = min_i
            base_min_j = min_j
            base_max_i = max_i
            base_max_j = max_j
            min_i -= self._cell_inflation_radius
            min_j -= self._cell_inflation_radius
            max_i += self._cell_inflation_radius
            max_j += self._cell_inflation_radius

            min_i = max(0, min_i)
            min_j = max(0, min_j)
            max_i = min(size_x, max_i)
            max_j = min(size_y, max_j)

            # Seed the distance-zero bin with the lethal obstacles.
            obs_bin = self._inflation_cells[0]
            for j in range(min_j, max_j):
                for i in range(min_i, max_i):
                    index = master_grid.get_index(i, j)
                    cost = master_array[index]
                    if cost == LETHAL_OBSTACLE or (
                            self._inflate_around_unknown and cost == NO_INFORMATION):
                        obs_bin.append(CellData(i, j, i, j))

            # Process cells by increasing distance.
            for dist_bin in self._inflation_cells:
                idx = 0
                while idx < len(dist_bin):
                    cell = dist_bin[idx]
                    idx += 1
                    mx = cell.x
                    my = cell.y
                    sx = cell.src_x
                    sy = cell.src_y
                    index = master_grid.get_index(mx, my)

                    if self._seen[index]:
                        continue
                    self._seen[index] = True

                    cost = self.cost_lookup(mx, my, sx, sy)
                    old_cost = master_array[index]
                    if base_min_i <= mx < base_max_i and base_min_j <= my < base_max_j:
                        if old_cost == NO_INFORMATION and (
                                cost > FREE_SPACE if self._inflate_unknown
                                else cost >= INSCRIBED_INFLATED_OBSTACLE):
                            master_array[index] = cost
                        else:
                            master_array[index] = max(old_cost, cost)

                    if mx > 0:
                        self.enqueue(index - 1, mx - 1, my, sx, sy)
                    if my > 0:
                        self.enqueue(index - size_x, mx, my - 1, sx, sy)
                    if mx < size_x - 1:
                        self.enqueue(index + 1, mx + 1, my, sx, sy)
                    if my < size_y - 1:
                        self.enqueue(index + size_x, mx, my + 1, sx, sy)

            # Free each bin once processed (the bins stay allocated but empty,
            # ready for the next cycle).
            for b in range(len(self._inflation_cells)):
                self._inflation_cells[b] = []
            self._current = True

    def enqueue(
        self, index: int, mx: int, my: int, src_x: int, src_y: int
    ) -> None:
        """
        Enqueue a new cell into the cache distance update search.

        Parameters
        ----------
        index : int
            The flat index of the cell.
        mx, my : int
            The cell coordinates.
        src_x, src_y : int
            The coordinates of the obstacle the inflation started at.

        """
        if not self._seen[index]:
            distance = self.distance_lookup(mx, my, src_x, src_y)
            if distance > self._cell_inflation_radius:
                return
            r = self._cell_inflation_radius + 2
            dist = self._distance_matrix[mx - src_x + r][my - src_y + r]
            self._inflation_cells[dist].append(CellData(mx, my, src_x, src_y))

    # ------------------------------------------------------------------
    # Caches
    # ------------------------------------------------------------------

    def compute_caches(self) -> None:
        """Compute the cached distance and cost lookup tables."""
        with self._access:
            if self._cell_inflation_radius == 0:
                return

            self._cache_length = self._cell_inflation_radius + 2

            if self._cell_inflation_radius != self._cached_cell_inflation_radius:
                n = self._cache_length * self._cache_length
                self._cached_costs = [0] * n
                self._cached_distances = [0.0] * n
                for i in range(self._cache_length):
                    for j in range(self._cache_length):
                        self._cached_distances[i * self._cache_length + j] = math.hypot(i, j)
                self._cached_cell_inflation_radius = self._cell_inflation_radius

            for i in range(self._cache_length):
                for j in range(self._cache_length):
                    self._cached_costs[i * self._cache_length + j] = self.compute_cost(
                        self._cached_distances[i * self._cache_length + j])

            self._max_dist = self.generate_integer_distances()
            self._inflation_cells = [[] for _ in range(self._max_dist + 1)]

    def generate_integer_distances(self) -> int:
        """
        Generate the integer distance bins (sorted by squared distance).

        Returns
        -------
        int
            The number of distinct integer distance levels.

        """
        r = self._cell_inflation_radius + 2
        size = r * 2 + 1

        points = [
            (x, y)
            for y in range(-r, r + 1)
            for x in range(-r, r + 1)
            if x * x + y * y <= r * r
        ]
        points.sort(key=lambda p: p[0] * p[0] + p[1] * p[1])

        distance_matrix = [[0] * size for _ in range(size)]
        last = (0, 0)
        level = 0
        for p in points:
            if p[0] * p[0] + p[1] * p[1] != last[0] * last[0] + last[1] * last[1]:
                level += 1
            distance_matrix[p[0] + r][p[1] + r] = level
            last = p

        self._distance_matrix = distance_matrix
        return level

    def distance_lookup(self, mx: int, my: int, src_x: int, src_y: int) -> float:
        """Look up the pre-computed distance between a cell and its source."""
        dx = mx - src_x if mx > src_x else src_x - mx
        dy = my - src_y if my > src_y else src_y - my
        return self._cached_distances[dx * self._cache_length + dy]

    def cost_lookup(self, mx: int, my: int, src_x: int, src_y: int) -> int:
        """Look up the pre-computed cost between a cell and its source."""
        dx = mx - src_x if mx > src_x else src_x - mx
        dy = my - src_y if my > src_y else src_y - my
        return self._cached_costs[dx * self._cache_length + dy]

    def cell_distance(self, world_dist: float) -> int:
        """Convert a world distance to a cell distance via the master costmap."""
        return self._layered_costmap.get_costmap().cell_distance(world_dist)

    # ------------------------------------------------------------------
    # InflationLayerInterface implementation
    # ------------------------------------------------------------------

    def compute_cost(self, distance: float) -> int:
        """
        Given a distance (in cells), compute a cost.

        Parameters
        ----------
        distance : float
            The distance from an obstacle, in cells.

        Returns
        -------
        int
            The corresponding cost value.

        """
        if distance == 0:
            return LETHAL_OBSTACLE
        if distance * self._resolution <= self._inscribed_radius:
            return INSCRIBED_INFLATED_OBSTACLE
        factor = math.exp(
            -1.0 * self._cost_scaling_factor
            * (distance * self._resolution - self._inscribed_radius))
        return int((INSCRIBED_INFLATED_OBSTACLE - 1) * factor)

    def get_cost_scaling_factor(self) -> float:
        """Return the cost scaling factor."""
        return self._cost_scaling_factor

    def get_inflation_radius(self) -> float:
        """Return the inflation radius in meters."""
        return self._inflation_radius

    def get_mutex(self) -> Any:
        """Return the mutex guarding the inflation information."""
        return self._access

    def is_clearable(self) -> bool:
        """Return whether clearing operations should be processed on this layer."""
        return False

    def reset(self) -> None:
        """Reset this costmap layer, forcing a full re-inflation on the next cycle."""
        self.match_size()
        self._current = False
        self._need_reinflation = True
