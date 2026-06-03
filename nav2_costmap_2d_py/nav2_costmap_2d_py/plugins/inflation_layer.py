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
InflationLayer for nav2_costmap_2d_py.

Inflates lethal obstacles outward by the inflation radius, assigning
exponentially-decayed costs.

It mirrors the nav2_costmap_2d::InflationLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/InflationLayer"``
"""

import heapq
import math
from typing import List, Tuple

from nav2_costmap_2d_py.core.cost_values import INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer


class InflationLayer(Layer):
    """
    Convolve the costmap by the robot's radius or footprint to inflate obstacles.

    Inflates lethal obstacles outward by the inflation radius, assigning
    exponentially-decayed costs as a function of distance to the nearest
    obstacle.
    """

    def __init__(self) -> None:
        """Initialize inflation layer defaults."""
        super().__init__()
        self._inflation_radius = 0.55
        self._cost_scaling_factor = 10.0
        self._inflate_unknown = False
        self._inflate_around_unknown = False

        # Pre-computed distance/cost caches
        self._cell_inflation_radius: int = 0
        self._cached_costs: List[int] = []
        self._cached_distances: List[float] = []
        self._inflation_cells: List[Tuple[float, int, int, int]] = []

        self._need_reinflation = True

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer on startup: read the inflation parameters."""
        node = self._node

        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._inflation_radius = self._declare_parameter_if_not_declared('inflation_radius', 0.55)
        self._cost_scaling_factor = self._declare_parameter_if_not_declared(
            'cost_scaling_factor', 10.0)
        self._inflate_unknown = self._declare_parameter_if_not_declared('inflate_unknown', False)
        self._inflate_around_unknown = self._declare_parameter_if_not_declared(
            'inflate_around_unknown', False)

        self._need_reinflation = True

        node.get_logger().info(
            f'[InflationLayer] "{self._name}": '
            f'radius={self._inflation_radius:.3f} m  '
            f'scaling={self._cost_scaling_factor:.2f}'
        )

    def on_footprint_changed(self) -> None:
        """Process footprint changes, recomputing the cell inflation radius and caches."""
        if self._layered_costmap is None:
            return
        res = self._layered_costmap.get_costmap().resolution
        if res > 0:
            self._cell_inflation_radius = int(
                math.ceil(self._inflation_radius / res)
            )
            self._compute_caches(res)
        self._need_reinflation = True

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
        if self._need_reinflation:
            # Inflate whole map
            master = self._layered_costmap.get_costmap()
            ox = master.origin_x
            oy = master.origin_y
            min_x[0] = min(min_x[0], ox)
            min_y[0] = min(min_y[0], oy)
            max_x[0] = max(max_x[0], ox + master.size_x_meters)
            max_y[0] = max(max_y[0], oy + master.size_y_meters)
            self._need_reinflation = False

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
            The master costmap to inflate.
        min_i, min_j : int
            Lower x/y boundary of the window to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to update, in cells.

        """
        if not self._enabled:
            return

        res = master_grid.resolution
        if self._cell_inflation_radius == 0:
            self._cell_inflation_radius = int(math.ceil(self._inflation_radius / res))
            self._compute_caches(res)

        # BFS/Dijkstra-style inflation
        self._inflate(master_grid, min_i, min_j, max_i, max_j)
        self._current = True

    def reset(self) -> None:
        """Reset this costmap layer, forcing a full re-inflation on the next cycle."""
        self._need_reinflation = True
        self._current = False

    # ------------------------------------------------------------------
    # Cost computation
    # ------------------------------------------------------------------

    def _cost_at_distance(self, distance: float) -> int:
        """
        Given a distance from an obstacle, compute the inflated cost.

        Parameters
        ----------
        distance : float
            The distance from the obstacle, in metres.

        Returns
        -------
        int
            The computed cost value.

        """
        if distance == 0.0:
            return LETHAL_OBSTACLE
        if distance <= self._layered_costmap.inscribed_radius:
            return INSCRIBED_INFLATED_OBSTACLE
        factor = math.exp(-self._cost_scaling_factor
                          * (distance - self._layered_costmap.inscribed_radius))
        cost = int((INSCRIBED_INFLATED_OBSTACLE - 1) * factor)
        return max(1, cost) if cost > 0 else 0

    def _compute_caches(self, resolution: float) -> None:
        """
        Generate the cost and distance lookup tables for distance-to-cost mapping.

        Parameters
        ----------
        resolution : float
            The costmap resolution in meters/cell, used to convert cell offsets
            to world distances.

        """
        r = self._cell_inflation_radius + 2
        size = (2 * r + 1) ** 2
        self._cached_distances = [0.0] * size
        self._cached_costs = [0] * size
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                dist = math.sqrt(dx * dx + dy * dy) * resolution
                idx = (dy + r) * (2 * r + 1) + (dx + r)
                self._cached_distances[idx] = dist
                self._cached_costs[idx] = self._cost_at_distance(dist)

    # ------------------------------------------------------------------
    # BFS inflation
    # ------------------------------------------------------------------

    def _inflate(
        self,
        master: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Inflate lethal cells using a min-heap (distance-ordered BFS).

        Parameters
        ----------
        master : Costmap2D
            The master costmap whose cells are inflated in place.
        min_i, min_j : int
            Lower x/y boundary of the window to inflate, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to inflate, in cells.

        """
        sx = master.size_x
        arr = master.get_char_map()
        res = master.resolution

        seen = set()
        heap: List[Tuple[float, int, int]] = []  # (dist, mx, my)

        # Seed heap with lethal/inscribed cells in bounds
        for j in range(min_j, max_j):
            for i in range(min_i, max_i):
                idx = j * sx + i
                cost = arr[idx]
                if cost == LETHAL_OBSTACLE or cost == INSCRIBED_INFLATED_OBSTACLE:
                    heapq.heappush(heap, (0.0, i, j))
                    seen.add((i, j))

        while heap:
            dist, cx, cy = heapq.heappop(heap)
            if dist > self._inflation_radius:
                break

            inflated_cost = self._cost_at_distance(dist)
            if inflated_cost == 0:
                continue

            idx = cy * sx + cx
            existing = arr[idx]
            if (existing != LETHAL_OBSTACLE
                    and existing != INSCRIBED_INFLATED_OBSTACLE
                    and inflated_cost > existing):
                arr[idx] = inflated_cost

            # Expand to neighbours
            for ny in range(max(min_j, cy - 1), min(max_j, cy + 2)):
                for nx in range(max(min_i, cx - 1), min(max_i, cx + 2)):
                    if (nx, ny) in seen:
                        continue
                    ndist = math.sqrt((nx - cx) ** 2 + (ny - cy) ** 2) * res + dist
                    if ndist <= self._inflation_radius:
                        heapq.heappush(heap, (ndist, nx, ny))
                        seen.add((nx, ny))
