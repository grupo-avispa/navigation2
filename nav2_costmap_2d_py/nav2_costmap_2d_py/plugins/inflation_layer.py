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
import numpy as np
from scipy import ndimage


class InflationLayer(InflationLayerInterface):
    """
    Convolve the costmap by the robot's radius or footprint to inflate obstacles.

    Inflates lethal obstacles outward by the inflation radius, assigning
    exponentially-decayed costs as a function of distance to the nearest
    obstacle.
    """

    def __init__(self) -> None:
        """Initialize inflation layer defaults."""
        super().__init__()
        self._access = threading.RLock()
        self._inflation_radius = 0.55
        self._cost_scaling_factor = 10.0
        self._inflate_unknown = False
        self._inflate_around_unknown = False

        self._resolution = 0.0
        self._inscribed_radius = 0.0
        self._cell_inflation_radius = 0

        self._need_reinflation = True

        # Bounds touched on the previous cycle, so the next cycle can reset and
        # re-inflate the area that is no longer covered by an obstacle.
        self._last_min_x = -float('inf')
        self._last_min_y = -float('inf')
        self._last_max_x = float('inf')
        self._last_max_y = float('inf')

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
        """Process footprint changes, forcing a full re-inflation on the next cycle."""
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
            # Reset last_* to "no expansion" values so the next cycle won't merge
            # with these full-map bounds (avoids a double full-map update after a
            # reset).
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
            # Expand the update window by the inflation radius around both the
            # current and the previous obstacle-update region, so reset_map
            # clears the stale inflation before it is recomputed. Without this
            # the old inflation lingers and smears (especially with a rolling
            # window).
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

        master = master_grid
        res = master.resolution
        self._resolution = res
        self._inscribed_radius = self._layered_costmap.inscribed_radius
        self._cell_inflation_radius = master.cell_distance(self._inflation_radius)
        if self._cell_inflation_radius == 0:
            return

        sx = master.size_x
        sy = master.size_y
        min_i = max(0, min_i)
        min_j = max(0, min_j)
        max_i = min(sx, max_i)
        max_j = min(sy, max_j)

        # Writable view sharing memory with the underlying bytearray.
        grid = np.frombuffer(master.get_char_map(), dtype=np.uint8).reshape(sy, sx)

        # Build the obstacle mask (distance map seeds). Lethal cells always seed;
        # NO_INFORMATION cells seed too when inflate_around_unknown is set.
        obstacles = grid == LETHAL_OBSTACLE
        if self._inflate_around_unknown:
            obstacles |= (grid == NO_INFORMATION)
        if not obstacles.any():
            self._current = True
            return

        # Distance (in cells) from every cell to the nearest obstacle. The EDT
        # measures distance to the nearest zero, so feed the complement.
        distance_cells = ndimage.distance_transform_edt(~obstacles)

        self.apply_inflation(grid, distance_cells, min_i, min_j, max_i, max_j)
        self._current = True

    def reset(self) -> None:
        """Reset this costmap layer, forcing a full re-inflation on the next cycle."""
        self._need_reinflation = True
        self._current = False

    # ------------------------------------------------------------------
    # Cost mapping
    # ------------------------------------------------------------------

    def compute_cost(self, distance: float) -> int:
        """
        Map a distance (in cells) to a cost.

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

    def apply_inflation(
        self,
        grid: 'np.ndarray',
        distance_cells: 'np.ndarray',
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Apply the inflation costs from the distance map into the master grid window.

        Parameters
        ----------
        grid : numpy.ndarray
            2D ``uint8`` view of the master costmap (modified in place).
        distance_cells : numpy.ndarray
            2D distance-to-nearest-obstacle map, in cells.
        min_i, min_j : int
            Lower x/y boundary of the window to inflate, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to inflate, in cells.

        """
        win = (slice(min_j, max_j), slice(min_i, max_i))
        old = grid[win].astype(np.int32)
        dist = distance_cells[win]
        dist_m = dist * self._resolution

        # Vectorised compute_cost (matches the scalar version exactly).
        factor = np.exp(
            -1.0 * self._cost_scaling_factor * (dist_m - self._inscribed_radius))
        graded = ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor).astype(np.int32)
        cost = np.where(
            dist == 0,
            LETHAL_OBSTACLE,
            np.where(dist_m <= self._inscribed_radius, INSCRIBED_INFLATED_OBSTACLE, graded),
        )

        # Only cells within the inflation radius are touched.
        within = dist <= float(self._cell_inflation_radius)

        # NO_INFORMATION cells: promote only if allowed.
        is_unknown = old == NO_INFORMATION
        if self._inflate_unknown:
            promote = within & is_unknown & (cost > FREE_SPACE)
        else:
            promote = within & is_unknown & (cost >= INSCRIBED_INFLATED_OBSTACLE)
        # Every other in-radius cell takes max(old, cost).
        take_max = within & ~promote

        result = old.copy()
        result[promote] = cost[promote]
        result[take_max] = np.maximum(old[take_max], cost[take_max])
        grid[win] = result.astype(np.uint8)

    # ------------------------------------------------------------------
    # InflationLayerInterface implementation
    # ------------------------------------------------------------------

    def activate(self) -> None:
        """Activate the layer."""
        pass

    def deactivate(self) -> None:
        """Deactivate the layer."""
        pass

    def match_size(self) -> None:
        """Match the size of the master costmap and recompute the caches."""
        costmap = self._layered_costmap.get_costmap()
        self._resolution = costmap.resolution
        self._cell_inflation_radius = self.cell_distance(self._inflation_radius)
        self.compute_caches()
        self._need_reinflation = True

    def is_clearable(self) -> bool:
        """Return whether clearing operations should be processed on this layer."""
        return False

    def get_cost_scaling_factor(self) -> float:
        """Return the cost scaling factor."""
        return self._cost_scaling_factor

    def get_inflation_radius(self) -> float:
        """Return the inflation radius in meters."""
        return self._inflation_radius

    def get_mutex(self) -> Any:
        """Return the mutex guarding the inflation information."""
        return self._access

    def cell_distance(self, world_dist: float) -> int:
        """
        Convert a world distance to a cell distance via the master costmap.

        Parameters
        ----------
        world_dist : float
            The world distance, in metres.

        Returns
        -------
        int
            The equivalent cell distance.

        """
        return self._layered_costmap.get_costmap().cell_distance(world_dist)

    def compute_caches(self) -> None:
        """Generate the cell inflation radius from the current resolution."""
        if self._cell_inflation_radius == 0:
            self._inscribed_radius = self._layered_costmap.inscribed_radius
