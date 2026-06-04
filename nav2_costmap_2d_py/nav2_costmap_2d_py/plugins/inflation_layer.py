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

from typing import List

from nav2_costmap_2d_py.core.cost_values import (
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer
import numpy as np
from scipy import ndimage


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

        self._inflate(master_grid, min_i, min_j, max_i, max_j)
        self._current = True

    def reset(self) -> None:
        """Reset this costmap layer, forcing a full re-inflation on the next cycle."""
        self._need_reinflation = True
        self._current = False

    # ------------------------------------------------------------------
    # Vectorised inflation
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
        Inflate lethal cells using a vectorised Euclidean distance transform.

        Instead of a per-cell BFS (which holds the GIL for seconds on a global
        costmap and starves the lifecycle bond heartbeat), this computes the
        distance from every cell to the nearest obstacle in one ``scipy``
        call and maps those distances to costs with ``numpy``.

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
        sy = master.size_y
        res = master.resolution
        inscribed = self._layered_costmap.inscribed_radius

        # Writable view sharing memory with the underlying bytearray.
        grid = np.frombuffer(master.get_char_map(), dtype=np.uint8).reshape(sy, sx)

        # Obstacle seeds: lethal + inscribed cells (and unknown if requested).
        obstacles = (grid == LETHAL_OBSTACLE) | (grid == INSCRIBED_INFLATED_OBSTACLE)
        if self._inflate_around_unknown:
            obstacles |= (grid == NO_INFORMATION)
        if not obstacles.any():
            return

        # Distance (in metres) from every cell to the nearest obstacle.
        # distance_transform_edt measures distance to the nearest zero, so we
        # feed the complement of the obstacle mask.
        dist_m = ndimage.distance_transform_edt(~obstacles) * res

        # Vectorised distance -> cost mapping, mirroring _cost_at_distance.
        within = dist_m <= self._inflation_radius
        factor = np.exp(-self._cost_scaling_factor * (dist_m - inscribed))
        cost = ((INSCRIBED_INFLATED_OBSTACLE - 1) * factor).astype(np.int32)
        cost = np.where(cost > 0, np.clip(cost, 1, INSCRIBED_INFLATED_OBSTACLE - 1), 0)

        new_cost = np.zeros((sy, sx), dtype=np.uint8)
        new_cost[within] = cost[within].astype(np.uint8)
        new_cost[within & (dist_m <= inscribed)] = INSCRIBED_INFLATED_OBSTACLE

        # Restrict writes to the dirty window and never lower existing costs or
        # overwrite real obstacles.
        win = (slice(min_j, max_j), slice(min_i, max_i))
        sub = grid[win]
        sub_new = new_cost[win]
        free = (sub != LETHAL_OBSTACLE) & (sub != INSCRIBED_INFLATED_OBSTACLE)
        update = free & (sub_new > sub)
        sub[update] = sub_new[update]
