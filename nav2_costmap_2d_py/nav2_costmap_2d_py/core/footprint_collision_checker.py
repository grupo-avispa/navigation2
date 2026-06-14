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
FootprintCollisionChecker for nav2_costmap_2d_py.

Checker for collision with a footprint on a costmap.
It mirrors the nav2_costmap_2d::FootprintCollisionChecker from the C++
implementation.
"""

import math
from typing import Iterator, List, Optional, Tuple

from nav2_costmap_2d_py.core.cost_values import LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D

Footprint = List[Tuple[float, float]]


def _line_iterator(x0: int, y0: int, x1: int, y1: int) -> Iterator[Tuple[int, int]]:
    """
    Yield the cells along the line ``(x0, y0)``..``(x1, y1)``, endpoints included.

    Parameters
    ----------
    x0, y0 : int
        Start cell.
    x1, y1 : int
        End cell.

    Yields
    ------
    tuple of int
        Each ``(x, y)`` cell on the line.

    """
    dx = x1 - x0
    dy = y1 - y0
    abs_dx = abs(dx)
    abs_dy = abs(dy)
    x_inc = 1 if dx > 0 else -1
    y_inc = 1 if dy > 0 else -1

    x = x0
    y = y0
    if abs_dx >= abs_dy:
        error = abs_dx // 2
        for _ in range(abs_dx):
            yield x, y
            x += x_inc
            error += abs_dy
            if error >= abs_dx:
                y += y_inc
                error -= abs_dx
        yield x, y
    else:
        error = abs_dy // 2
        for _ in range(abs_dy):
            yield x, y
            y += y_inc
            error += abs_dx
            if error >= abs_dy:
                x += x_inc
                error -= abs_dy
        yield x, y


class FootprintCollisionChecker:
    """Checker for collision with a footprint on a costmap."""

    def __init__(self, costmap: Optional[Costmap2D] = None) -> None:
        """
        Construct the collision checker.

        Parameters
        ----------
        costmap : Costmap2D, optional
            The costmap object to use for collision detection.

        """
        self._costmap = costmap

    def footprint_cost(self, footprint: Footprint) -> float:
        """
        Find the footprint cost in an oriented footprint.

        Parameters
        ----------
        footprint : list of tuple of float
            The oriented footprint, as world-coordinate ``(x, y)`` points.

        Returns
        -------
        float
            The maximum cost along the footprint outline, or ``LETHAL_OBSTACLE``
            if any vertex is out of bounds.

        """
        footprint_cost = 0.0

        ok, x0, y0 = self.world_to_map(footprint[0][0], footprint[0][1])
        if not ok:
            return float(LETHAL_OBSTACLE)

        xstart = x0
        ystart = y0
        x1 = x0
        y1 = y0

        for i in range(len(footprint) - 1):
            ok, x1, y1 = self.world_to_map(
                footprint[i + 1][0], footprint[i + 1][1])
            if not ok:
                return float(LETHAL_OBSTACLE)

            footprint_cost = max(self.line_cost(
                x0, x1, y0, y1), footprint_cost)

            x0 = x1
            y0 = y1

            if footprint_cost == float(LETHAL_OBSTACLE):
                return footprint_cost

        return max(self.line_cost(xstart, x1, ystart, y1), footprint_cost)

    def footprint_cost_at_pose(
        self, x: float, y: float, theta: float, footprint: Footprint
    ) -> float:
        """
        Find the footprint cost at a pose with an unoriented footprint.

        Parameters
        ----------
        x, y : float
            The position to place the footprint at, in world coordinates.
        theta : float
            The orientation of the footprint, in radians.
        footprint : list of tuple of float
            The unoriented footprint, in the robot frame.

        Returns
        -------
        float
            The maximum cost along the oriented footprint outline.

        """
        cos_th = math.cos(theta)
        sin_th = math.sin(theta)
        oriented_footprint: Footprint = []
        for px, py in footprint:
            new_x = x + (px * cos_th - py * sin_th)
            new_y = y + (px * sin_th + py * cos_th)
            oriented_footprint.append((new_x, new_y))
        return self.footprint_cost(oriented_footprint)

    def line_cost(self, x0: int, x1: int, y0: int, y1: int) -> float:
        """
        Get the cost for a line segment, the maximum cost along the line.

        Parameters
        ----------
        x0, x1 : int
            Start and end x cell coordinates.
        y0, y1 : int
            Start and end y cell coordinates.

        Returns
        -------
        float
            The maximum point cost along the line.

        """
        line_cost = 0.0
        for x, y in _line_iterator(x0, y0, x1, y1):
            point_cost = self.point_cost(x, y)
            if point_cost == float(LETHAL_OBSTACLE):
                return point_cost
            if line_cost < point_cost:
                line_cost = point_cost
        return line_cost

    def world_to_map(self, wx: float, wy: float) -> Tuple[bool, int, int]:
        """
        Get the map coordinates from a world point.

        Parameters
        ----------
        wx, wy : float
            The world coordinates.

        Returns
        -------
        (ok, mx, my)
            ``ok`` is False if the point is out of bounds.

        """
        assert self._costmap is not None
        return self._costmap.world_to_map(wx, wy)

    def point_cost(self, x: int, y: int) -> float:
        """
        Get the cost of a point.

        Parameters
        ----------
        x, y : int
            The cell coordinates.

        Returns
        -------
        float
            The cost of the cell.

        """
        assert self._costmap is not None
        return float(self._costmap.get_cost(x, y))

    def set_costmap(self, costmap: Costmap2D) -> None:
        """
        Set the current costmap object to use for collision detection.

        Parameters
        ----------
        costmap : Costmap2D
            The costmap object to use.

        """
        self._costmap = costmap

    def get_costmap(self) -> Optional[Costmap2D]:
        """
        Get the current costmap object.

        Returns
        -------
        Costmap2D or None
            The current costmap object.

        """
        return self._costmap
