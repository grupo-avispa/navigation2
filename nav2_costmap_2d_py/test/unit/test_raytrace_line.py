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
Parity tests for Costmap2D.raytrace_line.

These pin the behaviour of nav2_util::raytraceLine / bresenham2D: midpoint
error start, the endpoint is always written, ``min_length`` skips cells near
the origin and ``max_length`` is scaled by the line length.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D


def _marked(costmap):
    """Return the set of (mx, my) cells set to LETHAL_OBSTACLE."""
    cells = set()
    for my in range(costmap.size_y):
        for mx in range(costmap.size_x):
            if costmap.get_cost(mx, my) == LETHAL_OBSTACLE:
                cells.add((mx, my))
    return cells


class TestRaytraceLine(unittest.TestCase):
    """Test suite for raytrace_line."""

    def setUp(self) -> None:
        """Build a 10x10 free costmap."""
        self.costmap = Costmap2D(10, 10, 1.0, 0.0, 0.0, FREE_SPACE)

    def test_horizontal(self) -> None:
        """A horizontal ray marks every cell including both endpoints."""
        self.costmap.raytrace_line(LETHAL_OBSTACLE, 0, 0, 4, 0)
        self.assertEqual(_marked(self.costmap), {(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)})

    def test_vertical(self) -> None:
        """A vertical ray marks every cell including both endpoints."""
        self.costmap.raytrace_line(LETHAL_OBSTACLE, 2, 1, 2, 4)
        self.assertEqual(_marked(self.costmap), {(2, 1), (2, 2), (2, 3), (2, 4)})

    def test_diagonal(self) -> None:
        """A 45-degree ray marks the staircase diagonal including the endpoint."""
        self.costmap.raytrace_line(LETHAL_OBSTACLE, 0, 0, 3, 3)
        self.assertEqual(_marked(self.costmap), {(0, 0), (1, 1), (2, 2), (3, 3)})

    def test_min_length_skips_near_origin(self) -> None:
        """min_length skips the cells closer than min_length to the origin."""
        self.costmap.raytrace_line(LETHAL_OBSTACLE, 0, 0, 9, 0, min_length=3)
        marked = _marked(self.costmap)
        self.assertNotIn((0, 0), marked)
        self.assertNotIn((2, 0), marked)
        self.assertIn((3, 0), marked)
        self.assertIn((9, 0), marked)

    def test_max_length_scales_with_distance(self) -> None:
        """max_length limits the trace to that many cells from the origin."""
        self.costmap.raytrace_line(LETHAL_OBSTACLE, 0, 0, 9, 0, max_length=2)
        marked = _marked(self.costmap)
        # scale = 2/9, dominant steps = int(2/9 * 9) = 2 -> cells 0, 1, 2
        self.assertEqual(marked, {(0, 0), (1, 0), (2, 0)})

    def test_single_cell(self) -> None:
        """A zero-length ray marks only the origin cell."""
        self.costmap.raytrace_line(LETHAL_OBSTACLE, 5, 5, 5, 5)
        self.assertEqual(_marked(self.costmap), {(5, 5)})


if __name__ == '__main__':
    unittest.main()
