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
Unit tests for the CostmapLayer merge helpers.

Exercises the update_with_max / update_with_overwrite / update_with_true_overwrite
combination methods and the clear_area / touch helpers used by the obstacle,
static and keepout layers.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer


class _DummyLayer(CostmapLayer):
    """Minimal concrete CostmapLayer with no-op update hooks."""

    def update_bounds(self, robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y):
        """No-op bounds update."""
        pass

    def update_costs(self, master_grid, min_i, min_j, max_i, max_j):
        """No-op cost update."""
        pass


class TestCostmapLayer(unittest.TestCase):
    """Test suite for the CostmapLayer combination methods."""

    def setUp(self) -> None:
        """Create a 5x5 master grid and a matching layer grid."""
        self.master = Costmap2D(5, 5, 1.0, 0.0, 0.0, FREE_SPACE)
        self.layer = _DummyLayer()
        self.layer.enabled = True
        self.layer.resize_map(5, 5, 1.0, 0.0, 0.0)

    def test_touch(self) -> None:
        """Touch expands the bounding box to include the point."""
        min_x, min_y = [float('inf')], [float('inf')]
        max_x, max_y = [float('-inf')], [float('-inf')]
        CostmapLayer.touch(2.0, 3.0, min_x, min_y, max_x, max_y)
        CostmapLayer.touch(-1.0, 5.0, min_x, min_y, max_x, max_y)
        self.assertEqual((min_x[0], min_y[0]), (-1.0, 3.0))
        self.assertEqual((max_x[0], max_y[0]), (2.0, 5.0))

    def test_update_with_max(self) -> None:
        """update_with_max keeps the larger of master and layer per cell."""
        self.layer.set_cost(1, 1, 100)
        self.master.set_cost(2, 2, 200)
        self.layer.set_cost(2, 2, 50)
        self.master.set_cost(3, 3, NO_INFORMATION)
        self.layer.set_cost(3, 3, 10)

        self.layer.update_with_max(self.master, 0, 0, 5, 5)

        self.assertEqual(self.master.get_cost(1, 1), 100)   # layer wins
        self.assertEqual(self.master.get_cost(2, 2), 200)   # master wins
        self.assertEqual(self.master.get_cost(3, 3), 10)    # NO_INFORMATION overwritten

    def test_update_with_max_no_information_layer(self) -> None:
        """A NO_INFORMATION layer cell never overwrites the master."""
        self.master.set_cost(0, 0, 42)
        self.layer.set_cost(0, 0, NO_INFORMATION)
        self.layer.update_with_max(self.master, 0, 0, 5, 5)
        self.assertEqual(self.master.get_cost(0, 0), 42)

    def test_update_with_overwrite(self) -> None:
        """update_with_overwrite copies valid layer cells over the master."""
        self.master.set_cost(0, 0, 200)
        self.layer.set_cost(0, 0, 50)
        self.master.set_cost(1, 1, 200)
        self.layer.set_cost(1, 1, NO_INFORMATION)

        self.layer.update_with_overwrite(self.master, 0, 0, 5, 5)

        self.assertEqual(self.master.get_cost(0, 0), 50)    # overwritten
        self.assertEqual(self.master.get_cost(1, 1), 200)   # NO_INFORMATION not copied

    def test_update_with_true_overwrite(self) -> None:
        """update_with_true_overwrite copies every layer cell, including unknown."""
        self.master.set_cost(1, 1, 200)
        self.layer.set_cost(1, 1, NO_INFORMATION)
        self.layer.update_with_true_overwrite(self.master, 0, 0, 5, 5)
        self.assertEqual(self.master.get_cost(1, 1), NO_INFORMATION)

    def test_update_with_max_without_unknown_overwrite(self) -> None:
        """A NO_INFORMATION master cell is never overwritten by this variant."""
        self.master.set_cost(0, 0, NO_INFORMATION)
        self.layer.set_cost(0, 0, 100)
        self.master.set_cost(1, 1, 50)
        self.layer.set_cost(1, 1, 100)
        self.layer.update_with_max_without_unknown_overwrite(self.master, 0, 0, 5, 5)
        self.assertEqual(self.master.get_cost(0, 0), NO_INFORMATION)  # not overwritten
        self.assertEqual(self.master.get_cost(1, 1), 100)            # max taken

    def test_update_with_addition(self) -> None:
        """update_with_addition sums and caps at INSCRIBED_INFLATED_OBSTACLE - 1."""
        self.master.set_cost(0, 0, 10)
        self.layer.set_cost(0, 0, 20)
        self.master.set_cost(1, 1, 200)
        self.layer.set_cost(1, 1, 200)
        self.layer.update_with_addition(self.master, 0, 0, 5, 5)
        self.assertEqual(self.master.get_cost(0, 0), 30)
        self.assertEqual(self.master.get_cost(1, 1), INSCRIBED_INFLATED_OBSTACLE - 1)

    def test_extra_bounds(self) -> None:
        """add_extra_bounds is merged in (once) by use_extra_bounds."""
        self.layer.add_extra_bounds(1.0, 2.0, 3.0, 4.0)
        min_x, min_y = [float('inf')], [float('inf')]
        max_x, max_y = [float('-inf')], [float('-inf')]
        self.layer.use_extra_bounds(min_x, min_y, max_x, max_y)
        self.assertEqual((min_x[0], min_y[0]), (1.0, 2.0))
        self.assertEqual((max_x[0], max_y[0]), (3.0, 4.0))
        # The bounds are consumed: a second call leaves the box unchanged.
        min_x2 = [float('inf')]
        self.layer.use_extra_bounds(min_x2, [float('inf')], [float('-inf')], [float('-inf')])
        self.assertEqual(min_x2[0], float('inf'))

    def test_disabled_layer_does_not_update(self) -> None:
        """A disabled layer leaves the master grid untouched."""
        self.master.set_cost(0, 0, FREE_SPACE)
        self.layer.set_cost(0, 0, LETHAL_OBSTACLE)
        self.layer.enabled = False
        self.layer.update_with_max(self.master, 0, 0, 5, 5)
        self.assertEqual(self.master.get_cost(0, 0), FREE_SPACE)

    def test_clear_area(self) -> None:
        """clear_area resets cells inside the rectangle to NO_INFORMATION (matches C++)."""
        for my in range(5):
            for mx in range(5):
                self.layer.set_cost(mx, my, LETHAL_OBSTACLE)
        # Strict bounds: clears cells with 0 < x < 4 and 0 < y < 4
        self.layer.clear_area(0, 0, 4, 4)
        self.assertEqual(self.layer.get_cost(2, 2), NO_INFORMATION)
        # Border cells outside the strict interior are preserved
        self.assertEqual(self.layer.get_cost(0, 0), LETHAL_OBSTACLE)

    def test_clear_area_invert(self) -> None:
        """clear_area with invert clears everything outside the rectangle."""
        for my in range(5):
            for mx in range(5):
                self.layer.set_cost(mx, my, LETHAL_OBSTACLE)
        self.layer.clear_area(0, 0, 4, 4, invert=True)
        # Interior is preserved, exterior cleared.
        self.assertEqual(self.layer.get_cost(2, 2), LETHAL_OBSTACLE)
        self.assertEqual(self.layer.get_cost(0, 0), NO_INFORMATION)


if __name__ == '__main__':
    unittest.main()
