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
Unit tests for LayeredCostmap and the footprint helper functions.

Covers the update_map cycle, current/bounds bookkeeping and the footprint
utility functions (radii, parsing, padding and transforming).
"""

import math
import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layer import Layer
from nav2_costmap_2d_py.core.layered_costmap import (
    LayeredCostmap,
    make_footprint_from_radius,
    make_footprint_from_string,
    pad_footprint,
    transform_footprint,
)


class _MarkingLayer(Layer):
    """Test layer that claims the whole map and marks a single lethal cell."""

    def __init__(self, mark_cell=(5, 5)):
        super().__init__()
        self._enabled = True
        self._mark_cell = mark_cell

    def update_bounds(self, robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y):
        """Expand the bounding box to cover the whole 10x10 map."""
        min_x[0] = min(min_x[0], 0.0)
        min_y[0] = min(min_y[0], 0.0)
        max_x[0] = max(max_x[0], 9.5)
        max_y[0] = max(max_y[0], 9.5)

    def update_costs(self, master_grid, min_i, min_j, max_i, max_j):
        """Mark a single lethal cell inside the window."""
        mx, my = self._mark_cell
        master_grid.set_cost(mx, my, LETHAL_OBSTACLE)
        self._current = True


class TestLayeredCostmap(unittest.TestCase):
    """Test suite for LayeredCostmap."""

    def setUp(self) -> None:
        """Create a 10x10 non-rolling layered costmap that tracks free space."""
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)

    def test_construction(self) -> None:
        """A fresh layered costmap exposes its frame and rolling flag."""
        self.assertEqual(self.lc.get_global_frame_id(), 'map')
        self.assertFalse(self.lc.is_rolling())
        self.assertFalse(self.lc.is_initialized())

    def test_plugin_management(self) -> None:
        """Plugins and filters are stored and returned in order."""
        layer = _MarkingLayer()
        self.lc.add_plugin(layer)
        self.assertEqual(self.lc.get_plugins(), [layer])
        self.assertEqual(self.lc.get_filters(), [])

    def test_update_map(self) -> None:
        """update_map runs the layers and records the updated bounds."""
        layer = _MarkingLayer(mark_cell=(5, 5))
        self.lc.add_plugin(layer)
        self.lc.update_map(0.0, 0.0, 0.0)

        master = self.lc.get_costmap()
        self.assertEqual(master.get_cost(5, 5), LETHAL_OBSTACLE)
        self.assertTrue(self.lc.is_initialized())
        x0, y0, xn, yn = self.lc.get_updated_bounds()
        self.assertEqual((x0, y0), (0, 0))
        self.assertEqual((xn, yn), (10, 10))

    def test_update_map_no_bounds(self) -> None:
        """With no layer expanding the bounds nothing is updated."""
        self.lc.update_map(0.0, 0.0, 0.0)
        master = self.lc.get_costmap()
        self.assertEqual(master.get_cost(5, 5), FREE_SPACE)
        self.assertTrue(self.lc.is_initialized())

    def test_is_current(self) -> None:
        """is_current is True once the (enabled) layer reports current."""
        layer = _MarkingLayer()
        self.lc.add_plugin(layer)
        self.assertFalse(self.lc.is_current())
        self.lc.update_map(0.0, 0.0, 0.0)
        self.assertTrue(self.lc.is_current())

    def test_is_out_of_bounds(self) -> None:
        """is_out_of_bounds reflects whether the robot lies on the map."""
        self.assertFalse(self.lc.is_out_of_bounds(5.0, 5.0))
        self.assertTrue(self.lc.is_out_of_bounds(-5.0, 5.0))

    def test_set_footprint_radii(self) -> None:
        """set_footprint computes the circumscribed and inscribed radii."""
        footprint = [(0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)]
        self.lc.set_footprint(footprint)
        self.assertAlmostEqual(self.lc.circumscribed_radius, math.sqrt(0.5))
        self.assertAlmostEqual(self.lc.inscribed_radius, 0.5)
        self.assertEqual(self.lc.get_footprint(), footprint)


class TestFootprintHelpers(unittest.TestCase):
    """Test suite for the module-level footprint helper functions."""

    def test_make_footprint_from_radius(self) -> None:
        """make_footprint_from_radius builds a 16-point circle of that radius."""
        fp = make_footprint_from_radius(1.0)
        self.assertEqual(len(fp), 16)
        for x, y in fp:
            self.assertAlmostEqual(math.hypot(x, y), 1.0)

    def test_make_footprint_from_string(self) -> None:
        """make_footprint_from_string parses a list-of-pairs string."""
        fp = make_footprint_from_string('[[0.1, 0.2], [-0.1, 0.2], [-0.1, -0.2]]')
        self.assertEqual(fp, [(0.1, 0.2), (-0.1, 0.2), (-0.1, -0.2)])

    def test_make_footprint_from_string_invalid(self) -> None:
        """An unparsable footprint string yields an empty list."""
        self.assertEqual(make_footprint_from_string('not a footprint'), [])

    def test_pad_footprint(self) -> None:
        """pad_footprint pushes every point outward from the centroid."""
        fp = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
        padded = pad_footprint(fp, 0.5)
        for (px, py), (qx, qy) in zip(fp, padded):
            self.assertGreater(math.hypot(qx, qy), math.hypot(px, py))

    def test_transform_footprint(self) -> None:
        """transform_footprint rotates and translates the points."""
        fp = [(1.0, 0.0)]
        out = transform_footprint(2.0, 3.0, math.pi / 2.0, fp)
        self.assertAlmostEqual(out[0][0], 2.0)
        self.assertAlmostEqual(out[0][1], 4.0)


if __name__ == '__main__':
    unittest.main()
