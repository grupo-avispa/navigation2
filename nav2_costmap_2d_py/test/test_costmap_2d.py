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
Unit tests for the Costmap2D class.

Inspired by nav2_costmap_2d/test/integration/costmap_tests.cpp, restricted to
the pure-logic API exposed by the Python port.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D

# Default geometry used across the tests: 10x10 grid, 1 m/cell, origin (0, 0).
GRID_WIDTH = 10
GRID_HEIGHT = 10
RESOLUTION = 1.0
WINDOW_LENGTH = 10


class TestCostmap2D(unittest.TestCase):
    """Test suite for Costmap2D."""

    def setUp(self) -> None:
        """Create a default 10x10 free-space costmap before each test."""
        self.map = Costmap2D(
            GRID_WIDTH, GRID_HEIGHT, RESOLUTION, 0.0, 0.0, FREE_SPACE
        )

    # ------------------------------------------------------------------
    # Construction / accessors
    # ------------------------------------------------------------------

    def test_dimensions(self) -> None:
        """Verify size, resolution and metric extents."""
        self.assertEqual(self.map.size_x, GRID_WIDTH)
        self.assertEqual(self.map.size_y, GRID_HEIGHT)
        self.assertAlmostEqual(self.map.resolution, RESOLUTION)
        self.assertAlmostEqual(self.map.size_x_meters, GRID_WIDTH * RESOLUTION)
        self.assertAlmostEqual(self.map.size_y_meters, GRID_HEIGHT * RESOLUTION)
        self.assertAlmostEqual(self.map.origin_x, 0.0)
        self.assertAlmostEqual(self.map.origin_y, 0.0)

    def test_default_value_fills_grid(self) -> None:
        """A costmap is filled with its default value on construction."""
        unknown = Costmap2D(3, 3, RESOLUTION, 0.0, 0.0, NO_INFORMATION)
        for my in range(3):
            for mx in range(3):
                self.assertEqual(unknown.get_cost(mx, my), NO_INFORMATION)

    def test_empty_construction(self) -> None:
        """A zero-sized costmap allocates no cells."""
        empty = Costmap2D()
        self.assertEqual(empty.size_x, 0)
        self.assertEqual(empty.size_y, 0)
        self.assertEqual(len(empty.get_char_map()), 0)

    # ------------------------------------------------------------------
    # Cell access / indexing
    # ------------------------------------------------------------------

    def test_get_set_cost(self) -> None:
        """Cost values written are read back unchanged."""
        self.map.set_cost(1, 2, LETHAL_OBSTACLE)
        self.assertEqual(self.map.get_cost(1, 2), LETHAL_OBSTACLE)

    def test_index_round_trip(self) -> None:
        """get_index and index_to_cells are inverse operations."""
        for my in range(GRID_HEIGHT):
            for mx in range(GRID_WIDTH):
                idx = self.map.get_index(mx, my)
                self.assertEqual(idx, my * GRID_WIDTH + mx)
                self.assertEqual(self.map.index_to_cells(idx), (mx, my))

    # ------------------------------------------------------------------
    # Coordinate conversion
    # ------------------------------------------------------------------

    def test_world_to_map_valid(self) -> None:
        """world_to_map succeeds and round-trips through map_to_world."""
        ok, mx, my = self.map.world_to_map(5.5, 3.2)
        self.assertTrue(ok)
        self.assertEqual((mx, my), (5, 3))

    def test_world_to_map_out_of_bounds(self) -> None:
        """world_to_map fails for coordinates outside the grid."""
        ok, _, _ = self.map.world_to_map(-1.0, 0.0)
        self.assertFalse(ok)
        ok, _, _ = self.map.world_to_map(0.0, -1.0)
        self.assertFalse(ok)
        ok, _, _ = self.map.world_to_map(100.0, 0.0)
        self.assertFalse(ok)
        ok, _, _ = self.map.world_to_map(0.0, 100.0)
        self.assertFalse(ok)

    def test_world_to_map_enforce_bounds(self) -> None:
        """world_to_map_enforce_bounds clamps to the legal grid range."""
        self.assertEqual(self.map.world_to_map_enforce_bounds(-50.0, -50.0), (0, 0))
        self.assertEqual(
            self.map.world_to_map_enforce_bounds(50.0, 50.0),
            (GRID_WIDTH - 1, GRID_HEIGHT - 1),
        )

    def test_map_to_world_cell_centre(self) -> None:
        """map_to_world returns the centre of the cell."""
        wx, wy = self.map.map_to_world(0, 0)
        self.assertAlmostEqual(wx, 0.5)
        self.assertAlmostEqual(wy, 0.5)

    def test_is_in_bounds(self) -> None:
        """is_in_bounds reflects the grid extents."""
        self.assertTrue(self.map.is_in_bounds(0, 0))
        self.assertTrue(self.map.is_in_bounds(GRID_WIDTH - 1, GRID_HEIGHT - 1))
        self.assertFalse(self.map.is_in_bounds(-1, 0))
        self.assertFalse(self.map.is_in_bounds(GRID_WIDTH, 0))

    # ------------------------------------------------------------------
    # Map-wide operations
    # ------------------------------------------------------------------

    def test_reset_maps(self) -> None:
        """reset_maps restores every cell to the default value."""
        self.map.set_cost(4, 4, LETHAL_OBSTACLE)
        self.map.reset_maps()
        self.assertEqual(self.map.get_cost(4, 4), FREE_SPACE)

    def test_reset_map_region(self) -> None:
        """reset_map clears only the requested region."""
        for my in range(GRID_HEIGHT):
            for mx in range(GRID_WIDTH):
                self.map.set_cost(mx, my, LETHAL_OBSTACLE)
        self.map.reset_map(0, 0, 5, 5)
        self.assertEqual(self.map.get_cost(2, 2), FREE_SPACE)
        # Outside the reset region the obstacle is preserved
        self.assertEqual(self.map.get_cost(7, 7), LETHAL_OBSTACLE)

    def test_resize_map(self) -> None:
        """resize_map reinitializes the grid to the new geometry."""
        self.map.set_cost(1, 1, LETHAL_OBSTACLE)
        self.map.resize_map(4, 6, 0.5, 1.0, 2.0)
        self.assertEqual(self.map.size_x, 4)
        self.assertEqual(self.map.size_y, 6)
        self.assertAlmostEqual(self.map.resolution, 0.5)
        self.assertAlmostEqual(self.map.origin_x, 1.0)
        self.assertAlmostEqual(self.map.origin_y, 2.0)
        # Resize resets the contents to the default value
        self.assertEqual(self.map.get_cost(1, 1), FREE_SPACE)

    def test_set_default_value(self) -> None:
        """The default value is configurable through the property."""
        self.map.default_value = NO_INFORMATION
        self.assertEqual(self.map.default_value, NO_INFORMATION)
        self.map.reset_maps()
        self.assertEqual(self.map.get_cost(0, 0), NO_INFORMATION)

    # ------------------------------------------------------------------
    # Polygons and footprints
    # ------------------------------------------------------------------

    def test_set_convex_polygon_cost(self) -> None:
        """A convex polygon fills its interior cells with the given cost."""
        polygon = [(2.0, 2.0), (7.0, 2.0), (7.0, 7.0), (2.0, 7.0)]
        self.assertTrue(self.map.set_convex_polygon_cost(polygon, LETHAL_OBSTACLE))
        # Interior point is filled
        self.assertEqual(self.map.get_cost(4, 4), LETHAL_OBSTACLE)
        # A point well outside is untouched
        self.assertEqual(self.map.get_cost(9, 9), FREE_SPACE)

    def test_set_convex_polygon_cost_out_of_bounds(self) -> None:
        """A polygon with a vertex outside the map is rejected and fills nothing."""
        polygon = [(2.0, 2.0), (7.0, 2.0), (7.0, 7.0), (-5.0, 7.0)]
        self.assertFalse(self.map.set_convex_polygon_cost(polygon, LETHAL_OBSTACLE))
        self.assertEqual(self.map.get_cost(4, 4), FREE_SPACE)

    def test_set_convex_polygon_cost_degenerate(self) -> None:
        """A sub-triangle polygon that is in-bounds succeeds but fills nothing (matches C++)."""
        before = bytes(self.map.get_char_map())
        self.assertTrue(self.map.set_convex_polygon_cost([(0.0, 0.0)], LETHAL_OBSTACLE))
        self.assertEqual(bytes(self.map.get_char_map()), before)

    def test_footprint_cost(self) -> None:
        """footprint_cost returns the maximum cost under the footprint."""
        self.map.set_cost(5, 5, LETHAL_OBSTACLE)
        footprint = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
        # Centred on cell (5, 5) world centre (5.5, 5.5)
        cost = self.map.footprint_cost(5.5, 5.5, 0.0, footprint)
        self.assertEqual(cost, LETHAL_OBSTACLE)

    def test_footprint_cost_out_of_bounds(self) -> None:
        """footprint_cost returns -1 when a footprint point leaves the grid."""
        footprint = [(-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)]
        cost = self.map.footprint_cost(0.0, 0.0, 0.0, footprint)
        self.assertEqual(cost, -1.0)

    # ------------------------------------------------------------------
    # update_origin
    # ------------------------------------------------------------------

    def test_update_origin_no_shift(self) -> None:
        """Moving by less than a cell leaves the origin unchanged."""
        self.map.set_cost(3, 3, LETHAL_OBSTACLE)
        self.map.update_origin(0.0, 0.0)
        self.assertAlmostEqual(self.map.origin_x, 0.0)
        self.assertEqual(self.map.get_cost(3, 3), LETHAL_OBSTACLE)

    def test_update_origin_shift(self) -> None:
        """
        Moving the origin shifts the data so its world position is preserved.

        Mirrors nav2_costmap_2d::Costmap2D::updateOrigin: an obstacle at cell
        (5, 5) (world centre 5.5 m) must remain at world 5.5 m after the origin
        moves to 1.0 m, i.e. it lands in cell (4, 5).
        """
        self.map.set_cost(5, 5, LETHAL_OBSTACLE)
        self.map.update_origin(1.0, 0.0)
        self.assertAlmostEqual(self.map.origin_x, 1.0)
        self.assertEqual(self.map.get_cost(4, 5), LETHAL_OBSTACLE)


if __name__ == '__main__':
    unittest.main()
