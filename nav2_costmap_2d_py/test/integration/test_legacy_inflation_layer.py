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
Unit tests for the LegacyInflationLayer.

Mirrors nav2_costmap_2d/test/integration/legacy_inflation_tests.cpp: a single
lethal obstacle is surrounded by an inscribed ring and an exponentially-decaying
inflation computed with the BFS-based legacy algorithm.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
)
from nav2_costmap_2d_py.core.layered_costmap import (
    LayeredCostmap,
    make_footprint_from_radius,
)
from nav2_costmap_2d_py.plugins.legacy_inflation_layer import CellData, LegacyInflationLayer
import rclpy
from rclpy.node import Node


class TestLegacyInflationLayer(unittest.TestCase):
    """Test suite for LegacyInflationLayer."""

    @classmethod
    def setUpClass(cls) -> None:
        """Initialize ROS 2 for all tests."""
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        """Shutdown ROS 2 after all tests."""
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self) -> None:
        """Build a 10x10 layered costmap with a legacy inflation layer."""
        self.node = Node('test_legacy_inflation_layer')
        self.node.declare_parameter('inflation.inflation_radius', 3.0)
        self.node.declare_parameter('inflation.cost_scaling_factor', 10.0)

        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)

        self.layer = LegacyInflationLayer()
        self.layer.initialize(self.lc, 'inflation', None, self.node)
        self.lc.add_plugin(self.layer)
        self.lc.set_footprint(make_footprint_from_radius(1.5))

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_cell_data(self) -> None:
        """CellData stores the cell and source coordinates."""
        cell = CellData(1, 2, 3, 4)
        self.assertEqual((cell.x, cell.y, cell.src_x, cell.src_y), (1, 2, 3, 4))

    def test_single_obstacle_inflation(self) -> None:
        """A lethal cell inflates into an inscribed ring and a decaying halo."""
        master = self.lc.get_costmap()
        master.set_cost(5, 5, LETHAL_OBSTACLE)

        self.layer.update_costs(master, 0, 0, 10, 10)

        self.assertEqual(master.get_cost(5, 5), LETHAL_OBSTACLE)
        self.assertEqual(master.get_cost(6, 5), INSCRIBED_INFLATED_OBSTACLE)
        self.assertEqual(master.get_cost(5, 6), INSCRIBED_INFLATED_OBSTACLE)
        halo = master.get_cost(7, 5)
        self.assertGreater(halo, FREE_SPACE)
        self.assertLess(halo, INSCRIBED_INFLATED_OBSTACLE)
        self.assertEqual(master.get_cost(0, 0), FREE_SPACE)

    def test_disabled_layer_does_nothing(self) -> None:
        """A disabled legacy inflation layer leaves the costmap untouched."""
        master = self.lc.get_costmap()
        master.set_cost(5, 5, LETHAL_OBSTACLE)
        self.layer._enabled = False
        self.layer.update_costs(master, 0, 0, 10, 10)
        self.assertEqual(master.get_cost(6, 5), FREE_SPACE)

    def test_compute_cost(self) -> None:
        """compute_cost mirrors the C++ piecewise mapping (distance in cells)."""
        self.layer._resolution = 1.0
        self.layer._inscribed_radius = 1.47
        self.layer._cost_scaling_factor = 10.0
        self.assertEqual(self.layer.compute_cost(0), LETHAL_OBSTACLE)
        self.assertEqual(self.layer.compute_cost(1), INSCRIBED_INFLATED_OBSTACLE)
        far = self.layer.compute_cost(2)
        self.assertGreater(far, FREE_SPACE)
        self.assertLess(far, INSCRIBED_INFLATED_OBSTACLE)

    def test_accessors(self) -> None:
        """The interface accessors report the configured values."""
        self.assertEqual(self.layer.get_inflation_radius(), 3.0)
        self.assertEqual(self.layer.get_cost_scaling_factor(), 10.0)
        self.assertIsNotNone(self.layer.get_mutex())
        self.assertFalse(self.layer.is_clearable())


if __name__ == '__main__':
    unittest.main()
