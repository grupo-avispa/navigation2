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
Unit tests for the PluginContainerLayer.

Mirrors nav2_costmap_2d/test/integration/plugin_container_tests.cpp: the
container holds a list of plugins and combines their result into the master.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import (
    CombinationMethod,
    FREE_SPACE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.plugins.inflation_layer import InflationLayer
from nav2_costmap_2d_py.plugins.plugin_container_layer import PluginContainerLayer
import rclpy
from rclpy.node import Node

_INF = float('inf')


def _bounds():
    """Return fresh (min_x, min_y, max_x, max_y) single-element bound lists."""
    return [_INF], [_INF], [-_INF], [-_INF]


class TestPluginContainerLayer(unittest.TestCase):
    """Test suite for PluginContainerLayer."""

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
        """Build a 10x10 layered costmap with an (empty) plugin container."""
        self.node = Node('test_plugin_container_layer')
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)
        self.container = PluginContainerLayer()
        self.container.initialize(self.lc, 'container', None, self.node)
        self.lc.add_plugin(self.container)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_empty_container_is_not_clearable(self) -> None:
        """A container with no plugins is not clearable."""
        self.assertFalse(self.container.is_clearable())
        self.assertEqual(self.container.get_plugins(), [])

    def test_default_combination_method(self) -> None:
        """The default combination method is Max."""
        self.assertEqual(self.container._combination_method, CombinationMethod.Max)

    def test_default_value_is_unknown(self) -> None:
        """The container's internal grid defaults to NO_INFORMATION."""
        self.assertEqual(self.container.default_value, NO_INFORMATION)

    def test_combines_internal_grid_into_master(self) -> None:
        """A lethal cell in the container grid is combined into the master with Max."""
        master = self.lc.get_costmap()
        self.container.set_cost(5, 5, LETHAL_OBSTACLE)
        min_x, min_y, max_x, max_y = _bounds()
        self.container.update_bounds(0.0, 0.0, 0.0, min_x, min_y, max_x, max_y)
        self.container.update_costs(master, 0, 0, 10, 10)
        self.assertEqual(master.get_cost(5, 5), LETHAL_OBSTACLE)
        self.assertEqual(master.get_cost(0, 0), FREE_SPACE)

    def test_add_plugin_appends_and_initializes(self) -> None:
        """add_plugin appends the child and initializes it under the container name."""
        child = InflationLayer()
        self.container.add_plugin(child, 'inflation')
        self.assertIn(child, self.container.get_plugins())
        self.assertEqual(child.get_name(), 'container.inflation')


if __name__ == '__main__':
    unittest.main()
