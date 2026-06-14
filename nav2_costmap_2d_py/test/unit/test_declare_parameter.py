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
Unit tests for Layer parameter declaration.

Mirrors nav2_costmap_2d/test/unit/declare_parameter_test.cpp: a layer declares
parameters under its own namespace via the declare-or-get helper.
"""

import unittest
from typing import List

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layer import Layer
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
import rclpy
from rclpy.node import Node


class _LayerWrapper(Layer):
    """Concrete Layer with no-op update methods, for testing the base class."""

    def update_bounds(self, robot_x: float, robot_y: float, robot_yaw: float,
                      min_x: List[float], min_y: List[float],
                      max_x: List[float], max_y: List[float]) -> None:
        """No-op bounds update."""
        pass

    def update_costs(self, master_grid: Costmap2D, min_i: int, min_j: int,
                     max_i: int, max_j: int) -> None:
        """No-op costs update."""
        pass


class TestDeclareParameter(unittest.TestCase):
    """Test suite for Layer parameter declaration."""

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
        """Build a node and an initialized layer wrapper."""
        self.node = Node('test_declare_parameter')
        self.lc = LayeredCostmap('frame', False, False)
        self.layer = _LayerWrapper()
        self.layer.initialize(self.lc, 'test_layer', None, self.node)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_get_full_name(self) -> None:
        """get_full_name prefixes the parameter with the layer name."""
        self.assertEqual(self.layer.get_full_name('test1'), 'test_layer.test1')

    def test_use_valid_parameter(self) -> None:
        """A declared parameter is readable via the declare-or-get helper."""
        val = self.layer._declare_parameter_if_not_declared('test1', 'test_val1')
        self.assertEqual(val, 'test_val1')
        # The parameter is registered under the layer namespace.
        self.assertTrue(self.node.has_parameter('test_layer.test1'))
        self.assertEqual(
            self.node.get_parameter('test_layer.test1').value, 'test_val1')

    def test_declare_or_get_is_idempotent(self) -> None:
        """Re-declaring returns the existing value instead of overwriting it."""
        self.layer._declare_parameter_if_not_declared('test2', 5)
        # A second call with a different default keeps the first value.
        again = self.layer._declare_parameter_if_not_declared('test2', 99)
        self.assertEqual(again, 5)


if __name__ == '__main__':
    unittest.main()
