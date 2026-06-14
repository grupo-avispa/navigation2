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
Unit tests for the Costmap2DPublisher cost translation.

Inspired by nav2_costmap_2d/test/unit/costmap_conversion_test.cpp: costmap cost
values are translated into OccupancyGrid occupancy values.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_2d_publisher import Costmap2DPublisher

import rclpy
from rclpy.node import Node


class TestCostmap2DPublisher(unittest.TestCase):
    """Test suite for Costmap2DPublisher."""

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
        """Build a costmap and its publisher."""
        self.node = Node('test_costmap_2d_publisher')
        self.costmap = Costmap2D(3, 3, 1.0, 0.0, 0.0, FREE_SPACE)
        self.pub = Costmap2DPublisher(
            self.node, self.costmap, 'map', 'costmap'
        )

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_translation_table_special_values(self) -> None:
        """Special cost values map to their OccupancyGrid equivalents."""
        table = Costmap2DPublisher._COST_TRANSLATION_TABLE_NP
        self.assertEqual(int(table[FREE_SPACE]), 0)
        self.assertEqual(int(table[INSCRIBED_INFLATED_OBSTACLE]), 99)
        self.assertEqual(int(table[LETHAL_OBSTACLE]), 100)
        self.assertEqual(int(table[NO_INFORMATION]), -1)

    def test_translation_table_monotonic(self) -> None:
        """Intermediate costs map to a strictly increasing 1..98 ramp."""
        table = Costmap2DPublisher._COST_TRANSLATION_TABLE_NP
        self.assertEqual(int(table[1]), 1)
        self.assertEqual(int(table[252]), 98)
        for c in range(1, 252):
            self.assertLessEqual(int(table[c]), int(table[c + 1]))

    def test_translate_costmap_buffer(self) -> None:
        """A raw costmap buffer is translated cell by cell."""
        self.costmap.set_cost(0, 0, LETHAL_OBSTACLE)
        self.costmap.set_cost(1, 0, NO_INFORMATION)
        self.costmap.set_cost(2, 0, FREE_SPACE)
        out = self.pub._translate_costmap(self.costmap.get_char_map())
        self.assertEqual(out[0], 100)
        self.assertEqual(out[1], -1)
        self.assertEqual(out[2], 0)

    def test_activation_flags(self) -> None:
        """on_activate / on_deactivate toggle the active flag."""
        self.assertFalse(self.pub._active)
        self.pub.on_activate()
        self.assertTrue(self.pub._active)
        self.pub.on_deactivate()
        self.assertFalse(self.pub._active)

    def test_publish_inactive_is_noop(self) -> None:
        """Publishing while inactive does nothing and does not raise."""
        self.pub.publish_costmap()  # inactive: should be a silent no-op

    def test_update_bounds_accumulates_dirty_window(self) -> None:
        """update_bounds widens the dirty window and publish resets it."""
        self.pub.update_bounds(1, 3, 0, 2)
        self.pub.update_bounds(0, 2, 1, 3)
        self.assertEqual((self.pub._x0, self.pub._xn), (0, 3))
        self.assertEqual((self.pub._y0, self.pub._yn), (0, 3))
        # After an (active) publish the window is reset to "empty".
        self.pub.on_activate()
        # Force the incremental path: pretend a full publish already happened.
        self.pub._saved_origin_x = self.costmap.origin_x
        self.pub._saved_origin_y = self.costmap.origin_y
        self.pub._saved_size_x = self.costmap.size_x
        self.pub._saved_size_y = self.costmap.size_y
        self.pub.update_bounds(0, 2, 0, 2)
        self.pub.publish_costmap()
        self.assertEqual((self.pub._xn, self.pub._yn), (0, 0))
        self.assertEqual(self.pub._x0, self.costmap.size_x)


if __name__ == '__main__':
    unittest.main()
