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
Unit tests for the StaticLayer.

Inspired by nav2_costmap_2d/test/integration/static_tests.cpp: an incoming
OccupancyGrid is interpreted into costmap cost values and written into the
master grid.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.plugins.static_layer import StaticLayer

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node


def make_occupancy_grid(width, height, resolution, data, origin=(0.0, 0.0)):
    """Build an OccupancyGrid message from flat data."""
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = list(data)
    return msg


class TestStaticLayer(unittest.TestCase):
    """Test suite for StaticLayer."""

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
        """Build a layered costmap with a static layer."""
        self.node = Node('test_static_layer')
        self.lc = LayeredCostmap('map', False, True)
        self.layer = StaticLayer()
        self.layer.initialize(self.lc, 'static_layer', None, self.node)
        self.lc.add_plugin(self.layer)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_interpret_value_trinary(self) -> None:
        """Trinary interpretation maps to FREE / LETHAL / NO_INFORMATION."""
        self.layer._track_unknown_space = True
        self.layer._trinary_costmap = True
        self.layer._lethal_threshold = 100
        self.assertEqual(self.layer._interpret_value(-1), NO_INFORMATION)
        self.assertEqual(self.layer._interpret_value(0), FREE_SPACE)
        self.assertEqual(self.layer._interpret_value(50), FREE_SPACE)
        self.assertEqual(self.layer._interpret_value(100), LETHAL_OBSTACLE)

    def test_interpret_value_analogue(self) -> None:
        """Non-trinary interpretation scales the occupancy to a cost."""
        self.layer._track_unknown_space = True
        self.layer._trinary_costmap = False
        self.layer._lethal_threshold = 100
        self.assertEqual(self.layer._interpret_value(50), int(0.5 * LETHAL_OBSTACLE))

    def test_interpret_value_unknown_as_free(self) -> None:
        """With unknown tracking off, -1 maps to free space."""
        self.layer._track_unknown_space = False
        self.assertEqual(self.layer._interpret_value(-1), FREE_SPACE)

    def test_map_callback_writes_master(self) -> None:
        """A received map is interpreted and written into the master grid."""
        # 3x3 grid: free, lethal and unknown cells.
        data = [
            0, 100, -1,
            0, 0, 0,
            100, 0, -1,
        ]
        msg = make_occupancy_grid(3, 3, 1.0, data)
        self.layer._map_callback(msg)

        master = self.lc.get_costmap()
        self.assertEqual(master.size_x, 3)
        self.assertEqual(master.size_y, 3)

        self.layer.update_costs(master, 0, 0, 3, 3)

        self.assertEqual(master.get_cost(0, 0), FREE_SPACE)
        self.assertEqual(master.get_cost(1, 0), LETHAL_OBSTACLE)
        self.assertEqual(master.get_cost(2, 0), NO_INFORMATION)
        self.assertEqual(master.get_cost(0, 2), LETHAL_OBSTACLE)


if __name__ == '__main__':
    unittest.main()
