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
Unit tests for the KeepoutFilter.

Inspired by nav2_costmap_2d/test/unit/keepout_filter_test.cpp: a keepout mask is
read per-cell (occupancy linearly mapped to cost) and merged onto the master
costmap with a max combination.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.filters.keepout_filter import KeepoutFilter
from nav2_msgs.msg import CostmapFilterInfo

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node


def make_mask(width, height, resolution, data, origin=(0.0, 0.0), frame='map'):
    """Build an OccupancyGrid keepout mask from flat data."""
    msg = OccupancyGrid()
    msg.header.frame_id = frame
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = list(data)
    return msg


class TestKeepoutFilter(unittest.TestCase):
    """Test suite for KeepoutFilter."""

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
        """Build a 10x10 layered costmap with a keepout filter."""
        self.node = Node('test_keepout_filter')
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)
        self.filter = KeepoutFilter()
        self.filter.initialize(self.lc, 'keepout_filter', None, self.node)
        self.lc.add_filter(self.filter)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_mask_marks_keepout(self) -> None:
        """A fully-occupied mask cell becomes a lethal cell in the master grid."""
        data = [0] * 100
        data[3 * 10 + 3] = 100   # keepout at cell (3, 3)
        self.filter._mask_callback(make_mask(10, 10, 1.0, data))

        master = self.lc.get_costmap()
        self.filter.update_costs(master, 0, 0, 10, 10)

        self.assertEqual(master.get_cost(3, 3), LETHAL_OBSTACLE)
        self.assertEqual(master.get_cost(0, 0), FREE_SPACE)

    def test_mask_intermediate_value_scaled(self) -> None:
        """An intermediate mask value is linearly scaled to a cost (not binarized)."""
        data = [50] * 100   # 50% occupancy -> round(50 * 254 / 100) = 127
        self.filter._mask_callback(make_mask(10, 10, 1.0, data))

        master = self.lc.get_costmap()
        self.filter.update_costs(master, 0, 0, 10, 10)
        self.assertEqual(master.get_cost(5, 5), 127)

    def test_filter_info_subscribes_to_mask(self) -> None:
        """The filter info callback subscribes to the announced mask topic."""
        info = CostmapFilterInfo()
        info.type = 0  # keepout
        info.filter_mask_topic = 'my_keepout_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.filter._filter_info_callback(info)
        self.assertIsNotNone(self.filter._mask_sub)
        self.assertTrue(self.filter._mask_topic.endswith('my_keepout_mask'))

    def test_not_clearable(self) -> None:
        """The keepout filter is never cleared."""
        self.assertFalse(self.filter.is_clearable())


if __name__ == '__main__':
    unittest.main()
