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
Unit tests for the BinaryFilter.

Mirrors nav2_costmap_2d/test/unit/binary_filter_test.cpp: a binary mask flips a
boolean state when ``base + data * multiplier`` exceeds the flip threshold.
"""

import unittest

from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.filters.binary_filter import BinaryFilter
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node

_INF = float('inf')


def make_mask(width, height, resolution, data, origin=(0.0, 0.0), frame='map'):
    """Build an OccupancyGrid binary mask from flat data."""
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


def _bounds():
    """Return fresh (min_x, min_y, max_x, max_y) single-element bound lists."""
    return [_INF], [_INF], [-_INF], [-_INF]


class TestBinaryFilter(unittest.TestCase):
    """Test suite for BinaryFilter."""

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
        """Build a layered costmap with a binary filter and a loaded mask."""
        self.node = Node('test_binary_filter')
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)
        self.filter = BinaryFilter()
        self.filter.initialize(self.lc, 'binary_filter', None, self.node)
        self.lc.add_filter(self.filter)

        data = [0] * 100
        data[3 * 10 + 3] = 60   # flagged zone at cell (3, 3)
        self.filter._mask_callback(make_mask(10, 10, 1.0, data))

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def _process_at(self, rx, ry):
        """Buffer a robot pose and run the filter."""
        min_x, min_y, max_x, max_y = _bounds()
        self.filter.update_bounds(rx, ry, 0.0, min_x, min_y, max_x, max_y)
        self.filter.update_costs(self.lc.get_costmap(), 0, 0, 10, 10)

    def test_is_active(self) -> None:
        """The filter reports active once a mask has been received."""
        self.assertTrue(self.filter.is_active())

    def test_filter_info_sets_mask_sub(self) -> None:
        """The filter info callback subscribes to the announced mask topic."""
        info = CostmapFilterInfo()
        info.type = 3  # BINARY_FILTER
        info.filter_mask_topic = 'binary_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.filter._filter_info_callback(info)
        self.assertIsNotNone(self.filter._mask_sub)

    def test_flip_on_above_threshold(self) -> None:
        """A masked cell above the flip threshold switches the state on."""
        self.filter._base = 0.0
        self.filter._multiplier = 1.0
        self.filter._flip_threshold = 50.0
        self._process_at(3.5, 3.5)  # data 60 -> above 50
        self.assertTrue(self.filter._binary_state)

    def test_stay_default_below_threshold(self) -> None:
        """A masked cell below the flip threshold keeps the default state."""
        self.filter._base = 0.0
        self.filter._multiplier = 1.0
        self.filter._flip_threshold = 50.0
        self._process_at(0.5, 0.5)  # data 0 -> below 50
        self.assertFalse(self.filter._binary_state)

    def test_flip_back(self) -> None:
        """Leaving the flagged zone flips the state back to default."""
        self.filter._flip_threshold = 50.0
        self._process_at(3.5, 3.5)
        self.assertTrue(self.filter._binary_state)
        self._process_at(0.5, 0.5)
        self.assertFalse(self.filter._binary_state)

    def test_unknown_cell_does_nothing(self) -> None:
        """An unknown mask cell leaves the state untouched."""
        data = [-1] * 100
        self.filter._mask_callback(make_mask(10, 10, 1.0, data))
        self.filter._binary_state = True
        self._process_at(3.5, 3.5)
        self.assertTrue(self.filter._binary_state)


if __name__ == '__main__':
    unittest.main()
