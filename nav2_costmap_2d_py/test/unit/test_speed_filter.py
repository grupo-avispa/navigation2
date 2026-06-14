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
Unit tests for the SpeedFilter.

Inspired by nav2_costmap_2d/test/unit/speed_filter_test.cpp: a speed-limit mask
maps the robot pose to a speed restriction using ``speed = base + data * mult``.
"""

import unittest

from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.filters.costmap_filter import (
    NO_SPEED_LIMIT,
    SPEED_FILTER_PERCENT,
)
from nav2_costmap_2d_py.filters.speed_filter import SpeedFilter
from nav2_msgs.msg import CostmapFilterInfo

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node

_INF = float('inf')


def make_mask(width, height, resolution, data, origin=(0.0, 0.0), frame='map'):
    """Build an OccupancyGrid speed mask from flat data."""
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


class TestSpeedFilter(unittest.TestCase):
    """Test suite for SpeedFilter."""

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
        """Build a layered costmap with a speed filter and a loaded mask."""
        self.node = Node('test_speed_filter')
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)
        self.filter = SpeedFilter()
        self.filter.initialize(self.lc, 'speed_filter', None, self.node)
        self.lc.add_filter(self.filter)

        data = [0] * 100
        data[3 * 10 + 3] = 60   # speed-restricted zone at cell (3, 3)
        self.filter._mask_callback(make_mask(10, 10, 1.0, data))

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def _process_at(self, rx, ry):
        """Buffer a robot pose and run the filter."""
        min_x, min_y, max_x, max_y = _bounds()
        self.filter.update_bounds(rx, ry, 0.0, min_x, min_y, max_x, max_y)
        self.filter.update_costs(self.lc.get_costmap(), 0, 0, 10, 10)

    def test_is_current_always_true(self) -> None:
        """The speed filter is always considered current."""
        self.assertTrue(self.filter.is_current())

    def test_filter_info_sets_mode(self) -> None:
        """The filter info callback sets the mode and base/multiplier."""
        info = CostmapFilterInfo()
        info.type = SPEED_FILTER_PERCENT
        info.filter_mask_topic = 'speed_mask'
        info.base = 0.0
        info.multiplier = 1.0
        self.filter._filter_info_callback(info)
        self.assertTrue(self.filter._percentage)
        self.assertIsNotNone(self.filter._mask_sub)

    def test_speed_percentage(self) -> None:
        """In percentage mode speed = base + data * multiplier (data as percent)."""
        self.filter._percentage = True
        self.filter._base = 0.0
        self.filter._multiplier = 1.0
        self._process_at(3.5, 3.5)
        self.assertAlmostEqual(self.filter._speed_limit, 60.0)

    def test_speed_absolute(self) -> None:
        """In absolute mode the limit is the linear base + data * multiplier."""
        self.filter._percentage = False
        self.filter._base = 0.0
        self.filter._multiplier = 0.01
        self._process_at(3.5, 3.5)
        self.assertAlmostEqual(self.filter._speed_limit, 0.6)

    def test_full_speed_zone(self) -> None:
        """A zero mask cell yields the no-limit value."""
        self.filter._percentage = True
        self._process_at(0.5, 0.5)
        self.assertAlmostEqual(self.filter._speed_limit, NO_SPEED_LIMIT)

    def test_unknown_cell_does_nothing(self) -> None:
        """An unknown mask cell leaves the previous limit untouched (no publish)."""
        data = [-1] * 100
        self.filter._mask_callback(make_mask(10, 10, 1.0, data))
        self.filter._speed_limit_prev = 42.0
        self._process_at(3.5, 3.5)
        # Unknown cell: process returns early without updating prev.
        self.assertEqual(self.filter._speed_limit_prev, 42.0)

    def test_outside_mask_no_change(self) -> None:
        """Outside the mask nothing is published."""
        self.filter._speed_limit_prev = 7.0
        self._process_at(-5.0, -5.0)
        self.assertEqual(self.filter._speed_limit_prev, 7.0)


if __name__ == '__main__':
    unittest.main()
