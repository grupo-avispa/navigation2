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

Inspired by nav2_costmap_2d/test/unit/speed_filter_test.cpp: a speed-limit
mask maps the robot pose to a speed restriction.
"""

import unittest

from geometry_msgs.msg import PoseStamped
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.filters.speed_filter import SpeedFilter

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.node import Node


def make_mask(width, height, resolution, data, origin=(0.0, 0.0)):
    """Build an OccupancyGrid speed mask from flat data."""
    msg = OccupancyGrid()
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution
    msg.info.origin.position.x = origin[0]
    msg.info.origin.position.y = origin[1]
    msg.info.origin.orientation.w = 1.0
    msg.data = list(data)
    return msg


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

    def test_is_current_always_true(self) -> None:
        """The speed filter is always considered current."""
        self.assertTrue(self.filter.is_current())

    def test_speed_percentage(self) -> None:
        """In percentage mode the mask value is the speed limit percentage."""
        self.filter._percentage = True
        speed = self.filter._get_speed_at(3.5, 3.5)
        self.assertIsNotNone(speed)
        assert speed is not None
        self.assertAlmostEqual(speed, 60.0)

    def test_full_speed_zone(self) -> None:
        """A zero mask value restores full speed (0.0)."""
        speed = self.filter._get_speed_at(0.5, 0.5)
        self.assertIsNotNone(speed)
        assert speed is not None
        self.assertAlmostEqual(speed, 0.0)

    def test_outside_mask(self) -> None:
        """Outside the mask the lookup returns None."""
        self.assertIsNone(self.filter._get_speed_at(-5.0, -5.0))

    def test_speed_absolute(self) -> None:
        """In absolute mode the limit scales the base speed."""
        self.filter._percentage = False
        self.filter._base_speed = 0.5
        speed = self.filter._get_speed_at(3.5, 3.5)
        self.assertIsNotNone(speed)
        assert speed is not None
        self.assertAlmostEqual(speed, 0.5 * (1.0 - 60.0 / 100.0))

    def test_update_costs_publishes_on_change(self) -> None:
        """update_costs publishes a speed limit derived from the robot pose."""
        pose = PoseStamped()
        pose.pose.position.x = 3.5
        pose.pose.position.y = 3.5
        # Provide the robot pose hook the filter looks up on the node.
        self.node.get_robot_pose = lambda: pose  # type: ignore[attr-defined]
        self.filter._percentage = True

        self.filter.update_costs(self.lc.get_costmap(), 0, 0, 10, 10)
        self.assertIsNotNone(self.filter._last_speed_limit)
        assert self.filter._last_speed_limit is not None
        self.assertAlmostEqual(self.filter._last_speed_limit, 60.0)


if __name__ == '__main__':
    unittest.main()
