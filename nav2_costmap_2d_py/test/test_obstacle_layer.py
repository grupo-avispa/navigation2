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
Unit tests for the ObstacleLayer.

Inspired by nav2_costmap_2d/test/unit/obstacle_layer_test.cpp: laser
observations are buffered and marked as lethal obstacles in the master grid.
"""

import math
import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.plugins.obstacle_layer import ObstacleLayer

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class TestObstacleLayer(unittest.TestCase):
    """Test suite for ObstacleLayer."""

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
        """Build a 10x10 layered costmap with an obstacle layer."""
        self.node = Node('test_obstacle_layer')
        self.lc = LayeredCostmap('map', False, False)
        self.layer = ObstacleLayer()
        self.layer.initialize(self.lc, 'obstacle_layer', None, self.node)
        self.lc.add_plugin(self.layer)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_is_clearable(self) -> None:
        """Obstacle layers are clearable."""
        self.assertTrue(self.layer.is_clearable())

    def test_marking_from_buffer(self) -> None:
        """A buffered observation is marked as a lethal obstacle."""
        master = self.lc.get_costmap()
        self.layer._observations[0]['buffer'].append((5.5, 5.5))

        self.layer.update_costs(master, 0, 0, 10, 10)

        self.assertEqual(master.get_cost(5, 5), LETHAL_OBSTACLE)
        # A cell with no observation stays free.
        self.assertEqual(master.get_cost(0, 0), FREE_SPACE)

    def test_marking_outside_window(self) -> None:
        """Observations outside the update window are not marked."""
        master = self.lc.get_costmap()
        self.layer._observations[0]['buffer'].append((5.5, 5.5))
        # Window excludes cell (5, 5)
        self.layer.update_costs(master, 0, 0, 3, 3)
        self.assertEqual(master.get_cost(5, 5), FREE_SPACE)

    def test_laser_callback_buffers_points(self) -> None:
        """A LaserScan callback fills the observation buffer."""
        scan = LaserScan()
        scan.header.frame_id = 'base_link'
        scan.angle_min = 0.0
        scan.angle_increment = math.pi / 2.0
        scan.range_min = 0.0
        scan.range_max = 10.0
        scan.ranges = [1.0, 1.0, 1.0, 1.0]

        buffer = self.layer._observations[0]['buffer']
        self.layer._laser_callback(
            scan, buffer, True, True, 2.5, 0.0, 3.0, 0.0
        )
        self.assertTrue(len(buffer) > 0)

    def test_reset_clears_buffer(self) -> None:
        """Reset empties the buffered observations."""
        self.layer._observations[0]['buffer'].append((1.0, 1.0))
        self.layer.reset()
        self.assertEqual(len(self.layer._observations[0]['buffer']), 0)


if __name__ == '__main__':
    unittest.main()
