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
observations are marked as lethal obstacles in the layer's own grid (during
``update_bounds``) and then combined into the master grid (during
``update_costs``); raytracing clears free space and footprint clearing happens
*after* marking.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.plugins.obstacle_layer import ObstacleLayer

import rclpy
from rclpy.node import Node

_INF = float('inf')


def _bounds():
    """Return fresh (min_x, min_y, max_x, max_y) single-element bound lists."""
    return [_INF], [_INF], [-_INF], [-_INF]


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
        self.obs = self.layer._observations[0]

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_is_clearable(self) -> None:
        """Obstacle layers are clearable."""
        self.assertTrue(self.layer.is_clearable())

    def test_marking_into_master(self) -> None:
        """A buffered return is marked lethal in the layer and combined into the master."""
        master = self.lc.get_costmap()
        self.obs.origin = (5.5, 5.5)
        self.obs.points = [(6.5, 5.5)]

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(5.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)

        self.assertEqual(master.get_cost(6, 5), LETHAL_OBSTACLE)
        # The raytraced free cell next to the origin stays free.
        self.assertEqual(master.get_cost(5, 5), FREE_SPACE)

    def test_marking_filtered_by_range(self) -> None:
        """A return beyond obstacle_max_range is not marked."""
        master = self.lc.get_costmap()
        self.obs.origin = (0.5, 0.5)
        self.obs.points = [(9.5, 9.5)]  # ~12.7 m away, beyond default 2.5 m

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(0.5, 0.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)
        self.assertEqual(master.get_cost(9, 9), FREE_SPACE)

    def test_raytrace_clears_obstacle(self) -> None:
        """A clearing ray frees a previously lethal cell in the layer."""
        # Pre-mark a lethal cell in the layer's own grid.
        self.layer.set_cost(3, 5, LETHAL_OBSTACLE)
        # Clearing-only observation whose ray passes through (3, 5).
        self.obs.marking = False
        self.obs.clearing = True
        self.obs.origin = (0.5, 5.5)
        self.obs.points = [(6.5, 5.5)]

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(0.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.assertEqual(self.layer.get_cost(3, 5), FREE_SPACE)

    def test_footprint_clearing_after_marking(self) -> None:
        """Footprint clearing removes an obstacle marked under the robot footprint."""
        master = self.lc.get_costmap()
        # A square footprint around the robot at (5.5, 5.5).
        self.lc.set_footprint([(-1.5, -1.5), (1.5, -1.5), (1.5, 1.5), (-1.5, 1.5)])
        # A return that falls under the footprint.
        self.obs.origin = (5.5, 5.5)
        self.obs.points = [(5.5, 5.5)]

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(5.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)
        # The self-mark under the footprint is cleared (C++ clears after marking).
        self.assertEqual(master.get_cost(5, 5), FREE_SPACE)

    def test_reset_clears_observations(self) -> None:
        """Reset empties the buffered observations and flags a reset."""
        self.obs.origin = (1.0, 1.0)
        self.obs.points = [(1.0, 1.0)]
        self.layer.reset()
        self.assertIsNone(self.obs.origin)
        self.assertEqual(self.obs.points, [])
        self.assertTrue(self.layer._was_reset)


if __name__ == '__main__':
    unittest.main()
