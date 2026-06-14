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

Mirrors nav2_costmap_2d/test/integration/obstacle_tests.cpp: static
observations (origin + PointCloud2) are marked as lethal obstacles in the
layer's own grid during ``update_bounds`` and combined into the master grid
during ``update_costs``; raytracing clears free space and footprint clearing
happens *after* marking.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.core.observation import Observation
from nav2_costmap_2d_py.plugins.obstacle_layer import ObstacleLayer
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

_INF = float('inf')


def _bounds():
    """Return fresh (min_x, min_y, max_x, max_y) single-element bound lists."""
    return [_INF], [_INF], [-_INF], [-_INF]


def _make_observation(origin, points, obstacle_max_range=100.0,
                      obstacle_min_range=0.0, raytrace_max_range=100.0,
                      raytrace_min_range=0.0):
    """Build a static Observation already expressed in the global frame."""
    obs = Observation()
    obs.origin.x, obs.origin.y, obs.origin.z = origin
    header = Header()
    header.frame_id = 'map'
    obs.cloud = point_cloud2.create_cloud_xyz32(header, points)
    obs.obstacle_max_range = obstacle_max_range
    obs.obstacle_min_range = obstacle_min_range
    obs.raytrace_max_range = raytrace_max_range
    obs.raytrace_min_range = raytrace_min_range
    return obs


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

    def test_marking_into_master(self) -> None:
        """A buffered return is marked lethal in the layer and combined into the master."""
        master = self.lc.get_costmap()
        obs = _make_observation((5.5, 5.5, 0.0), [(6.5, 5.5, 0.0)])
        self.layer.add_static_observation(obs, marking=True, clearing=True)

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(5.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)

        self.assertEqual(master.get_cost(6, 5), LETHAL_OBSTACLE)
        # The raytraced free cell next to the origin stays free.
        self.assertEqual(master.get_cost(5, 5), FREE_SPACE)

    def test_marking_filtered_by_range(self) -> None:
        """A return beyond obstacle_max_range is not marked."""
        master = self.lc.get_costmap()
        obs = _make_observation(
            (0.5, 0.5, 0.0), [(9.5, 9.5, 0.0)], obstacle_max_range=2.5)
        self.layer.add_static_observation(obs, marking=True, clearing=False)

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(0.5, 0.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)
        self.assertEqual(master.get_cost(9, 9), FREE_SPACE)

    def test_raytrace_clears_obstacle(self) -> None:
        """A clearing ray frees a previously lethal cell in the layer."""
        self.layer.set_cost(3, 5, LETHAL_OBSTACLE)
        obs = _make_observation((0.5, 5.5, 0.0), [(6.5, 5.5, 0.0)])
        self.layer.add_static_observation(obs, marking=False, clearing=True)

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(0.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.assertEqual(self.layer.get_cost(3, 5), FREE_SPACE)

    def test_footprint_clearing_after_marking(self) -> None:
        """Footprint clearing removes an obstacle marked under the robot footprint."""
        master = self.lc.get_costmap()
        self.lc.set_footprint(
            [(-1.5, -1.5), (1.5, -1.5), (1.5, 1.5), (-1.5, 1.5)])
        obs = _make_observation((5.5, 5.5, 0.0), [(5.5, 5.5, 0.0)])
        self.layer.add_static_observation(obs, marking=True, clearing=False)

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(5.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)
        # The self-mark under the footprint is cleared (C++ clears after marking).
        self.assertEqual(master.get_cost(5, 5), FREE_SPACE)

    def test_reset_clears_grid(self) -> None:
        """Reset clears the layer's grid and flags a reset."""
        self.layer.set_cost(2, 2, LETHAL_OBSTACLE)
        self.layer.reset()
        self.assertEqual(self.layer.get_cost(2, 2), FREE_SPACE)
        self.assertTrue(self.layer._was_reset)

    def test_clear_static_observations(self) -> None:
        """Clearing static observations empties the marking/clearing lists."""
        obs = _make_observation((1.0, 1.0, 0.0), [(1.0, 1.0, 0.0)])
        self.layer.add_static_observation(obs, marking=True, clearing=True)
        self.layer.clear_static_observations(marking=True, clearing=True)
        _, marking = self.layer.get_marking_observations()
        _, clearing = self.layer.get_clearing_observations()
        self.assertEqual(marking, [])
        self.assertEqual(clearing, [])


if __name__ == '__main__':
    unittest.main()
