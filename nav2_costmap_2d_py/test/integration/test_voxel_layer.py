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
Unit tests for the VoxelLayer.

Mirrors nav2_costmap_2d/test/integration/voxel_tests: returns above the floor
are marked into the 3D voxel grid and projected into the 2D costmap.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.core.observation import Observation
from nav2_costmap_2d_py.plugins.voxel_layer import VoxelLayer
import rclpy
from rclpy.node import Node
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

_INF = float('inf')


def _bounds():
    """Return fresh (min_x, min_y, max_x, max_y) single-element bound lists."""
    return [_INF], [_INF], [-_INF], [-_INF]


def _make_observation(origin, points, obstacle_max_range=100.0,
                      raytrace_max_range=100.0):
    """Build a static Observation already expressed in the global frame."""
    obs = Observation()
    obs.origin.x, obs.origin.y, obs.origin.z = origin
    header = Header()
    header.frame_id = 'map'
    obs.cloud = point_cloud2.create_cloud_xyz32(header, points)
    obs.obstacle_max_range = obstacle_max_range
    obs.obstacle_min_range = 0.0
    obs.raytrace_max_range = raytrace_max_range
    obs.raytrace_min_range = 0.0
    return obs


class TestVoxelLayer(unittest.TestCase):
    """Test suite for VoxelLayer."""

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
        """Build a 10x10 layered costmap with a voxel layer."""
        self.node = Node('test_voxel_layer')
        self.lc = LayeredCostmap('map', False, False)
        self.layer = VoxelLayer()
        self.layer.initialize(self.lc, 'voxel_layer', None, self.node)
        self.lc.add_plugin(self.layer)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_is_clearable(self) -> None:
        """Voxel layers are clearable."""
        self.assertTrue(self.layer.is_clearable())

    def test_size_in_meters_z(self) -> None:
        """The vertical extent is z_voxels * z_resolution."""
        self.assertAlmostEqual(self.layer.get_size_in_meters_z(), 10 * 0.2)

    def test_world_to_map_3d(self) -> None:
        """A point above the floor maps to a positive voxel z index."""
        ok, mx, my, mz = self.layer.world_to_map_3d(2.5, 3.5, 0.5)
        self.assertTrue(ok)
        self.assertEqual((mx, my), (2, 3))
        self.assertEqual(mz, int(0.5 / 0.2))

    def test_marking_into_master(self) -> None:
        """A return above the floor is marked lethal and combined into the master."""
        master = self.lc.get_costmap()
        obs = _make_observation((5.5, 5.5, 0.1), [(6.5, 5.5, 0.1)])
        self.layer.add_static_observation(obs, marking=True, clearing=False)

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(5.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)

        self.assertEqual(master.get_cost(6, 5), LETHAL_OBSTACLE)

    def test_marking_filtered_by_height(self) -> None:
        """A return above max_obstacle_height is not marked."""
        master = self.lc.get_costmap()
        obs = _make_observation((5.5, 5.5, 0.1), [(6.5, 5.5, 5.0)])
        self.layer.add_static_observation(obs, marking=True, clearing=False)

        min_x, min_y, max_x, max_y = _bounds()
        self.layer.update_bounds(5.5, 5.5, 0.0, min_x, min_y, max_x, max_y)
        self.layer.update_costs(master, 0, 0, 10, 10)
        self.assertEqual(master.get_cost(6, 5), FREE_SPACE)

    def test_reset_clears_grid(self) -> None:
        """Reset clears the layer's 2D grid and the voxel grid."""
        self.layer.set_cost(2, 2, LETHAL_OBSTACLE)
        self.layer.reset()
        self.assertEqual(self.layer.get_cost(2, 2), FREE_SPACE)


if __name__ == '__main__':
    unittest.main()
