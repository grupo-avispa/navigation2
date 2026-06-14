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
Integration tests for the GetCosts service of Costmap2DROS.

Mirrors nav2_costmap_2d/test/unit/costmap_cost_service_test.cpp: the service
returns the cost at one or more world-frame poses.
"""

import unittest

from geometry_msgs.msg import Pose
from nav2_costmap_2d_py.core.cost_values import LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.costmap_2d_ros import Costmap2DROS
from nav2_msgs.srv import GetCosts
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform)
from tf2_ros import Buffer


def _pose(x, y):
    """Build a Pose at (x, y) with identity orientation."""
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.orientation.w = 1.0
    return p


class TestCostmapCostService(unittest.TestCase):
    """Test suite for the GetCosts service callback."""

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
        """Build a Costmap2DROS with a hand-built layered costmap."""
        self.ros = Costmap2DROS('cost_service_costmap')
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 1.0, 0.0, 0.0)
        self.lc.get_costmap().set_cost(5, 5, LETHAL_OBSTACLE)
        self.ros._layered_costmap = self.lc
        self.ros._global_frame = 'map'
        self.ros._robot_base_frame = 'base_link'
        self.ros._transform_tolerance = 0.3
        self.ros._tf_buffer = Buffer()

    def tearDown(self) -> None:
        """Destroy the node."""
        self.ros.destroy_node()

    def test_get_costs(self) -> None:
        """The service returns the cost at each pose, or -1 when out of bounds."""
        req = GetCosts.Request()
        req.header.frame_id = 'map'
        req.poses = [_pose(5.5, 5.5), _pose(100.0, 100.0)]
        resp = self.ros._get_costs_callback(req, GetCosts.Response())
        self.assertEqual(len(resp.costs), 2)
        self.assertEqual(resp.costs[0], float(LETHAL_OBSTACLE))
        self.assertEqual(resp.costs[1], -1.0)


if __name__ == '__main__':
    unittest.main()
