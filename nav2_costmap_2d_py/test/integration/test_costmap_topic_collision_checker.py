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
Unit tests for the CostmapTopicCollisionChecker.

Mirrors nav2_costmap_2d/test/integration/test_costmap_topic_collision_checker.cpp:
a footprint is scored against a topic-based costmap.
"""

import unittest

from geometry_msgs.msg import Pose
from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_subscriber import CostmapSubscriber
from nav2_costmap_2d_py.core.costmap_topic_collision_checker import (
    CostmapTopicCollisionChecker,
)
from nav2_costmap_2d_py.core.exceptions import CollisionCheckerException
from nav2_msgs.msg import Costmap
import rclpy
from rclpy.node import Node

_FOOTPRINT = '[[0.2, 0.2], [0.2, -0.2], [-0.2, -0.2], [-0.2, 0.2]]'


def _make_costmap_msg(size_x, size_y, data):
    """Build a nav2_msgs/Costmap message with a 1 m resolution."""
    msg = Costmap()
    msg.header.frame_id = 'map'
    msg.metadata.size_x = size_x
    msg.metadata.size_y = size_y
    msg.metadata.resolution = 1.0
    msg.data = data
    return msg


def _pose(x, y):
    """Build a Pose at (x, y) with identity orientation."""
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.orientation.w = 1.0
    return p


class TestCostmapTopicCollisionChecker(unittest.TestCase):
    """Test suite for CostmapTopicCollisionChecker."""

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
        """Build a subscriber and inject a costmap."""
        self.node = Node('test_topic_collision_checker')
        self.sub = CostmapSubscriber(self.node, 'costmap')
        self.checker = CostmapTopicCollisionChecker(
            self.sub, footprint_string=_FOOTPRINT, name='test_checker')

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def _inject(self, data):
        """Push a 10x10 costmap into the subscriber."""
        self.sub.costmap_callback(_make_costmap_msg(10, 10, data))

    def test_collision_free_in_free_space(self) -> None:
        """A pose over free space is collision free with score zero."""
        self._inject([FREE_SPACE] * 100)
        self.assertEqual(self.checker.score_pose(_pose(5.5, 5.5)), 0.0)
        self.assertTrue(self.checker.is_collision_free(_pose(5.5, 5.5)))

    def test_collision_with_lethal(self) -> None:
        """A pose whose footprint covers a lethal cell is in collision."""
        data = [FREE_SPACE] * 100
        data[5 * 10 + 5] = LETHAL_OBSTACLE  # cell (5, 5)
        self._inject(data)
        self.assertFalse(self.checker.is_collision_free(_pose(5.5, 5.5)))

    def test_invalid_footprint_string_raises(self) -> None:
        """An invalid footprint string raises a CollisionCheckerException."""
        with self.assertRaises(CollisionCheckerException):
            CostmapTopicCollisionChecker(self.sub, footprint_string='[[bad')


if __name__ == '__main__':
    unittest.main()
