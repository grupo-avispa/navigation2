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
Unit tests for the CostmapSubscriber.

Mirrors nav2_costmap_2d/test/integration/test_costmap_subscriber.cpp: a Costmap
message is reconstructed into a Costmap2D and incremental updates are applied.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_subscriber import CostmapSubscriber
from nav2_msgs.msg import Costmap, CostmapUpdate
import rclpy
from rclpy.node import Node


def _make_costmap_msg(size_x, size_y, resolution=1.0, data=None):
    """Build a nav2_msgs/Costmap message."""
    msg = Costmap()
    msg.header.frame_id = 'map'
    msg.metadata.size_x = size_x
    msg.metadata.size_y = size_y
    msg.metadata.resolution = resolution
    msg.metadata.origin.position.x = 0.0
    msg.metadata.origin.position.y = 0.0
    msg.data = data if data is not None else [FREE_SPACE] * (size_x * size_y)
    return msg


class TestCostmapSubscriber(unittest.TestCase):
    """Test suite for CostmapSubscriber."""

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
        """Build a node and a costmap subscriber."""
        self.node = Node('test_costmap_subscriber')
        self.sub = CostmapSubscriber(self.node, 'costmap')

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_get_costmap_before_received_raises(self) -> None:
        """Requesting the costmap before any message raises."""
        self.assertFalse(self.sub.is_costmap_received())
        with self.assertRaises(RuntimeError):
            self.sub.get_costmap()

    def test_costmap_callback_builds_costmap(self) -> None:
        """A full Costmap message is reconstructed into a Costmap2D."""
        data = [FREE_SPACE] * 9
        data[4] = LETHAL_OBSTACLE  # cell (1, 1) in a 3x3 grid
        self.sub.costmap_callback(_make_costmap_msg(3, 3, 1.0, data))

        costmap = self.sub.get_costmap()
        self.assertEqual(costmap.size_x, 3)
        self.assertEqual(costmap.size_y, 3)
        self.assertEqual(costmap.get_cost(1, 1), LETHAL_OBSTACLE)
        self.assertEqual(costmap.get_cost(0, 0), FREE_SPACE)
        self.assertEqual(self.sub.get_frame_id(), 'map')

    def test_costmap_update_applied(self) -> None:
        """An incremental CostmapUpdate is applied to the stored costmap."""
        self.sub.costmap_callback(_make_costmap_msg(3, 3))
        self.sub.get_costmap()  # consume the full message

        upd = CostmapUpdate()
        upd.x = 1
        upd.y = 1
        upd.size_x = 1
        upd.size_y = 1
        upd.data = [LETHAL_OBSTACLE]
        self.sub.costmap_update_callback(upd)

        self.assertEqual(self.sub.get_costmap().get_cost(1, 1), LETHAL_OBSTACLE)

    def test_resize_on_new_size(self) -> None:
        """A message with a new size resizes the stored costmap."""
        self.sub.costmap_callback(_make_costmap_msg(3, 3))
        self.sub.get_costmap()
        self.sub.costmap_callback(_make_costmap_msg(4, 2))
        costmap = self.sub.get_costmap()
        self.assertEqual((costmap.size_x, costmap.size_y), (4, 2))


if __name__ == '__main__':
    unittest.main()
