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
Unit tests for the ClearCostmapService.

Inspired by nav2_costmap_2d/test/unit/clear_costmap_service_test.cpp: the
service clears the costmap entirely, around a pose, or everywhere except a
region around the robot.
"""

import unittest

from geometry_msgs.msg import PoseStamped
from nav2_costmap_2d_py.core.clear_costmap_service import ClearCostmapService
from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap

import rclpy
from rclpy.node import Node


class _ClearableLayer:
    """Fake clearable layer recording whether reset() was called."""

    def __init__(self, clearable):
        self._clearable = clearable
        self.reset_called = False

    def is_clearable(self):
        return self._clearable

    def reset(self):
        self.reset_called = True


class _FakeCostmapROS:
    """Minimal Costmap2DROS stand-in for the clear service."""

    def __init__(self, costmap, layered_costmap, pose):
        self._costmap = costmap
        self._lc = layered_costmap
        self._pose = pose

    def get_costmap(self):
        return self._costmap

    def get_layered_costmap(self):
        return self._lc

    def get_robot_pose(self):
        return self._pose

    def get_name(self):
        return 'costmap'


def make_pose(x, y):
    """Build a PoseStamped at the given world position."""
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


class TestClearCostmapService(unittest.TestCase):
    """Test suite for ClearCostmapService."""

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
        """Build a fully lethal 10x10 costmap and the clear service."""
        self.node = Node('test_clear_costmap_service')
        self.costmap = Costmap2D(10, 10, 1.0, 0.0, 0.0, FREE_SPACE)
        for my in range(10):
            for mx in range(10):
                self.costmap.set_cost(mx, my, LETHAL_OBSTACLE)

        self.lc = LayeredCostmap('map', False, False)
        self.clearable = _ClearableLayer(True)
        self.non_clearable = _ClearableLayer(False)
        self.lc.add_plugin(self.clearable)        # type: ignore[arg-type]
        self.lc.add_plugin(self.non_clearable)    # type: ignore[arg-type]

        self.pose = make_pose(5.5, 5.5)
        self.ros = _FakeCostmapROS(self.costmap, self.lc, self.pose)
        self.service = ClearCostmapService(
            self.node, self.ros)  # type: ignore[arg-type]

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_clear_entirely(self) -> None:
        """Clearing entirely resets the grid and clearable layers."""
        self.service._clear_entirely()
        for my in range(10):
            for mx in range(10):
                self.assertEqual(self.costmap.get_cost(mx, my), FREE_SPACE)
        self.assertTrue(self.clearable.reset_called)
        self.assertFalse(self.non_clearable.reset_called)

    def test_clear_around_robot(self) -> None:
        """Clearing around the robot resets only nearby cells."""
        self.assertTrue(self.service._clear_around(2.0))
        # Robot is at cell (5, 5): a near cell is cleared, a far one is kept.
        self.assertEqual(self.costmap.get_cost(5, 5), FREE_SPACE)
        self.assertEqual(self.costmap.get_cost(0, 0), LETHAL_OBSTACLE)

    def test_clear_except_region(self) -> None:
        """Clearing except a region keeps nearby cells and clears the rest."""
        self.assertTrue(self.service._clear_except_region(2.0))
        # Robot is at cell (5, 5): a near cell is kept, a far one is cleared.
        self.assertEqual(self.costmap.get_cost(5, 5), LETHAL_OBSTACLE)
        self.assertEqual(self.costmap.get_cost(0, 0), FREE_SPACE)

    def test_clear_around_no_pose(self) -> None:
        """Clearing fails gracefully when the robot pose is unavailable."""
        self.ros._pose = None
        self.assertFalse(self.service._clear_around(2.0))


if __name__ == '__main__':
    unittest.main()
