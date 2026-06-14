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

These tests pin the C++ behaviour: clearing acts on the *clearable layers*
(not the master grid), uses a *square* region of side ``reset_distance`` and
resets cells to ``NO_INFORMATION``.
"""

import unittest

from geometry_msgs.msg import PoseStamped
from nav2_costmap_2d_py.core.clear_costmap_service import ClearCostmapService
from nav2_costmap_2d_py.core.cost_values import LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap

import rclpy
from rclpy.node import Node


class _FakeClearableLayer(CostmapLayer):
    """A concrete clearable layer with its own 10x10 grid filled with lethal cost."""

    def __init__(self, name, clearable=True):
        super().__init__()
        self._name = name
        self._enabled = True
        self._clearable = clearable
        self.reset_called = False
        self.resize_map(10, 10, 1.0, 0.0, 0.0)
        for my in range(10):
            for mx in range(10):
                self.set_cost(mx, my, LETHAL_OBSTACLE)

    def update_bounds(self, rx, ry, ryaw, min_x, min_y, max_x, max_y):
        pass

    def update_costs(self, master_grid, min_i, min_j, max_i, max_j):
        pass

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

    def get_global_frame_id(self):
        return 'map'

    def get_tf_buffer(self):
        return None

    def reset_layers(self):
        top = self._lc.get_costmap()
        top.reset_map(0, 0, top.size_x, top.size_y)
        for layer in self._lc.get_plugins():
            layer.reset()
        for f in self._lc.get_filters():
            f.reset()


def make_pose(x, y):
    """Build a PoseStamped at the given world position."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
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
        """Build a clearable layer, a master grid and the clear service."""
        self.node = Node('test_clear_costmap_service')
        self.master = Costmap2D(10, 10, 1.0, 0.0, 0.0, NO_INFORMATION)

        self.lc = LayeredCostmap('map', False, True)
        self.layer = _FakeClearableLayer('obstacle_layer', clearable=True)
        self.non_clearable = _FakeClearableLayer('static_layer', clearable=False)
        self.lc.add_plugin(self.layer)
        self.lc.add_plugin(self.non_clearable)

        self.pose = make_pose(5.5, 5.5)
        self.ros = _FakeCostmapROS(self.master, self.lc, self.pose)
        self.service = ClearCostmapService(
            self.node, self.ros)  # type: ignore[arg-type]

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_clear_entirely(self) -> None:
        """Clearing entirely resets the master and every layer (via reset_layers)."""
        self.assertTrue(self.service.clear_entirely())
        self.assertTrue(self.layer.reset_called)
        self.assertTrue(self.non_clearable.reset_called)

    def test_clear_entirely_specific_plugin(self) -> None:
        """Clearing a named plugin resets that layer's grid and the master."""
        self.assertTrue(self.service.clear_entirely(['obstacle_layer']))
        # The named layer's own grid is reset to its default value.
        self.assertEqual(self.layer.get_cost(5, 5), self.layer.default_value)

    def test_clear_entirely_invalid_plugin(self) -> None:
        """Requesting an unknown plugin fails and clears nothing."""
        self.assertFalse(self.service.clear_entirely(['does_not_exist']))
        self.assertEqual(self.layer.get_cost(5, 5), LETHAL_OBSTACLE)

    def test_clear_entirely_non_clearable_plugin(self) -> None:
        """Requesting a non-clearable plugin fails and clears nothing."""
        self.assertFalse(self.service.clear_entirely(['static_layer']))

    def test_clear_around_robot_is_square(self) -> None:
        """Clearing around the robot resets a square region in the *layer*."""
        self.assertTrue(self.service.clear_region(2.0, invert=False))
        # Robot at (5.5, 5.5): square side 2.0 -> only cell (5, 5) is inside.
        self.assertEqual(self.layer.get_cost(5, 5), NO_INFORMATION)
        self.assertEqual(self.layer.get_cost(0, 0), LETHAL_OBSTACLE)
        # The master grid is NOT touched directly (the layer carries the change).
        self.assertEqual(self.layer.get_cost(4, 4), LETHAL_OBSTACLE)

    def test_clear_except_region(self) -> None:
        """Clearing except a region keeps the square and clears the rest of the layer."""
        self.assertTrue(self.service.clear_region(2.0, invert=True))
        self.assertEqual(self.layer.get_cost(5, 5), LETHAL_OBSTACLE)
        self.assertEqual(self.layer.get_cost(0, 0), NO_INFORMATION)

    def test_clear_sets_extra_bounds(self) -> None:
        """Clearing a region registers extra bounds so the master is re-merged."""
        self.service.clear_region(2.0, invert=False)
        self.assertTrue(self.layer._has_extra_bounds)

    def test_clear_around_no_pose(self) -> None:
        """Clearing fails gracefully when the robot pose is unavailable."""
        self.ros._pose = None
        self.assertFalse(self.service.clear_region(2.0, invert=False))


if __name__ == '__main__':
    unittest.main()
