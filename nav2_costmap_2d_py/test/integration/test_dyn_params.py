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
Integration tests for the dynamic-parameter callback of Costmap2DROS.

Mirrors nav2_costmap_2d/test/integration/dyn_params_tests.cpp: changing
parameters at runtime updates the costmap configuration.
"""

import unittest

from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap
from nav2_costmap_2d_py.costmap_2d_ros import Costmap2DROS
import rclpy
from rclpy.parameter import Parameter


class TestDynParams(unittest.TestCase):
    """Test suite for the dynamic-parameter callback."""

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
        self.ros = Costmap2DROS('dyn_params_costmap')
        self.lc = LayeredCostmap('map', False, False)
        self.lc.resize_map(10, 10, 0.05, 0.0, 0.0)
        self.ros._layered_costmap = self.lc
        self.ros._use_radius = True

    def tearDown(self) -> None:
        """Destroy the node."""
        self.ros.destroy_node()

    def test_update_frequency(self) -> None:
        """Setting update_frequency updates the cached value."""
        result = self.ros._dynamic_parameters_callback(
            [Parameter('update_frequency', Parameter.Type.DOUBLE, 10.0)])
        self.assertTrue(result.successful)
        self.assertEqual(self.ros._update_frequency, 10.0)

    def test_publish_frequency(self) -> None:
        """Setting publish_frequency updates the cached value."""
        result = self.ros._dynamic_parameters_callback(
            [Parameter('publish_frequency', Parameter.Type.DOUBLE, 2.0)])
        self.assertTrue(result.successful)
        self.assertEqual(self.ros._publish_frequency, 2.0)

    def test_robot_radius(self) -> None:
        """Setting robot_radius rebuilds the footprint from the radius."""
        result = self.ros._dynamic_parameters_callback(
            [Parameter('robot_radius', Parameter.Type.DOUBLE, 0.3)])
        self.assertTrue(result.successful)
        self.assertEqual(self.ros._robot_radius, 0.3)
        self.assertGreater(len(self.ros.get_footprint()), 0)

    def test_valid_footprint(self) -> None:
        """A valid footprint string switches off radius mode."""
        fp = '[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]'
        result = self.ros._dynamic_parameters_callback(
            [Parameter('footprint', Parameter.Type.STRING, fp)])
        self.assertTrue(result.successful)
        self.assertFalse(self.ros._use_radius)

    def test_invalid_footprint(self) -> None:
        """An invalid footprint string is rejected."""
        result = self.ros._dynamic_parameters_callback(
            [Parameter('footprint', Parameter.Type.STRING, '[[bad')])
        self.assertFalse(result.successful)


if __name__ == '__main__':
    unittest.main()
