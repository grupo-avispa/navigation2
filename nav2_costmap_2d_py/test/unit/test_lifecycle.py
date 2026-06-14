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
Integration tests for the Costmap2DROS lifecycle transitions.

Mirrors nav2_costmap_2d/test/unit/lifecycle_test.cpp: the node configures and
activates once a transform from the global to the robot base frame is available.
"""

import unittest
from typing import Any

from geometry_msgs.msg import TransformStamped
from nav2_costmap_2d_py.costmap_2d_ros import Costmap2DROS
import rclpy
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

# Lifecycle transition callbacks ignore the state argument; pass an Any-typed
# placeholder so the direct calls type-check without per-call ignores.
_STATE: Any = None


class TestLifecycle(unittest.TestCase):
    """Test suite for the Costmap2DROS lifecycle transitions."""

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
        """Build a Costmap2DROS with a minimal (inflation-only) plugin set."""
        self.ros = Costmap2DROS('lifecycle_costmap')
        self.ros.set_parameters([
            Parameter('plugins', Parameter.Type.STRING_ARRAY, ['inflation_layer']),
            Parameter('observation_sources', Parameter.Type.STRING, ''),
            Parameter('initial_transform_timeout', Parameter.Type.DOUBLE, 0.0),
            Parameter('update_frequency', Parameter.Type.DOUBLE, 1.0),
        ])

    def tearDown(self) -> None:
        """Destroy the node."""
        self.ros.destroy_node()

    def _set_identity_transform(self) -> None:
        """Publish an identity global -> base transform into the tf buffer."""
        t = TransformStamped()
        t.header.frame_id = self.ros.get_global_frame_id()
        t.child_frame_id = self.ros.get_base_frame_id()
        t.transform.rotation.w = 1.0
        self.ros._tf_buffer.set_transform_static(t, 'test')

    def test_configure(self) -> None:
        """on_configure builds the layered costmap and returns SUCCESS."""
        result = self.ros.on_configure(_STATE)
        self.assertEqual(result, TransitionCallbackReturn.SUCCESS)
        self.assertIsNotNone(self.ros.get_layered_costmap())

    def test_full_cycle(self) -> None:
        """A full configure -> activate -> deactivate -> cleanup cycle succeeds."""
        self.assertEqual(
            self.ros.on_configure(_STATE), TransitionCallbackReturn.SUCCESS)
        self._set_identity_transform()
        self.assertEqual(
            self.ros.on_activate(_STATE), TransitionCallbackReturn.SUCCESS)
        self.assertEqual(
            self.ros.on_deactivate(_STATE), TransitionCallbackReturn.SUCCESS)
        self.assertEqual(
            self.ros.on_cleanup(_STATE), TransitionCallbackReturn.SUCCESS)


if __name__ == '__main__':
    unittest.main()
