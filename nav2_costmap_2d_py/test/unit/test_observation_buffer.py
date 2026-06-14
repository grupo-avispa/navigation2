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
Unit tests for the ObservationBuffer.

Mirrors nav2_costmap_2d/test/unit/observation_buffer_test.cpp: the buffer keeps
observations current and purges stale ones.
"""

import unittest

from nav2_costmap_2d_py.core.observation import Observation
from nav2_costmap_2d_py.core.observation_buffer import ObservationBuffer
import rclpy
from rclpy.node import Node


class TestObservationBuffer(unittest.TestCase):
    """Test suite for ObservationBuffer."""

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
        """Build an observation buffer (keep only the latest observation)."""
        self.node = Node('test_observation_buffer')
        self.buffer = ObservationBuffer(
            self.node, 'topic', 0.0, 0.0, 0.0, 2.0, 2.5, 0.0, 3.0, 0.0,
            None, 'map', '', 0.1)

    def tearDown(self) -> None:
        """Destroy the test node."""
        self.node.destroy_node()

    def test_is_current_no_rate(self) -> None:
        """With expected_update_rate == 0 the buffer is always current."""
        self.assertTrue(self.buffer.is_current())

    def test_get_observations_empty(self) -> None:
        """An empty buffer returns no observations."""
        self.assertEqual(self.buffer.get_observations(), [])

    def test_lock_unlock(self) -> None:
        """Lock and unlock do not raise."""
        self.buffer.lock()
        self.buffer.unlock()

    def test_reset_last_updated(self) -> None:
        """Resetting the last-updated timestamp keeps the buffer current."""
        self.buffer.reset_last_updated()
        self.assertTrue(self.buffer.is_current())

    def test_purge_keeps_only_latest(self) -> None:
        """With keep_time == 0 only the most recent observation is retained."""
        self.buffer._observation_list = [Observation(), Observation(), Observation()]
        observations = self.buffer.get_observations()
        self.assertEqual(len(observations), 1)


if __name__ == '__main__':
    unittest.main()
