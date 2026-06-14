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
Unit tests for the RangeSensorLayer sensor model.

Mirrors nav2_costmap_2d/test/integration/range_tests.cpp: the probabilistic
sensor model and its helpers map ranges/angles to occupancy probabilities.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import LETHAL_OBSTACLE
from nav2_costmap_2d_py.plugins.range_sensor_layer import RangeSensorLayer


class TestRangeSensorLayer(unittest.TestCase):
    """Test suite for the RangeSensorLayer sensor model."""

    def setUp(self) -> None:
        """Build a range layer and set the sensor-model parameters directly."""
        self.layer = RangeSensorLayer()
        self.layer._max_angle = 0.5
        self.layer._phi_v = 1.2
        self.layer._resolution = 0.1

    def test_to_prob_and_to_cost(self) -> None:
        """to_prob and to_cost are inverse maps over [0, LETHAL_OBSTACLE]."""
        self.assertAlmostEqual(self.layer.to_prob(LETHAL_OBSTACLE), 1.0)
        self.assertAlmostEqual(self.layer.to_prob(0), 0.0)
        self.assertEqual(self.layer.to_cost(1.0), LETHAL_OBSTACLE)
        self.assertEqual(self.layer.to_cost(0.5), LETHAL_OBSTACLE // 2)

    def test_gamma(self) -> None:
        """gamma is 1 at the cone centre and 0 at/beyond the edge."""
        self.assertAlmostEqual(self.layer.gamma(0.0), 1.0)
        self.assertAlmostEqual(self.layer.gamma(0.5), 0.0)
        self.assertEqual(self.layer.gamma(1.0), 0.0)  # beyond max_angle

    def test_delta(self) -> None:
        """delta equals 0.5 at phi == phi_v and decreases past it."""
        self.assertAlmostEqual(self.layer.delta(self.layer._phi_v), 0.5)
        self.assertLess(self.layer.delta(2.0), 0.5)

    def test_sensor_model_far(self) -> None:
        """Well beyond the measured range the sensor model returns 0.5 (unknown)."""
        self.assertAlmostEqual(self.layer.sensor_model(1.0, 5.0, 0.0), 0.5)

    def test_area(self) -> None:
        """area returns the triangle area of three points."""
        self.assertAlmostEqual(self.layer.area(0, 0, 4, 0, 0, 3), 6.0)

    def test_orient2d(self) -> None:
        """orient2d returns a positive value for a counter-clockwise turn."""
        self.assertGreater(self.layer.orient2d(0, 0, 1, 0, 0, 1), 0)
        self.assertLess(self.layer.orient2d(0, 0, 1, 0, 0, -1), 0)

    def test_reset_range(self) -> None:
        """reset_range sets the cached bounds to +/- infinity."""
        self.layer.reset_range()
        self.assertEqual(self.layer._min_x, float('inf'))
        self.assertEqual(self.layer._max_x, -float('inf'))

    def test_is_clearable(self) -> None:
        """Range sensor layers are clearable."""
        self.assertTrue(self.layer.is_clearable())


if __name__ == '__main__':
    unittest.main()
