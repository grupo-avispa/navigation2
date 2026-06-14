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
Unit tests for the CostmapFilter enable/disable service callback.

Mirrors nav2_costmap_2d/test/unit/costmap_filter_service_test.cpp: the
``toggle_filter`` SetBool service enables/disables the filter.
"""

import unittest

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.filters.costmap_filter import CostmapFilter
from std_srvs.srv import SetBool


class _FilterWrapper(CostmapFilter):
    """Concrete CostmapFilter with no-op hooks, for testing the enable service."""

    def initialize_filter(self, filter_info_topic: str) -> None:
        """No-op filter initialization."""
        pass

    def process(self, master_grid: Costmap2D, min_i: int, min_j: int,
                max_i: int, max_j: int, pose: object) -> None:
        """No-op process."""
        pass

    def reset_filter(self) -> None:
        """No-op reset."""
        pass


class TestCostmapFilterService(unittest.TestCase):
    """Test suite for the CostmapFilter enable/disable callback."""

    def setUp(self) -> None:
        """Build a costmap filter wrapper."""
        self.filter = _FilterWrapper()

    def test_enable(self) -> None:
        """A SetBool request with data=True enables the filter."""
        req = SetBool.Request()
        req.data = True
        resp = self.filter.enable_callback(req, SetBool.Response())
        self.assertTrue(resp.success)
        self.assertEqual(resp.message, 'Enabled')
        self.assertTrue(self.filter._enabled)

    def test_disable(self) -> None:
        """A SetBool request with data=False disables the filter."""
        req = SetBool.Request()
        req.data = False
        resp = self.filter.enable_callback(req, SetBool.Response())
        self.assertTrue(resp.success)
        self.assertEqual(resp.message, 'Disabled')
        self.assertFalse(self.filter._enabled)


if __name__ == '__main__':
    unittest.main()
