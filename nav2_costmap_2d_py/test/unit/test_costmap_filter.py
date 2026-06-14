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
Unit tests for the CostmapFilter base helpers.

Mirrors nav2_costmap_2d/test/unit/costmap_filter_test.cpp: the mask
world-to-map conversion and the mask-cost lookup.
"""

import unittest

from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.filters.costmap_filter import CostmapFilter
from nav_msgs.msg import OccupancyGrid


def _make_mask(width, height, resolution, origin, data):
    """Build an OccupancyGrid mask."""
    mask = OccupancyGrid()
    mask.header.frame_id = 'map'
    mask.info.resolution = resolution
    mask.info.width = width
    mask.info.height = height
    mask.info.origin.position.x = origin[0]
    mask.info.origin.position.y = origin[1]
    mask.data = data
    return mask


class TestCostmapFilter(unittest.TestCase):
    """Test suite for the CostmapFilter base helpers."""

    def test_world_to_map(self) -> None:
        """mask_world_to_map maps world points to mask cells with bounds checks."""
        mask = _make_mask(3, 3, 1.0, (3.0, 3.0), [100] * 9)

        ok, mx, my = CostmapFilter.mask_world_to_map(mask, 4.0, 5.0)
        self.assertTrue(ok)
        self.assertEqual((mx, my), (1, 2))

        ok, mx, my = CostmapFilter.mask_world_to_map(mask, 3.0, 3.0)
        self.assertTrue(ok)
        self.assertEqual((mx, my), (0, 0))

        ok, mx, my = CostmapFilter.mask_world_to_map(mask, 5.9, 5.9)
        self.assertTrue(ok)
        self.assertEqual((mx, my), (2, 2))

        ok, _, _ = CostmapFilter.mask_world_to_map(mask, 2.9, 2.9)
        self.assertFalse(ok)
        ok, _, _ = CostmapFilter.mask_world_to_map(mask, 6.0, 6.0)
        self.assertFalse(ok)

    def test_get_mask_cost(self) -> None:
        """get_mask_cost converts occupancy values into costmap costs."""
        # Mask data laid out as [-1, 0, 50, 100].
        mask = _make_mask(2, 2, 1.0, (0.0, 0.0), [-1, 0, 50, 100])
        self.assertEqual(CostmapFilter.get_mask_cost(mask, 0, 0), NO_INFORMATION)
        self.assertEqual(CostmapFilter.get_mask_cost(mask, 1, 0), FREE_SPACE)
        self.assertEqual(
            CostmapFilter.get_mask_cost(mask, 0, 1), LETHAL_OBSTACLE // 2)
        self.assertEqual(CostmapFilter.get_mask_cost(mask, 1, 1), LETHAL_OBSTACLE)


if __name__ == '__main__':
    unittest.main()
