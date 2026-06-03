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
Unit tests for the Costmap2D coordinate transforms.

Python port of nav2_costmap_2d/test/unit/coordinate_transform_test.cpp.
"""

import unittest

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D


class TestCoordinateTransform(unittest.TestCase):
    """Test suite for Costmap2D coordinate transforms."""

    def test_map_to_world_negative_map_coords(self) -> None:
        """
        Mirror coordinate_transform_test.cpp MapToWorldNoBoundsNegativeMapCoords.

        ``map_to_world`` returns the centre of the cell and is allowed to
        produce negative world coordinates for negative map coordinates,
        matching the C++ ``mapToWorldNoBounds``.
        """
        # 10x10, resolution 1.0, origin (0, 0)
        cmap = Costmap2D(10, 10, 1.0, 0.0, 0.0)
        wx, wy = cmap.map_to_world(-1, -1)
        self.assertAlmostEqual(wx, -0.5)
        self.assertAlmostEqual(wy, -0.5)

        # 10x10, resolution 1.0, origin (1, 2)
        cmap = Costmap2D(10, 10, 1.0, 1.0, 2.0)
        wx, wy = cmap.map_to_world(-5, -5)
        self.assertAlmostEqual(wx, -3.5)
        self.assertAlmostEqual(wy, -2.5)

        # 10x10, resolution 2.0, origin (3, 4)
        cmap = Costmap2D(10, 10, 2.0, 3.0, 4.0)
        wx, wy = cmap.map_to_world(-10, -10)
        self.assertAlmostEqual(wx, -16.0)
        self.assertAlmostEqual(wy, -15.0)


if __name__ == '__main__':
    unittest.main()
