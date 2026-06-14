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

"""Tests for OccupancyGrid -> Costmap2D conversion, replicating costmap_conversion_test.cpp."""

from nav2_costmap_2d_py.core.cost_values import NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D, OCC_GRID_OCCUPIED, OCC_GRID_UNKNOWN
from nav_msgs.msg import OccupancyGrid

RESOLUTION = 0.05
ORIGIN_X = 0.1
ORIGIN_Y = 0.2
EPSILON = 1e-6


def _create_occ_grid() -> OccupancyGrid:
    """Build a 4x3 OccupancyGrid with data[i] = i * 10 (clamped to unknown)."""
    occ_grid = OccupancyGrid()
    width = 4
    height = 3
    occ_grid.info.resolution = RESOLUTION
    occ_grid.info.width = width
    occ_grid.info.height = height
    occ_grid.info.origin.position.x = ORIGIN_X
    occ_grid.info.origin.position.y = ORIGIN_Y
    occ_grid.info.origin.orientation.w = 1.0
    data = []
    for i in range(width * height):
        value = i * 10
        data.append(value if value <= OCC_GRID_OCCUPIED else OCC_GRID_UNKNOWN)
    occ_grid.data = data
    return occ_grid


def test_convert_occ_grid_to_costmap() -> None:
    """Converting an OccupancyGrid to a Costmap2D preserves info and scales the data."""
    occ_grid = _create_occ_grid()
    costmap = Costmap2D.from_occupancy_grid(occ_grid)

    assert abs(costmap.get_resolution() - RESOLUTION) < EPSILON
    assert abs(costmap.get_origin_x() - ORIGIN_X) < EPSILON
    assert abs(costmap.get_origin_y() - ORIGIN_Y) < EPSILON

    size = costmap.get_size_in_cells_x() * costmap.get_size_in_cells_y()
    char_map = costmap.get_char_map()
    for it in range(size - 1):
        # data[it] = it * 10 (all <= 100 for it in 0..10), scaled by 254/100.
        data_ref = round(254 * it / 10)
        assert char_map[it] == data_ref

    # The last cell (index 11, value 110) is clamped to unknown.
    assert char_map[size - 1] == NO_INFORMATION
