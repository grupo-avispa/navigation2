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

"""Tests for the denoise image-processing group removal."""

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.denoise.image import Image
from nav2_costmap_2d_py.core.denoise.image_processing import (
    ConnectivityType,
    GroupsRemover,
    MemoryBuffer,
)
import numpy as np


def _is_bg(pixel: int) -> bool:
    """Background = anything that is not a lethal obstacle."""
    return pixel != LETHAL_OBSTACLE


def test_remove_isolated_pixel() -> None:
    """An isolated obstacle pixel is removed when minimal_group_size = 2."""
    grid = np.full((5, 5), FREE_SPACE, dtype=np.uint8)
    grid[2, 2] = LETHAL_OBSTACLE
    GroupsRemover().remove_groups(
        Image(grid), MemoryBuffer(), ConnectivityType.Way8, 2, _is_bg)
    assert grid[2, 2] == FREE_SPACE


def test_keep_large_group() -> None:
    """A group of obstacles larger than the threshold is preserved."""
    grid = np.full((5, 5), FREE_SPACE, dtype=np.uint8)
    grid[1, 1] = LETHAL_OBSTACLE
    grid[1, 2] = LETHAL_OBSTACLE
    grid[2, 1] = LETHAL_OBSTACLE
    GroupsRemover().remove_groups(
        Image(grid), MemoryBuffer(), ConnectivityType.Way8, 3, _is_bg)
    assert grid[1, 1] == LETHAL_OBSTACLE
    assert grid[1, 2] == LETHAL_OBSTACLE
    assert grid[2, 1] == LETHAL_OBSTACLE


def test_remove_small_group_keep_large() -> None:
    """A small group is removed while a larger one is kept."""
    grid = np.full((6, 6), FREE_SPACE, dtype=np.uint8)
    # Small group of size 2.
    grid[0, 0] = LETHAL_OBSTACLE
    grid[0, 1] = LETHAL_OBSTACLE
    # Large group of size 4.
    grid[3, 3] = LETHAL_OBSTACLE
    grid[3, 4] = LETHAL_OBSTACLE
    grid[4, 3] = LETHAL_OBSTACLE
    grid[4, 4] = LETHAL_OBSTACLE
    GroupsRemover().remove_groups(
        Image(grid), MemoryBuffer(), ConnectivityType.Way4, 3, _is_bg)
    assert grid[0, 0] == FREE_SPACE
    assert grid[0, 1] == FREE_SPACE
    assert grid[3, 3] == LETHAL_OBSTACLE
    assert grid[4, 4] == LETHAL_OBSTACLE


def test_way4_vs_way8_connectivity() -> None:
    """Diagonal neighbours form one group under Way8 but two under Way4."""
    grid = np.full((4, 4), FREE_SPACE, dtype=np.uint8)
    grid[1, 1] = LETHAL_OBSTACLE
    grid[2, 2] = LETHAL_OBSTACLE
    # Under Way8 they are one group of size 2 -> kept with threshold 2.
    GroupsRemover().remove_groups(
        Image(grid), MemoryBuffer(), ConnectivityType.Way8, 2, _is_bg)
    assert grid[1, 1] == LETHAL_OBSTACLE
    assert grid[2, 2] == LETHAL_OBSTACLE

    grid2 = np.full((4, 4), FREE_SPACE, dtype=np.uint8)
    grid2[1, 1] = LETHAL_OBSTACLE
    grid2[2, 2] = LETHAL_OBSTACLE
    # Under Way4 they are two isolated pixels -> both removed with threshold 2.
    GroupsRemover().remove_groups(
        Image(grid2), MemoryBuffer(), ConnectivityType.Way4, 2, _is_bg)
    assert grid2[1, 1] == FREE_SPACE
    assert grid2[2, 2] == FREE_SPACE
