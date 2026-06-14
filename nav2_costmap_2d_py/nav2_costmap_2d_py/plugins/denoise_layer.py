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
DenoiseLayer for nav2_costmap_2d_py.

Layer that filters noise-induced standalone obstacles (single costmap pixels) or
small obstacle groups.

It mirrors the nav2_costmap_2d::DenoiseLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/DenoiseLayer"``
"""

from typing import List

from nav2_costmap_2d_py.core.cost_values import (
    INSCRIBED_INFLATED_OBSTACLE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.denoise.image import Image
from nav2_costmap_2d_py.core.denoise.image_processing import (
    ConnectivityType,
    GroupsRemover,
    MemoryBuffer,
)
from nav2_costmap_2d_py.core.layer import Layer
import numpy as np


class DenoiseLayer(Layer):
    """Filters noise-induced standalone obstacles or small obstacle groups."""

    def __init__(self) -> None:
        """Initialize denoise layer defaults."""
        super().__init__()
        # The border value of group size. Groups of this and larger size are kept.
        self._minimal_group_size = 2
        # Pixel connectivity type.
        self._group_connectivity_type = ConnectivityType.Way8
        self._buffer = MemoryBuffer()
        self._groups_remover = GroupsRemover()
        # Interpret NO_INFORMATION as an obstacle.
        self._no_information_is_obstacle = False

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer on startup: read the denoise parameters."""
        node = self._node
        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        minimal_group_size_param = self._declare_parameter_if_not_declared(
            'minimal_group_size', 2)
        group_connectivity_type_param = self._declare_parameter_if_not_declared(
            'group_connectivity_type', 8)

        if minimal_group_size_param <= 1:
            node.get_logger().warning(
                f'DenoiseLayer: minimal_group_size: {minimal_group_size_param}. '
                'A value of 1 or less leaves all map cells as they are.')
            self._minimal_group_size = 1
        else:
            self._minimal_group_size = int(minimal_group_size_param)

        if group_connectivity_type_param == 4:
            self._group_connectivity_type = ConnectivityType.Way4
        else:
            self._group_connectivity_type = ConnectivityType.Way8
            if group_connectivity_type_param != 8:
                node.get_logger().warning(
                    f'DenoiseLayer: group_connectivity_type: '
                    f'{group_connectivity_type_param}. Valid values are 4 or 8. '
                    'Using the default value 8.')

        self.set_current(True)

    def reset(self) -> None:
        """Reset this layer (mark it not current)."""
        self.set_current(False)

    def is_clearable(self) -> bool:
        """Report that no clearing operation is required."""
        return False

    def update_bounds(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """Report that no expansion is required (a filter never expands bounds)."""
        pass

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_x: int,
        min_y: int,
        max_x: int,
        max_y: int,
    ) -> None:
        """Filter noise-induced obstacles in the selected region of the costmap."""
        if not self._enabled:
            return
        if min_x >= max_x or min_y >= max_y:
            return

        self._no_information_is_obstacle = master_grid.default_value != NO_INFORMATION

        grid = np.frombuffer(master_grid.get_char_map(), dtype=np.uint8).reshape(
            master_grid.size_y, master_grid.size_x)
        roi = grid[min_y:max_y, min_x:max_x]
        try:
            self.denoise(Image(roi))
        except Exception as ex:  # noqa: BLE001
            self._node.get_logger().error(f'Inner error: {ex}')

        self.set_current(True)

    # ------------------------------------------------------------------
    # Denoise implementation
    # ------------------------------------------------------------------

    def denoise(self, image: Image) -> None:
        """
        Remove single obstacles or small obstacle groups from the image.

        Parameters
        ----------
        image : Image
            The region of the costmap to filter (modified in place).

        """
        if image.empty():
            return
        if self._minimal_group_size <= 1:
            # A smaller group cannot exist. No pixel will be changed.
            return
        if self._minimal_group_size == 2:
            self.remove_single_pixels(image)
        else:
            self.remove_groups(image)

    def remove_groups(self, image: Image) -> None:
        """Remove obstacle groups smaller than ``minimal_group_size``."""
        self._groups_remover.remove_groups(
            image, self._buffer, self._group_connectivity_type,
            self._minimal_group_size, self.is_background)

    def remove_single_pixels(self, image: Image) -> None:
        """Remove freestanding single obstacle pixels (groups of size 1)."""
        self._groups_remover.remove_groups(
            image, self._buffer, self._group_connectivity_type,
            2, self.is_background)

    def is_background(self, pixel: int) -> bool:
        """Return True if the pixel value is not an obstacle code."""
        is_obstacle = (
            pixel == LETHAL_OBSTACLE
            or pixel == INSCRIBED_INFLATED_OBSTACLE
            or (pixel == NO_INFORMATION and self._no_information_is_obstacle))
        return not is_obstacle
