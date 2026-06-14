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
Image processing for nav2_costmap_2d_py.

Connected-components helpers used by the denoise layer, mirroring the
nav2_costmap_2d::imgproc_impl utilities (``ConnectivityType``, ``MemoryBuffer``,
``GroupsRemover``) from the C++ implementation.
"""

import enum
from typing import Callable

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE
from nav2_costmap_2d_py.core.denoise.image import Image
import numpy as np
from scipy import ndimage


class ConnectivityType(enum.IntEnum):
    """Pixel connectivity type (how neighbouring pixels are connected)."""

    # Neighbours connected horizontally and vertically.
    Way4 = 4
    # Neighbours connected horizontally, vertically and diagonally.
    Way8 = 8


class MemoryBuffer:
    """Reusable scratch buffer (kept for API parity with the C++ implementation)."""

    def __init__(self) -> None:
        """Initialize an empty buffer."""
        self._data: np.ndarray = np.empty(0, dtype=np.uint8)

    def get(self, count: int) -> np.ndarray:
        """Return a buffer of at least ``count`` ``uint8`` elements."""
        if self._data.size < count:
            self._data = np.empty(count, dtype=np.uint8)
        return self._data[:count]


def _structure(connectivity: ConnectivityType) -> np.ndarray:
    """Return the 3x3 binary structuring element for the given connectivity."""
    if connectivity == ConnectivityType.Way4:
        return np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
    return np.ones((3, 3), dtype=bool)


class GroupsRemover:
    """Removes groups of connected obstacle pixels smaller than a threshold."""

    def remove_groups(
        self,
        image: Image,
        buffer: MemoryBuffer,
        connectivity: ConnectivityType,
        minimal_group_size: int,
        is_bg: Callable[[int], bool],
    ) -> None:
        """
        Remove obstacle groups smaller than ``minimal_group_size`` (set them to free space).

        Parameters
        ----------
        image : Image
            The image to filter (modified in place).
        buffer : MemoryBuffer
            Scratch buffer (unused in the numpy implementation, kept for parity).
        connectivity : ConnectivityType
            How obstacle pixels are considered connected.
        minimal_group_size : int
            Groups smaller than this size are removed.
        is_bg : callable
            Predicate returning True if a pixel value is background (free).

        """
        del buffer
        arr = image.data
        if arr.size == 0:
            return

        # Vectorise the background predicate over all 256 possible byte values.
        obstacle_lut = np.array([not is_bg(v) for v in range(256)], dtype=bool)
        obstacle = obstacle_lut[arr]

        labeled, num = ndimage.label(obstacle, structure=_structure(connectivity))
        if num == 0:
            return

        counts = np.bincount(labeled.ravel())
        # Label 0 is the background; never remove it.
        small_labels = np.nonzero(counts < minimal_group_size)[0]
        small_labels = small_labels[small_labels != 0]
        if small_labels.size == 0:
            return
        remove_mask = np.isin(labeled, small_labels)
        arr[remove_mask] = FREE_SPACE
