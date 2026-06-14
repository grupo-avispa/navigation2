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
Image for nav2_costmap_2d_py.

Lightweight 2D image wrapper over a numpy array, mirroring the
nav2_costmap_2d::Image<T> from the C++ implementation. Used by the denoise layer.
"""

import numpy as np


class Image:
    """A 2D image backed by a numpy array (row-major)."""

    def __init__(self, data: np.ndarray) -> None:
        """
        Construct an image around a 2D numpy array.

        Parameters
        ----------
        data : numpy.ndarray
            The backing 2D array (shape ``(rows, columns)``). The array is used
            as a view, so in-place edits affect the original buffer.

        """
        self._data = data

    def rows(self) -> int:
        """Return the number of rows in the image."""
        return int(self._data.shape[0])

    def columns(self) -> int:
        """Return the number of columns in the image."""
        return int(self._data.shape[1])

    def empty(self) -> bool:
        """Return whether the image has no pixels."""
        return self._data.size == 0

    @property
    def data(self) -> np.ndarray:
        """Return the backing numpy array."""
        return self._data
