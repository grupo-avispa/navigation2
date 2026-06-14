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
VoxelGrid for nav2_costmap_2d_py.

A numpy-based 3D occupancy grid mirroring the nav2_voxel_grid::VoxelGrid used by
the VoxelLayer: each voxel is UNKNOWN, MARKED or FREE.
"""

from typing import Iterator, Tuple

import numpy as np

UNKNOWN = 0
MARKED = 1
FREE = 2


def _bresenham_3d(
    x0: int, y0: int, z0: int, x1: int, y1: int, z1: int
) -> Iterator[Tuple[int, int, int]]:
    """Yield the integer cells along the 3D line ``(x0,y0,z0)``..``(x1,y1,z1)``."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    dz = abs(z1 - z0)
    xs = 1 if x1 > x0 else -1
    ys = 1 if y1 > y0 else -1
    zs = 1 if z1 > z0 else -1
    x, y, z = x0, y0, z0

    if dx >= dy and dx >= dz:
        p1 = 2 * dy - dx
        p2 = 2 * dz - dx
        for _ in range(dx):
            yield x, y, z
            if p1 >= 0:
                y += ys
                p1 -= 2 * dx
            if p2 >= 0:
                z += zs
                p2 -= 2 * dx
            p1 += 2 * dy
            p2 += 2 * dz
            x += xs
    elif dy >= dx and dy >= dz:
        p1 = 2 * dx - dy
        p2 = 2 * dz - dy
        for _ in range(dy):
            yield x, y, z
            if p1 >= 0:
                x += xs
                p1 -= 2 * dy
            if p2 >= 0:
                z += zs
                p2 -= 2 * dy
            p1 += 2 * dx
            p2 += 2 * dz
            y += ys
    else:
        p1 = 2 * dy - dz
        p2 = 2 * dx - dz
        for _ in range(dz):
            yield x, y, z
            if p1 >= 0:
                y += ys
                p1 -= 2 * dz
            if p2 >= 0:
                x += xs
                p2 -= 2 * dz
            p1 += 2 * dy
            p2 += 2 * dx
            z += zs
    yield x, y, z


class VoxelGrid:
    """A 3D occupancy grid of UNKNOWN/MARKED/FREE voxels."""

    def __init__(self, size_x: int, size_y: int, size_z: int) -> None:
        """Construct a voxel grid of the given dimensions."""
        self.resize(size_x, size_y, size_z)

    def resize(self, size_x: int, size_y: int, size_z: int) -> None:
        """Resize the grid (and reset it to all-unknown)."""
        self._size_x = size_x
        self._size_y = size_y
        self._size_z = size_z
        self._data = np.full((size_y, size_x, max(size_z, 1)), UNKNOWN, dtype=np.uint8)

    def reset(self) -> None:
        """Reset every voxel to UNKNOWN."""
        self._data.fill(UNKNOWN)

    def size_x(self) -> int:
        """Return the x size, in voxels."""
        return self._size_x

    def size_y(self) -> int:
        """Return the y size, in voxels."""
        return self._size_y

    def size_z(self) -> int:
        """Return the z size, in voxels."""
        return self._size_z

    def get_data(self) -> np.ndarray:
        """Return the backing 3D numpy array (shape ``(size_y, size_x, size_z)``)."""
        return self._data

    def mark_voxel_in_map(
        self, mx: int, my: int, mz: int, marked_threshold: int
    ) -> bool:
        """
        Mark a voxel and report whether the column should be a 2D obstacle.

        Parameters
        ----------
        mx, my, mz : int
            The voxel coordinates.
        marked_threshold : int
            The minimum number of marked voxels in the column for the 2D cell to
            be considered an obstacle.

        Returns
        -------
        bool
            True if the number of marked voxels in the column exceeds
            ``marked_threshold``.

        """
        if not (0 <= mx < self._size_x and 0 <= my < self._size_y
                and 0 <= mz < self._size_z):
            return False
        self._data[my, mx, mz] = MARKED
        marked = int(np.count_nonzero(self._data[my, mx, :] == MARKED))
        return marked > marked_threshold

    def clear_voxel_line_in_map(
        self,
        x0: float, y0: float, z0: float,
        x1: float, y1: float, z1: float,
        costmap: bytearray,
        unknown_threshold: int,
        marked_threshold: int,
        free_cost: int,
        unknown_cost: int,
        max_length: int = 2 ** 31 - 1,
        min_length: int = 0,
    ) -> None:
        """
        Clear voxels along a 3D line and update the projected 2D costmap cells.

        Parameters
        ----------
        x0, y0, z0 : float
            The start voxel (sensor origin).
        x1, y1, z1 : float
            The end voxel (ray endpoint).
        costmap : bytearray
            The 2D costmap buffer to update (size ``size_x * size_y``).
        unknown_threshold : int
            Threshold of unknown voxels above which a cleared cell stays unknown.
        marked_threshold : int
            Threshold of marked voxels above which a cell stays an obstacle.
        free_cost : int
            The cost to assign to a cleared free cell.
        unknown_cost : int
            The cost to assign to a still-unknown cell.
        max_length : int
            Maximum number of dominant-axis steps to clear.
        min_length : int
            Minimum number of dominant-axis steps before clearing.

        """
        ix0, iy0, iz0 = int(x0), int(y0), int(z0)
        ix1, iy1, iz1 = int(x1), int(y1), int(z1)
        step = 0
        for x, y, z in _bresenham_3d(ix0, iy0, iz0, ix1, iy1, iz1):
            if step > max_length:
                break
            step += 1
            if not (0 <= x < self._size_x and 0 <= y < self._size_y
                    and 0 <= z < self._size_z):
                continue
            if step < min_length:
                continue
            self._data[y, x, z] = FREE

            column = self._data[y, x, :]
            marked = int(np.count_nonzero(column == MARKED))
            unknown = int(np.count_nonzero(column == UNKNOWN))
            index = y * self._size_x + x
            if marked > marked_threshold:
                continue
            if unknown > unknown_threshold:
                costmap[index] = unknown_cost
            else:
                costmap[index] = free_cost
