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
DistanceTransform for nav2_costmap_2d_py.

Efficient Euclidean distance transform using the Felzenszwalb-Huttenlocher
linear-time algorithm.
It mirrors the nav2_costmap_2d::DistanceTransform from the C++ implementation.

Reference: Distance Transforms of Sampled Functions, P. Felzenszwalb and
D. Huttenlocher, Theory of Computing, Vol. 8, No. 19, September 2012.
"""

import math

import numpy as np


class DistanceTransform:
    """Euclidean distance transform via the Felzenszwalb-Huttenlocher algorithm."""

    # Infinity constant for distance transform.
    DT_INF: float = float(np.finfo(np.float32).max)

    @staticmethod
    def distance_transform_1d(f: np.ndarray, d: np.ndarray, n: int,
                              v: np.ndarray, z: np.ndarray) -> None:
        """
        Perform a 1D distance transform using a lower envelope of parabolas.

        Core Felzenszwalb-Huttenlocher algorithm computing squared Euclidean
        distances in 1D in linear time.

        Parameters
        ----------
        f : numpy.ndarray
            Input array of squared distances (0 for obstacles, INF for free
            space).
        d : numpy.ndarray
            Output array for the transformed squared distances (same size as f).
        n : int
            Length of the arrays.
        v : numpy.ndarray
            Buffer for parabola indices (size n).
        z : numpy.ndarray
            Buffer for parabola boundaries (size n + 1).

        """
        if f is None or d is None or v is None or z is None or n <= 0:
            return

        inf = DistanceTransform.DT_INF
        k = 0
        v[0] = 0
        z[0] = -inf
        z[1] = inf

        for q in range(1, n):
            s = (f[q] - f[v[k]] + float(q * q - v[k] * v[k])) / (2.0 * float(q - v[k]))
            while s <= z[k]:
                k -= 1
                s = (f[q] - f[v[k]] + float(q * q - v[k] * v[k])) / (2.0 * float(q - v[k]))
            k += 1
            v[k] = q
            z[k] = s
            z[k + 1] = inf

        k = 0
        for q in range(n):
            while z[k + 1] < float(q):
                k += 1
            diff = q - v[k]
            d[q] = float(diff * diff) + f[v[k]]

    @staticmethod
    def distance_transform_2d(img: np.ndarray, height: int, width: int) -> None:
        """
        Perform a 2D Euclidean distance transform using separable passes.

        Applies the 1D transform along columns then rows. ``img`` is modified in
        place; input values should be 0 for obstacles and ``DT_INF`` for free
        space, and the output contains the Euclidean distances.

        Parameters
        ----------
        img : numpy.ndarray
            Input/output matrix (modified in place), shape ``(height, width)``.
        height : int
            Number of rows in the matrix.
        width : int
            Number of columns in the matrix.

        """
        # Column pass.
        for x in range(width):
            f = np.empty(height, dtype=np.float32)
            d = np.empty(height, dtype=np.float32)
            v = np.zeros(height, dtype=np.int32)
            z = np.empty(height + 1, dtype=np.float32)
            for y in range(height):
                f[y] = img[y, x]
            DistanceTransform.distance_transform_1d(f, d, height, v, z)
            for y in range(height):
                img[y, x] = d[y]

        # Row pass.
        for y in range(height):
            f = np.empty(width, dtype=np.float32)
            d = np.empty(width, dtype=np.float32)
            v = np.zeros(width, dtype=np.int32)
            z = np.empty(width + 1, dtype=np.float32)
            for x in range(width):
                f[x] = img[y, x]
            DistanceTransform.distance_transform_1d(f, d, width, v, z)
            for x in range(width):
                img[y, x] = d[x]

        # Square root to get Euclidean distance (not squared distance).
        for y in range(height):
            for x in range(width):
                img[y, x] = math.sqrt(img[y, x])
