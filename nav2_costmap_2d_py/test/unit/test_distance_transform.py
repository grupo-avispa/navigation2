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

"""Tests for the DistanceTransform (Felzenszwalb-Huttenlocher)."""

import math

from nav2_costmap_2d_py.core.distance_transform import DistanceTransform
import numpy as np


def test_single_obstacle_center() -> None:
    """A single obstacle yields exact Euclidean distances."""
    h, w = 5, 5
    img = np.full((h, w), DistanceTransform.DT_INF, dtype=np.float32)
    img[2, 2] = 0.0
    DistanceTransform.distance_transform_2d(img, h, w)

    assert abs(img[2, 2] - 0.0) < 1e-4
    assert abs(img[2, 3] - 1.0) < 1e-4
    assert abs(img[3, 3] - math.sqrt(2.0)) < 1e-4
    assert abs(img[0, 0] - math.sqrt(8.0)) < 1e-4


def test_two_obstacles() -> None:
    """Each cell takes the distance to the nearest obstacle."""
    h, w = 1, 5
    img = np.full((h, w), DistanceTransform.DT_INF, dtype=np.float32)
    img[0, 0] = 0.0
    img[0, 4] = 0.0
    DistanceTransform.distance_transform_2d(img, h, w)

    expected = [0.0, 1.0, 2.0, 1.0, 0.0]
    for x in range(w):
        assert abs(img[0, x] - expected[x]) < 1e-4
