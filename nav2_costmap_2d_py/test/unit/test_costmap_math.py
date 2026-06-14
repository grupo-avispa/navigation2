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

"""Tests for the costmap_math helpers."""

import math

from nav2_costmap_2d_py.core.costmap_math import distance, distance_to_line, sign, sign0


def test_sign() -> None:
    """sign returns -1 for negatives and +1 otherwise."""
    assert sign(-3.0) == -1.0
    assert sign(0.0) == 1.0
    assert sign(2.5) == 1.0


def test_sign0() -> None:
    """sign0 returns -1, 0 or +1."""
    assert sign0(-3.0) == -1.0
    assert sign0(0.0) == 0.0
    assert sign0(2.5) == 1.0


def test_distance() -> None:
    """distance returns the Euclidean distance."""
    assert abs(distance(0.0, 0.0, 3.0, 4.0) - 5.0) < 1e-9


def test_distance_to_line() -> None:
    """distance_to_line returns the shortest distance to a segment."""
    # Point above the middle of a horizontal segment.
    assert abs(distance_to_line(1.0, 1.0, 0.0, 0.0, 2.0, 0.0) - 1.0) < 1e-9
    # Point beyond the segment end falls back to the endpoint distance.
    assert abs(distance_to_line(3.0, 0.0, 0.0, 0.0, 2.0, 0.0) - 1.0) < 1e-9
    # Point at a diagonal from an endpoint.
    assert abs(
        distance_to_line(-1.0, -1.0, 0.0, 0.0, 2.0, 0.0) - math.hypot(1.0, 1.0)) < 1e-9
