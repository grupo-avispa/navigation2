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

"""Tests for the footprint helper functions."""

import math

from nav2_costmap_2d_py.core.footprint import (
    calculate_min_and_max_distances,
    make_footprint_from_radius,
    pad_footprint,
    transform_footprint,
)


def test_make_footprint_from_radius() -> None:
    """A circular footprint has 16 points all at the given radius."""
    radius = 1.5
    footprint = make_footprint_from_radius(radius)
    assert len(footprint) == 16
    for px, py in footprint:
        assert abs(math.hypot(px, py) - radius) < 1e-6


def test_pad_footprint() -> None:
    """Padding pushes each coordinate outward by the padding amount."""
    footprint = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
    padded = pad_footprint(footprint, 0.5)
    assert padded[0] == (1.5, 1.5)
    assert padded[1] == (-1.5, 1.5)
    assert padded[2] == (-1.5, -1.5)
    assert padded[3] == (1.5, -1.5)


def test_transform_footprint() -> None:
    """Transforming by a translation shifts every point."""
    footprint = [(1.0, 0.0), (0.0, 1.0)]
    out = transform_footprint(2.0, 3.0, 0.0, footprint)
    assert abs(out[0][0] - 3.0) < 1e-9
    assert abs(out[0][1] - 3.0) < 1e-9
    assert abs(out[1][0] - 2.0) < 1e-9
    assert abs(out[1][1] - 4.0) < 1e-9


def test_transform_footprint_rotation() -> None:
    """Rotating by 90 degrees maps (1, 0) to (0, 1)."""
    footprint = [(1.0, 0.0)]
    out = transform_footprint(0.0, 0.0, math.pi / 2, footprint)
    assert abs(out[0][0] - 0.0) < 1e-9
    assert abs(out[0][1] - 1.0) < 1e-9


def test_calculate_min_and_max_distances_square() -> None:
    """A unit square has inscribed 1.0 and circumscribed sqrt(2)."""
    footprint = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
    min_dist, max_dist = calculate_min_and_max_distances(footprint)
    assert abs(min_dist - 1.0) < 1e-9
    assert abs(max_dist - math.hypot(1.0, 1.0)) < 1e-9
