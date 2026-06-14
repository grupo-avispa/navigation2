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

"""Tests for FootprintCollisionChecker, replicating footprint_collision_checker_test.cpp."""

import math

from geometry_msgs.msg import Point
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.footprint import (
    calculate_min_and_max_distances,
    make_footprint_from_string,
    to_point32,
    to_polygon,
)
from nav2_costmap_2d_py.core.footprint_collision_checker import FootprintCollisionChecker


def test_basic() -> None:
    """A diamond footprint over free space scores 0."""
    costmap = Costmap2D(100, 100, 0.1, 0.0, 0.0, 0)
    footprint = [(-0.5, 0.0), (0.0, 0.5), (0.5, 0.0), (0.0, -0.5)]
    checker = FootprintCollisionChecker(costmap)
    value = checker.footprint_cost_at_pose(5.0, 5.0, 0.0, footprint)
    assert abs(value - 0.0) < 0.001


def test_point_cost() -> None:
    """A point over free space scores 0."""
    costmap = Costmap2D(100, 100, 0.1, 0.0, 0.0, 0)
    checker = FootprintCollisionChecker(costmap)
    assert abs(checker.point_cost(50, 50) - 0.0) < 0.001


def test_world_to_map() -> None:
    """world_to_map then point_cost reflects the cell value."""
    costmap = Costmap2D(100, 100, 0.1, 0.0, 0.0, 0)
    checker = FootprintCollisionChecker(costmap)

    _, x, y = checker.world_to_map(1.0, 1.0)
    assert abs(checker.point_cost(x, y) - 0.0) < 0.001

    costmap.set_cost(50, 50, 200)
    _, x, y = checker.world_to_map(5.0, 5.0)
    assert abs(checker.point_cost(x, y) - 200.0) < 0.001


def test_footprint_at_pose_with_movement() -> None:
    """Moving the footprint over a lethal border returns lethal cost."""
    costmap = Costmap2D(100, 100, 0.1, 0.0, 0.0, 254)
    for i in range(40, 61):
        for j in range(40, 61):
            costmap.set_cost(i, j, 0)

    footprint = [(-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]
    checker = FootprintCollisionChecker(costmap)

    assert abs(checker.footprint_cost_at_pose(5.0, 5.0, 0.0, footprint) - 0.0) < 0.001
    assert abs(checker.footprint_cost_at_pose(5.0, 4.9, 0.0, footprint) - 254.0) < 0.001
    assert abs(checker.footprint_cost_at_pose(5.0, 5.2, 0.0, footprint) - 254.0) < 0.001


def test_point_and_line_cost() -> None:
    """A footprint outline crossing lethal cells returns lethal cost."""
    costmap = Costmap2D(100, 100, 0.1, 0.0, 0.0, 0.0)
    costmap.set_cost(62, 50, 254)
    costmap.set_cost(39, 60, 254)

    footprint = [(-1.0, 1.0), (1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]
    checker = FootprintCollisionChecker(costmap)

    assert abs(checker.footprint_cost_at_pose(5.0, 5.0, 0.0, footprint) - 0.0) < 0.001
    assert abs(checker.footprint_cost_at_pose(4.9, 5.0, 0.0, footprint) - 254.0) < 0.001
    assert abs(checker.footprint_cost_at_pose(5.2, 5.0, 0.0, footprint) - 254.0) < 0.001


def test_not_enough_points() -> None:
    """A two-point footprint yields (inf, 0.0)."""
    footprint = [(2.0, 2.0), (-2.0, -2.0)]
    min_dist, max_dist = calculate_min_and_max_distances(footprint)
    assert min_dist == math.inf
    assert max_dist == 0.0


def test_to_point_32() -> None:
    """to_point32 preserves the coordinates."""
    p = Point()
    p.x = 123.0
    p.y = 456.0
    p.z = 789.0
    p32 = to_point32(p)
    assert abs(p.x - p32.x) < 1e-5
    assert abs(p.y - p32.y) < 1e-5
    assert abs(p.z - p32.z) < 1e-5


def test_to_polygon() -> None:
    """to_polygon converts a list of (x, y) points to a Polygon (2D)."""
    pts = [(1.2, 3.4), (-5.6, -7.8)]
    poly = to_polygon(pts)
    assert len(poly.points) == 2
    assert abs(poly.points[0].x - 1.2) < 1e-5
    assert abs(poly.points[0].y - 3.4) < 1e-5
    assert abs(poly.points[1].x - (-5.6)) < 1e-5
    assert abs(poly.points[1].y - (-7.8)) < 1e-5


def test_make_footprint_from_string() -> None:
    """A valid footprint string parses into points."""
    footprint = make_footprint_from_string(
        '[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2]]')
    assert len(footprint) == 4
    assert abs(footprint[0][0] - 1.0) < 1e-5
    assert abs(footprint[0][1] - 2.2) < 1e-5
    assert abs(footprint[1][0] - 0.3) < 1e-5
    assert abs(footprint[1][1] - (-4e4)) < 1e-5
    assert abs(footprint[2][0] - (-0.3)) < 1e-5
    assert abs(footprint[2][1] - (-4e4)) < 1e-5
    assert abs(footprint[3][0] - (-1.0)) < 1e-5
    assert abs(footprint[3][1] - 2.2) < 1e-5


def test_make_footprint_from_string_parse_error() -> None:
    """A malformed footprint string returns an empty list."""
    assert make_footprint_from_string('[[bad_string') == []


def test_make_footprint_from_string_two_points_error() -> None:
    """A footprint string with fewer than three points returns an empty list."""
    assert make_footprint_from_string('[[1, 2.2], [.3, -4e4]') == []


def test_make_footprint_from_string_not_pairs() -> None:
    """A footprint string with a non-pair point returns an empty list."""
    assert make_footprint_from_string(
        '[[1, 2.2], [.3, -4e4], [-.3, -4e4], [-1, 2.2, 5.6]]') == []
