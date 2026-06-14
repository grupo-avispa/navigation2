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
Observation for nav2_costmap_2d_py.

Stores an observation in terms of a point cloud and the origin of the source.
It mirrors the nav2_costmap_2d::Observation from the C++ implementation.
"""

from typing import Optional

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2


class Observation:
    """Stores an observation in terms of a point cloud and the origin of the source."""

    def __init__(
        self,
        origin: Optional[Point] = None,
        cloud: Optional[PointCloud2] = None,
        obstacle_max_range: float = 0.0,
        obstacle_min_range: float = 0.0,
        raytrace_max_range: float = 0.0,
        raytrace_min_range: float = 0.0,
    ) -> None:
        """
        Create an observation from an origin point and a point cloud.

        Parameters
        ----------
        origin : geometry_msgs.msg.Point, optional
            The origin point of the observation.
        cloud : sensor_msgs.msg.PointCloud2, optional
            The point cloud of the observation.
        obstacle_max_range : float
            The range out to which an observation should insert obstacles.
        obstacle_min_range : float
            The range from which an observation should insert obstacles.
        raytrace_max_range : float
            The range out to which an observation should clear via raytracing.
        raytrace_min_range : float
            The range from which an observation should clear via raytracing.

        """
        self.origin: Point = origin if origin is not None else Point()
        self.cloud: PointCloud2 = cloud if cloud is not None else PointCloud2()
        self.obstacle_max_range: float = obstacle_max_range
        self.obstacle_min_range: float = obstacle_min_range
        self.raytrace_max_range: float = raytrace_max_range
        self.raytrace_min_range: float = raytrace_min_range
