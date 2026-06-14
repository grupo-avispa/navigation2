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
CostmapTopicCollisionChecker for nav2_costmap_2d_py.

Uses a costmap from a ROS topic to find if robot poses are in collision with the
costmap environment.
It mirrors the nav2_costmap_2d::CostmapTopicCollisionChecker from the C++
implementation.
"""

import math
from typing import Any, List, Optional, Tuple

from nav2_costmap_2d_py.core.costmap_subscriber import CostmapSubscriber
from nav2_costmap_2d_py.core.cost_values import LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.exceptions import (
    CollisionCheckerException,
    IllegalPoseException,
)
from nav2_costmap_2d_py.core.footprint import make_footprint_from_string, transform_footprint
from nav2_costmap_2d_py.core.footprint_collision_checker import FootprintCollisionChecker
from nav2_costmap_2d_py.core.footprint_subscriber import FootprintSubscriber

Footprint = List[Tuple[float, float]]


def _yaw_from_quaternion(q: Any) -> float:
    """Return the yaw angle of a quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CostmapTopicCollisionChecker:
    """Finds whether robot poses are in collision with a topic-based costmap."""

    def __init__(
        self,
        costmap_sub: CostmapSubscriber,
        footprint_sub: Optional[FootprintSubscriber] = None,
        footprint_string: Optional[str] = None,
        name: str = 'collision_checker',
    ) -> None:
        """
        Construct the collision checker.

        Either a footprint subscriber or a footprint string must be supplied.

        Parameters
        ----------
        costmap_sub : CostmapSubscriber
            The costmap subscriber to fetch the costmap from.
        footprint_sub : FootprintSubscriber, optional
            The footprint subscriber to fetch the footprint from.
        footprint_string : str, optional
            A footprint defined relative to the robot frame, as a string.
        name : str
            The name used for logging.

        """
        self._name = name
        self._costmap_sub = costmap_sub
        self._footprint_sub = footprint_sub
        self._collision_checker = FootprintCollisionChecker(None)
        self._footprint: Footprint = []
        self._footprint_string = footprint_string or ''
        if footprint_string is not None:
            self._footprint = make_footprint_from_string(footprint_string)
            if not self._footprint:
                raise CollisionCheckerException(
                    'Failed to create footprint from string')

    def is_collision_free(
        self, pose: Any, fetch_costmap_and_footprint: bool = True
    ) -> bool:
        """
        Return whether a pose is collision free.

        Parameters
        ----------
        pose : geometry_msgs.msg.Pose
            The pose to check collision at.
        fetch_costmap_and_footprint : bool
            Whether to fetch the latest costmap and footprint.

        Returns
        -------
        bool
            True if the pose is collision free.

        """
        try:
            if self.score_pose(pose, fetch_costmap_and_footprint) >= LETHAL_OBSTACLE:
                return False
            return True
        except CollisionCheckerException as e:
            self._costmap_sub._logger.error(str(e))
            return False

    def score_pose(
        self, pose: Any, fetch_costmap_and_footprint: bool = True
    ) -> float:
        """
        Return the obstacle footprint score for a particular pose.

        Parameters
        ----------
        pose : geometry_msgs.msg.Pose
            The pose to get the score at.
        fetch_costmap_and_footprint : bool
            Whether to fetch the latest costmap and footprint.

        Returns
        -------
        float
            The footprint cost at the pose.

        """
        if fetch_costmap_and_footprint:
            try:
                self._collision_checker.set_costmap(self._costmap_sub.get_costmap())
            except RuntimeError as e:
                raise CollisionCheckerException(str(e))

        ok, _, _ = self._collision_checker.world_to_map(
            pose.position.x, pose.position.y)
        if not ok:
            raise IllegalPoseException(self._name, 'Pose Goes Off Grid.')

        return self._collision_checker.footprint_cost(
            self.get_footprint(pose, fetch_costmap_and_footprint))

    def get_footprint(
        self, pose: Any, fetch_latest_footprint: bool = True
    ) -> Footprint:
        """
        Return the footprint at a set pose.

        Parameters
        ----------
        pose : geometry_msgs.msg.Pose
            The pose to get the footprint at.
        fetch_latest_footprint : bool
            Whether to fetch the latest footprint.

        Returns
        -------
        list of tuple of float
            The oriented footprint in world coordinates.

        """
        if fetch_latest_footprint and self._footprint_sub is not None:
            ok, footprint, _ = self._footprint_sub.get_footprint_in_robot_frame()
            if not ok:
                raise CollisionCheckerException('Current footprint not available.')
            self._footprint = footprint

        x = pose.position.x
        y = pose.position.y
        theta = _yaw_from_quaternion(pose.orientation)
        return transform_footprint(x, y, theta, self._footprint)
