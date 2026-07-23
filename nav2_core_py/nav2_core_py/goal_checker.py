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
Abstract base class for Nav2 goal checker plugins.

It mirrors the nav2_core::GoalChecker from the C++ implementation.
"""

from typing import Tuple

from geometry_msgs.msg import Pose, Twist
from nav2_costmap_2d_py import Costmap2DROS
from nav_msgs.msg import Path
from rclpy.lifecycle import LifecycleNode


class GoalChecker:
    """
    Function-object for checking whether a goal has been reached.

    This class defines the plugin interface for determining whether the
    robot has reached the goal state. This primarily consists of checking
    the relative positions of two poses (which are presumed to be in the
    same frame). It can also check the velocity, as some applications require
    that the robot be stopped to be considered as having reached the goal.
    """

    def initialize(
        self,
        parent: LifecycleNode,
        plugin_name: str,
        costmap_ros: Costmap2DROS,
    ) -> None:
        """
        Initialize any parameters from the node.

        Parameters
        ----------
        parent : LifecycleNode
            The parent lifecycle node (the controller server).
        plugin_name : str
            Plugin instance name.
        costmap_ros : Costmap2DROS
            nav2_costmap_2d_py.Costmap2DROS instance.

        """
        raise NotImplementedError

    def reset(self) -> None:
        """Reset internal state (called at the start of each goal)."""
        raise NotImplementedError

    def is_goal_reached(
        self,
        query_pose: Pose,
        goal_pose: Pose,
        velocity: Twist,
        transformed_global_plan: Path,
    ) -> bool:
        """
        Check whether the goal should be considered reached.

        Parameters
        ----------
        query_pose : Pose
            The pose to check.
        goal_pose : Pose
            The pose to check against.
        velocity : Twist
            The robot's current velocity.
        transformed_global_plan : Path
            The global plan after being processed by the path handler.

        Returns
        -------
        bool
            True if the goal is reached.

        """
        raise NotImplementedError

    def get_tolerances(
        self,
        pose_tolerance: Pose,
        vel_tolerance: Twist,
    ) -> Tuple[bool, Pose, Twist]:
        """
        Get the maximum possible tolerances used for goal checking.

        Any field without a valid entry is replaced with the lowest finite
        double to indicate that it is not measured. For tolerance across
        multiple entries (e.g. XY tolerances), both fields will contain this
        value since it is the maximum tolerance that each independent field
        could be assuming the other has no error.

        Parameters
        ----------
        pose_tolerance : Pose
            The tolerance used for checking in Pose fields (filled in place).
        vel_tolerance : Twist
            The tolerance used for checking velocity fields (filled in place).

        Returns
        -------
        Tuple[bool, Pose, Twist]
            (True if tolerances are valid to use, pose_tolerance, vel_tolerance).

        """
        raise NotImplementedError
