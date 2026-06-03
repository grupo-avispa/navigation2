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
Abstract base class for Nav2 path handler plugins.

It mirrors the nav2_core::PathHandler from the C++ implementation.

In the C++ interface the plan segment is delimited by a pair of iterators
(``PathSegment``). In this Python port the segment is delimited by a pair of
integer indices ``(closest_point, pruned_plan_end)`` into the stored plan.
"""

from typing import Tuple

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.lifecycle import LifecycleNode


class PathHandler:
    """
    Function-object for handling the path from the Planner Server.

    This class defines the plugin interface used by the Controller Server to
    manage the path received from the Planner Server. Its primary
    responsibilities are pruning path segments the robot has already traversed
    and transforming the remaining, relevant portion of the path into the
    costmap's global or base frame.
    """

    def initialize(
        self,
        parent: LifecycleNode,
        logger,
        plugin_name: str,
        costmap_ros,
        tf_buffer,
    ) -> None:
        """
        Initialize parameters.

        Parameters
        ----------
        parent : LifecycleNode
            The parent lifecycle node (the controller server).
        logger :
            Node logging interface.
        plugin_name : str
            Plugin instance name.
        costmap_ros :
            nav2_costmap_2d_py.Costmap2DROS instance.
        tf_buffer :
            tf2_ros.Buffer for coordinate transforms.

        """
        raise NotImplementedError

    def set_plan(self, path: Path) -> None:
        """
        Set the new reference plan.

        Parameters
        ----------
        path : Path
            Path to use.

        """
        raise NotImplementedError

    def find_plan_segment(self, pose: PoseStamped) -> Tuple[int, int]:
        """
        Determine the portion of the global plan to be used for local control.

        Locates the start and end indices of the global plan segment that is
        relevant for controller computation based on the robot's current pose
        and local costmap.

        Parameters
        ----------
        pose : PoseStamped
            Robot pose in the odom frame.

        Returns
        -------
        Tuple[int, int]
            A pair ``(closest_point, pruned_plan_end)`` of indices defining the
            start and end (exclusive) of the selected plan segment.

        """
        raise NotImplementedError

    def transform_local_plan(
        self,
        closest_point: int,
        pruned_plan_end: int,
    ) -> Path:
        """
        Transform a predefined segment of the global plan into the costmap global frame.

        Parameters
        ----------
        closest_point : int
            Index of the starting pose of the path segment.
        pruned_plan_end : int
            Index of the ending pose of the path segment (exclusive).

        Returns
        -------
        Path
            The transformed local plan segment in the costmap global frame.

        """
        raise NotImplementedError

    def get_transformed_goal(self, stamp: Time) -> PoseStamped:
        """
        Get the global goal pose transformed to the costmap global frame.

        Parameters
        ----------
        stamp : Time
            Time to get the goal pose at.

        Returns
        -------
        PoseStamped
            Transformed goal pose.

        """
        raise NotImplementedError
