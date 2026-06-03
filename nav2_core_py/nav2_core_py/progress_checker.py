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
Abstract base class for Nav2 progress checker plugins.

It mirrors the nav2_core::ProgressChecker from the C++ implementation.
"""

from geometry_msgs.msg import PoseStamped
from rclpy.lifecycle import LifecycleNode


class ProgressChecker:
    """
    Plugin interface used to check the position of the robot.

    This class defines the plugin interface used to check the position of the
    robot to make sure that it is actually progressing towards a goal.
    """

    def initialize(
        self,
        parent: LifecycleNode,
        plugin_name: str,
    ) -> None:
        """
        Initialize parameters for the progress checker.

        Parameters
        ----------
        parent : LifecycleNode
            The parent lifecycle node (the controller server).
        plugin_name : str
            Plugin instance name.

        """
        raise NotImplementedError

    def check(self, current_pose: PoseStamped) -> bool:
        """
        Check if the robot has moved compared to the previous pose.

        Parameters
        ----------
        current_pose : PoseStamped
            Current pose of the robot.

        Returns
        -------
        bool
            True if progress is made.

        """
        raise NotImplementedError

    def reset(self) -> None:
        """Reset class state upon calling."""
        raise NotImplementedError
