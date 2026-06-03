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
Abstract base class for Nav2 global planner plugins.

All Python planner plugins must inherit from GlobalPlanner and
implement every abstract method.

It mirrors the nav2_core::GlobalPlanner from the C++ implementation.
"""

from typing import List

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.lifecycle import LifecycleNode


class GlobalPlanner:
    """
    Abstract base class that all Python planner plugins must implement.

    Lifecycle
    ---------
    configure()  → called once during PlannerServer on_configure
    activate()   → called during PlannerServer on_activate
    deactivate() → called during PlannerServer on_deactivate
    cleanup()    → called during PlannerServer on_cleanup
    create_plan() → called on every planning request
    """

    def configure(
        self,
        parent: LifecycleNode,
        name: str,
        tf_buffer,
        costmap_ros,
    ) -> None:
        """
        Configure the planner plugin.

        Called once by the PlannerServer during its on_configure transition.
        Use this method to declare and read parameters, initialise data
        structures, and store references to shared resources.

        Parameters
        ----------
        parent : LifecycleNode
            The parent lifecycle node (the planner server).
            Use it to declare / get parameters and access the logger.
        name : str
            Plugin instance name as declared in planner_plugins.
            Use it to namespace your parameters, e.g. f'{name}.tolerance'.
        tf_buffer :
            tf2_ros.Buffer for coordinate transforms, or None if unavailable.
        costmap_ros :
            nav2_costmap_2d.Costmap2DROS instance (or None / mock).

        """
        raise NotImplementedError

    def cleanup(self) -> None:
        """
        Clean up plugin resources.

        Called during PlannerServer on_cleanup. Release any resources
        (subscribers, publishers, threads, etc.) acquired in configure().
        """
        raise NotImplementedError

    def activate(self) -> None:
        """
        Activate the plugin.

        Called during PlannerServer on_activate. Re-enable any resources
        that were deactivated (e.g. activate publishers).
        """
        raise NotImplementedError

    def deactivate(self) -> None:
        """
        Deactivate the plugin.

        Called during PlannerServer on_deactivate. Pause activity without
        releasing resources (those are released in cleanup()).
        """
        raise NotImplementedError

    def create_plan(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        viapoints: List[PoseStamped],
        cancel_checker,
    ) -> Path:
        """
        Compute a path from start to goal, optionally through intermediate viapoints.

        Called on every planning request. Must return as quickly as possible
        and must honour the cancel_checker to support action cancellation.

        Parameters
        ----------
        start : PoseStamped
            Start pose in the global frame.
        goal : PoseStamped
            Goal pose in the global frame.
        viapoints : List[PoseStamped]
            Intermediate viapoints the path should pass through.
            May be an empty list if no viapoints are specified.
        cancel_checker : callable
            Zero-argument callable; returns True if the action has been
            cancelled. Check this periodically in long-running computations.

        Returns
        -------
        Path
            The computed path. Return an empty Path() if no path was found
            (the server will raise NoValidPathCouldBeFound automatically).

        Raises
        ------
        PlannerException
            Any subclass of PlannerException to signal specific failure modes.

        """
        raise NotImplementedError
