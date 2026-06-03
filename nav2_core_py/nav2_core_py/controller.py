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
Abstract base class for Nav2 controller plugins.

All Python controller plugins must inherit from Controller and
implement every abstract method.

It mirrors the nav2_core::Controller from the C++ implementation.
"""

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.lifecycle import LifecycleNode


class Controller:
    """
    Controller interface that acts as a base class for all controller plugins.

    Lifecycle
    ---------
    configure()  → called once during ControllerServer on_configure
    activate()   → called during ControllerServer on_activate
    deactivate() → called during ControllerServer on_deactivate
    cleanup()    → called during ControllerServer on_cleanup
    compute_velocity_commands() → called on every control cycle
    """

    def configure(
        self,
        parent: LifecycleNode,
        name: str,
        tf_buffer,
        costmap_ros,
    ) -> None:
        """
        Configure the controller plugin.

        Parameters
        ----------
        parent : LifecycleNode
            The parent lifecycle node (the controller server).
        name : str
            Plugin instance name as declared in controller_plugins.
        tf_buffer :
            tf2_ros.Buffer for coordinate transforms.
        costmap_ros :
            nav2_costmap_2d_py.Costmap2DROS instance.

        """
        raise NotImplementedError

    def cleanup(self) -> None:
        """Clean up plugin resources. Called during on_cleanup."""
        raise NotImplementedError

    def activate(self) -> None:
        """Activate the plugin and any threads involved. Called during on_activate."""
        raise NotImplementedError

    def deactivate(self) -> None:
        """Deactivate the plugin and any threads involved. Called during on_deactivate."""
        raise NotImplementedError

    def new_path_received(self, raw_global_path: Path) -> None:
        """
        Notify the controller that a new plan is received from the Planner Server.

        This callback should only perform minimal work, such as extracting
        global information that may be of interest (e.g. resetting internal
        states when a new path is received). The controller will be provided
        with the transformed and pruned plan in the local frame during
        compute_velocity_commands().

        Parameters
        ----------
        raw_global_path : Path
            The new global plan.

        """
        raise NotImplementedError

    def compute_velocity_commands(
        self,
        pose: PoseStamped,
        velocity: Twist,
        goal_checker,
        transformed_global_plan: Path,
        global_goal: PoseStamped,
    ) -> TwistStamped:
        """
        Calculate the best command given the current pose and velocity.

        It is presumed that the global plan is already set.

        Parameters
        ----------
        pose : PoseStamped
            Current robot pose.
        velocity : Twist
            Current robot velocity.
        goal_checker :
            The current goal checker the task is utilizing.
        transformed_global_plan : Path
            The global plan after being processed by the path handler.
        global_goal : PoseStamped
            The last pose of the global plan.

        Returns
        -------
        TwistStamped
            The best command for the robot to drive.

        Raises
        ------
        nav2_core_py.controller_exceptions.NoValidControl
            If no valid control can be computed.
        nav2_core_py.controller_exceptions.ControllerTFError
            On transform failures.

        """
        raise NotImplementedError

    def cancel(self) -> bool:
        """
        Cancel the current control action.

        Returns
        -------
        bool
            True if the cancellation was successful. If False is returned,
            compute_velocity_commands will be called until cancel returns True.

        """
        return True

    def set_speed_limit(self, speed_limit: float, percentage: bool) -> None:
        """
        Limit the maximum linear speed of the robot.

        Parameters
        ----------
        speed_limit : float
            Speed limit expressed in absolute value (in m/s) or in percentage
            from maximum robot speed.
        percentage : bool
            Setting speed limit in percentage if True, or in absolute values
            otherwise.

        """
        raise NotImplementedError

    def reset(self) -> None:
        """Reset the state of the controller if necessary after task is exited."""
        pass
