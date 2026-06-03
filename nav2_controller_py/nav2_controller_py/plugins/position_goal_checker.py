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

"""PositionGoalChecker – Python port of nav2_controller::PositionGoalChecker."""

import sys
import threading

from geometry_msgs.msg import Pose, Twist
from nav2_controller_py.geometry_utils import calculate_path_length
from nav2_core_py.goal_checker import GoalChecker
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult


class PositionGoalChecker(GoalChecker):
    """
    Goal checker that only checks XY position and ignores orientation.

    Parameters (under ``<name>.``):
      xy_goal_tolerance      (float, default 0.25 m)
      path_length_tolerance  (float, default 1.0  m)
      stateful               (bool,  default True)
    """

    def __init__(self):
        self._node = None
        self._plugin_name = ''
        self._mutex = threading.RLock()
        self._xy_goal_tolerance = 0.25
        self._xy_goal_tolerance_sq = 0.0625
        self._path_length_tolerance = 1.0
        self._stateful = True
        self._position_reached = False

    def initialize(self, parent, plugin_name: str, costmap_ros) -> None:
        self._plugin_name = plugin_name
        self._node = parent
        self._logger = parent.get_logger()

        self._xy_goal_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.xy_goal_tolerance', 0.25)
        self._path_length_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.path_length_tolerance', 1.0)
        self._stateful = self._declare_or_get_parameter(f'{plugin_name}.stateful', True)

        self._xy_goal_tolerance_sq = self._xy_goal_tolerance * self._xy_goal_tolerance

        parent.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def reset(self) -> None:
        self._position_reached = False

    def is_goal_reached(
        self,
        query_pose: Pose,
        goal_pose: Pose,
        velocity: Twist,
        transformed_global_plan: Path,
    ) -> bool:
        with self._mutex:
            # If the local plan length is longer than the tolerance, we skip the check
            if calculate_path_length(transformed_global_plan) > self._path_length_tolerance:
                return False
            # If stateful and position was already reached, maintain state
            if self._stateful and self._position_reached:
                return True

            dx = query_pose.position.x - goal_pose.position.x
            dy = query_pose.position.y - goal_pose.position.y
            position_reached = (dx * dx + dy * dy <= self._xy_goal_tolerance_sq)

            if self._stateful and position_reached:
                self._position_reached = True

            return position_reached

    def get_tolerances(self, pose_tolerance: Pose, vel_tolerance: Twist):
        with self._mutex:
            invalid_field = -sys.float_info.max
            pose_tolerance.position.x = self._xy_goal_tolerance
            pose_tolerance.position.y = self._xy_goal_tolerance
            pose_tolerance.position.z = invalid_field

            # Return zero orientation tolerance as we don't check it
            pose_tolerance.orientation.x = 0.0
            pose_tolerance.orientation.y = 0.0
            pose_tolerance.orientation.z = 0.0
            pose_tolerance.orientation.w = 1.0

            vel_tolerance.linear.x = invalid_field
            vel_tolerance.linear.y = invalid_field
            vel_tolerance.linear.z = invalid_field
            vel_tolerance.angular.x = invalid_field
            vel_tolerance.angular.y = invalid_field
            vel_tolerance.angular.z = invalid_field
            return True, pose_tolerance, vel_tolerance

    def set_xy_goal_tolerance(self, tolerance: float) -> None:
        self._xy_goal_tolerance = tolerance
        self._xy_goal_tolerance_sq = tolerance * tolerance

    # ------------------------------------------------------------------ helpers

    def _declare_or_get_parameter(self, name: str, default_value):
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        return self._node.declare_parameter(name, default_value).value

    def _on_set_parameters_callback(self, parameters) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        prefix = self._plugin_name + '.'
        for parameter in parameters:
            if not parameter.name.startswith(prefix):
                continue
            if isinstance(parameter.value, float) and parameter.value < 0.0:
                self._logger.warning(
                    f"The value of parameter '{parameter.name}' is incorrectly set to "
                    f'{parameter.value}, it should be >=0. Ignoring parameter update.')
                result.successful = False
        if not result.successful:
            return result
        with self._mutex:
            for parameter in parameters:
                name = parameter.name
                if name == prefix + 'xy_goal_tolerance':
                    self._xy_goal_tolerance = parameter.value
                    self._xy_goal_tolerance_sq = self._xy_goal_tolerance * self._xy_goal_tolerance
                elif name == prefix + 'path_length_tolerance':
                    self._path_length_tolerance = parameter.value
                elif name == prefix + 'stateful':
                    self._stateful = parameter.value
        return result
