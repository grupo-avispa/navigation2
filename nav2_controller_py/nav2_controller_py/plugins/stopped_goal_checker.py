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

"""StoppedGoalChecker – Python port of nav2_controller::StoppedGoalChecker."""

import math
import sys

from geometry_msgs.msg import Pose, Twist
from nav2_controller_py.plugins.simple_goal_checker import SimpleGoalChecker
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult


class StoppedGoalChecker(SimpleGoalChecker):
    """
    Check the linear and angular velocity after the robot has stopped at the goal.

    Parameters (under ``<name>.``), in addition to those of SimpleGoalChecker:
      rot_stopped_velocity    (float, default 0.25 rad/s)
      trans_stopped_velocity  (float, default 0.25 m/s)
    """

    def __init__(self):
        super().__init__()
        self._rot_stopped_velocity = 0.25
        self._trans_stopped_velocity = 0.25

    def initialize(self, parent, plugin_name: str, costmap_ros) -> None:
        super().initialize(parent, plugin_name, costmap_ros)
        self._rot_stopped_velocity = self._declare_or_get_parameter(
            f'{plugin_name}.rot_stopped_velocity', 0.25)
        self._trans_stopped_velocity = self._declare_or_get_parameter(
            f'{plugin_name}.trans_stopped_velocity', 0.25)
        parent.add_on_set_parameters_callback(self._on_set_parameters_callback_stopped)

    def is_goal_reached(
        self,
        query_pose: Pose,
        goal_pose: Pose,
        velocity: Twist,
        transformed_global_plan: Path,
    ) -> bool:
        with self._mutex:
            ret = super().is_goal_reached(
                query_pose, goal_pose, velocity, transformed_global_plan)
            if not ret:
                return ret
            return (abs(velocity.angular.z) <= self._rot_stopped_velocity
                    and math.hypot(velocity.linear.x, velocity.linear.y)
                    <= self._trans_stopped_velocity)

    def get_tolerances(self, pose_tolerance: Pose, vel_tolerance: Twist):
        with self._mutex:
            invalid_field = -sys.float_info.max
            rtn, pose_tolerance, vel_tolerance = super().get_tolerances(
                pose_tolerance, vel_tolerance)

            # override the velocities
            vel_tolerance.linear.x = self._trans_stopped_velocity
            vel_tolerance.linear.y = self._trans_stopped_velocity
            vel_tolerance.linear.z = invalid_field
            vel_tolerance.angular.x = invalid_field
            vel_tolerance.angular.y = invalid_field
            vel_tolerance.angular.z = self._rot_stopped_velocity
            return rtn, pose_tolerance, vel_tolerance

    def _on_set_parameters_callback_stopped(self, parameters) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        prefix = self._plugin_name + '.'
        for parameter in parameters:
            if not parameter.name.startswith(prefix):
                continue
            if isinstance(parameter.value, float) and parameter.value < 0.0:
                result.successful = False
        if not result.successful:
            return result
        with self._mutex:
            for parameter in parameters:
                if parameter.name == prefix + 'rot_stopped_velocity':
                    self._rot_stopped_velocity = parameter.value
                elif parameter.name == prefix + 'trans_stopped_velocity':
                    self._trans_stopped_velocity = parameter.value
        return result
