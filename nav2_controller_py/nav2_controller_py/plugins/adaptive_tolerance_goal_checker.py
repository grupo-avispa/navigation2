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

"""AdaptiveToleranceGoalChecker – Python port of nav2_controller::AdaptiveToleranceGoalChecker."""

import math
import sys
import threading

from geometry_msgs.msg import Pose, Twist
from nav2_controller_py.geometry_utils import (calculate_path_length, get_yaw, normalize_angle,
                                               orientation_around_z_axis,
                                               shortest_angular_distance)
from nav2_core_py.goal_checker import GoalChecker
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult


class AdaptiveToleranceGoalChecker(GoalChecker):
    """
    Goal checker with two tolerance tiers.

    Accepts a tight (fine) tolerance immediately, or a loose (coarse) tolerance
    when the robot stops making progress toward the goal.

    Parameters (under ``<name>.``):
      fine_xy_goal_tolerance     (float, default 0.10 m)
      coarse_xy_goal_tolerance   (float, default 0.25 m)
      yaw_goal_tolerance         (float, default 0.25 rad)
      path_length_tolerance      (float, default 1.0  m)
      stateful                   (bool,  default True)
      symmetric_yaw_tolerance    (bool,  default False)
      trans_stopped_velocity     (float, default 0.10 m/s)
      rot_stopped_velocity       (float, default 0.10 rad/s)
      required_stagnation_cycles (int,   default 15)
    """

    def __init__(self):
        self._node = None
        self._plugin_name = ''
        self._mutex = threading.RLock()
        self._fine_xy_goal_tolerance = 0.10
        self._fine_xy_goal_tolerance_sq = 0.01
        self._coarse_xy_goal_tolerance = 0.25
        self._coarse_xy_goal_tolerance_sq = 0.0625
        self._yaw_goal_tolerance = 0.25
        self._path_length_tolerance = 1.0
        self._stateful = True
        self._symmetric_yaw_tolerance = False
        self._trans_stopped_velocity = 0.10
        self._rot_stopped_velocity = 0.10
        self._required_stagnation_cycles = 15
        self._check_xy = True
        self._in_tolerance_zone = False
        self._stopped_stagnation_count = 0
        self._distance_stagnation_count = 0
        self._best_distance_sq = sys.float_info.max
        self._approach_dx = 0.0
        self._approach_dy = 0.0
        self._xy_acceptance_reason = ''

    def initialize(self, parent, plugin_name: str, costmap_ros) -> None:
        self._plugin_name = plugin_name
        self._node = parent
        self._logger = parent.get_logger()

        self._fine_xy_goal_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.fine_xy_goal_tolerance', 0.10)
        self._coarse_xy_goal_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.coarse_xy_goal_tolerance', 0.25)
        self._yaw_goal_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.yaw_goal_tolerance', 0.25)
        self._path_length_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.path_length_tolerance', 1.0)
        self._stateful = self._declare_or_get_parameter(f'{plugin_name}.stateful', True)
        self._symmetric_yaw_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.symmetric_yaw_tolerance', False)
        self._trans_stopped_velocity = self._declare_or_get_parameter(
            f'{plugin_name}.trans_stopped_velocity', 0.10)
        self._rot_stopped_velocity = self._declare_or_get_parameter(
            f'{plugin_name}.rot_stopped_velocity', 0.10)
        self._required_stagnation_cycles = self._declare_or_get_parameter(
            f'{plugin_name}.required_stagnation_cycles', 15)

        self._fine_xy_goal_tolerance_sq = \
            self._fine_xy_goal_tolerance * self._fine_xy_goal_tolerance
        self._coarse_xy_goal_tolerance_sq = \
            self._coarse_xy_goal_tolerance * self._coarse_xy_goal_tolerance

        if self._fine_xy_goal_tolerance >= self._coarse_xy_goal_tolerance:
            self._warn_fine_coarse()

        parent.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def reset(self) -> None:
        self._check_xy = True
        self._in_tolerance_zone = False
        self._stopped_stagnation_count = 0
        self._distance_stagnation_count = 0
        self._best_distance_sq = sys.float_info.max
        self._approach_dx = 0.0
        self._approach_dy = 0.0
        self._xy_acceptance_reason = ''

    def is_goal_reached(
        self,
        query_pose: Pose,
        goal_pose: Pose,
        velocity: Twist,
        transformed_global_plan: Path,
    ) -> bool:
        with self._mutex:
            # Skip check if local plan is still long (robot is far from goal region)
            if calculate_path_length(transformed_global_plan) > self._path_length_tolerance:
                return False

            if self._check_xy:
                dx = query_pose.position.x - goal_pose.position.x
                dy = query_pose.position.y - goal_pose.position.y
                dist_sq = dx * dx + dy * dy

                # Tier 1: Tight (desired) tolerance — immediate acceptance
                if dist_sq <= self._fine_xy_goal_tolerance_sq:
                    self._xy_acceptance_reason = 'fine tolerance'
                    if self._stateful:
                        self._check_xy = False
                    # Fall through to yaw check

                # Tier 2: Within the coarse tolerance zone — check velocity stagnation
                elif dist_sq <= self._coarse_xy_goal_tolerance_sq:
                    # Just entered the zone: initialize tracking
                    if not self._in_tolerance_zone:
                        self._in_tolerance_zone = True
                        self._stopped_stagnation_count = 0
                        self._distance_stagnation_count = 0
                        self._best_distance_sq = dist_sq
                        self._approach_dx = -dx
                        self._approach_dy = -dy
                        return False

                    # Check if best distance has been improved
                    if dist_sq < self._best_distance_sq:
                        self._best_distance_sq = dist_sq
                        self._distance_stagnation_count = 0
                    else:
                        self._distance_stagnation_count += 1

                    # Check if the robot is stopped or not making progress toward goal
                    if (math.hypot(velocity.linear.x, velocity.linear.y)
                            <= self._trans_stopped_velocity
                            and abs(velocity.angular.z) <= self._rot_stopped_velocity):
                        self._stopped_stagnation_count += 1
                    else:
                        self._stopped_stagnation_count = 0

                    # Finish line: robot crossed from approaching (dot<0) to passed (dot>=0)
                    crossed_finish_line = \
                        dx * self._approach_dx + dy * self._approach_dy >= 0.0

                    if (not crossed_finish_line
                            and self._stopped_stagnation_count < self._required_stagnation_cycles
                            and self._distance_stagnation_count
                            < self._required_stagnation_cycles):
                        return False

                    # Accepted at coarse: record which trigger fired
                    if crossed_finish_line:
                        self._xy_acceptance_reason = 'coarse tolerance / finish line'
                    elif self._stopped_stagnation_count >= self._required_stagnation_cycles:
                        self._xy_acceptance_reason = 'coarse tolerance / stopped stagnation'
                    else:
                        self._xy_acceptance_reason = 'coarse tolerance / distance stagnation'

                    if self._stateful:
                        self._check_xy = False
                else:
                    # Outside both tolerances: reset tracking state
                    self._in_tolerance_zone = False
                    self._stopped_stagnation_count = 0
                    self._distance_stagnation_count = 0
                    return False

            # XY is satisfied — check yaw
            query_yaw = get_yaw(query_pose.orientation)
            goal_yaw = get_yaw(goal_pose.orientation)
            if self._symmetric_yaw_tolerance:
                dyaw_forward = shortest_angular_distance(query_yaw, goal_yaw)
                dyaw_backward = shortest_angular_distance(
                    query_yaw, normalize_angle(goal_yaw + math.pi))
                yaw_reached = (abs(dyaw_forward) <= self._yaw_goal_tolerance
                               or abs(dyaw_backward) <= self._yaw_goal_tolerance)
            else:
                dyaw = shortest_angular_distance(query_yaw, goal_yaw)
                yaw_reached = abs(dyaw) <= self._yaw_goal_tolerance

            if yaw_reached:
                self._logger.info(
                    f'AdaptiveToleranceGoalChecker: goal reached via {self._xy_acceptance_reason} '
                    f'(fine: {self._fine_xy_goal_tolerance:.3f} m, '
                    f'coarse: {self._coarse_xy_goal_tolerance:.3f} m)')

            return yaw_reached

    def get_tolerances(self, pose_tolerance: Pose, vel_tolerance: Twist):
        with self._mutex:
            invalid_field = -sys.float_info.max
            # Report max tolerance as the worst-case bound
            pose_tolerance.position.x = self._coarse_xy_goal_tolerance
            pose_tolerance.position.y = self._coarse_xy_goal_tolerance
            pose_tolerance.position.z = invalid_field
            pose_tolerance.orientation = orientation_around_z_axis(self._yaw_goal_tolerance)

            vel_tolerance.linear.x = self._trans_stopped_velocity
            vel_tolerance.linear.y = self._trans_stopped_velocity
            vel_tolerance.linear.z = invalid_field
            vel_tolerance.angular.x = invalid_field
            vel_tolerance.angular.y = invalid_field
            vel_tolerance.angular.z = self._rot_stopped_velocity
            return True, pose_tolerance, vel_tolerance

    # ------------------------------------------------------------------ helpers

    def _warn_fine_coarse(self) -> None:
        self._logger.warning(
            f'Fine XY goal tolerance ({self._fine_xy_goal_tolerance:.3f}) is greater or equal to '
            f'coarse XY goal tolerance ({self._coarse_xy_goal_tolerance:.3f}). This may lead to '
            'unintended behavior (when fine >= coarse the checker will act as a simple goal '
            'checker). Consider setting fine_xy_goal_tolerance < coarse_xy_goal_tolerance.')

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
            if isinstance(parameter.value, bool):
                continue
            if isinstance(parameter.value, float) and parameter.value < 0.0:
                self._logger.warning(
                    f"The value of parameter '{parameter.name}' is incorrectly set to "
                    f'{parameter.value}, it should be >=0. Ignoring parameter update.')
                result.successful = False
            elif isinstance(parameter.value, int) and \
                    parameter.name == prefix + 'required_stagnation_cycles' \
                    and parameter.value < 1:
                self._logger.warning(
                    f"The value of parameter '{parameter.name}' is incorrectly set to "
                    f'{parameter.value}, it should be >= 1. Ignoring parameter update.')
                result.successful = False
        if not result.successful:
            return result
        with self._mutex:
            for parameter in parameters:
                name = parameter.name
                if name == prefix + 'fine_xy_goal_tolerance':
                    self._fine_xy_goal_tolerance = parameter.value
                    self._fine_xy_goal_tolerance_sq = parameter.value * parameter.value
                    if self._fine_xy_goal_tolerance >= self._coarse_xy_goal_tolerance:
                        self._warn_fine_coarse()
                elif name == prefix + 'coarse_xy_goal_tolerance':
                    self._coarse_xy_goal_tolerance = parameter.value
                    self._coarse_xy_goal_tolerance_sq = parameter.value * parameter.value
                    if self._fine_xy_goal_tolerance >= self._coarse_xy_goal_tolerance:
                        self._warn_fine_coarse()
                elif name == prefix + 'yaw_goal_tolerance':
                    self._yaw_goal_tolerance = parameter.value
                elif name == prefix + 'path_length_tolerance':
                    self._path_length_tolerance = parameter.value
                elif name == prefix + 'trans_stopped_velocity':
                    self._trans_stopped_velocity = parameter.value
                elif name == prefix + 'rot_stopped_velocity':
                    self._rot_stopped_velocity = parameter.value
                elif name == prefix + 'stateful':
                    self._stateful = parameter.value
                elif name == prefix + 'symmetric_yaw_tolerance':
                    self._symmetric_yaw_tolerance = parameter.value
                elif name == prefix + 'required_stagnation_cycles':
                    self._required_stagnation_cycles = int(parameter.value)
        return result
