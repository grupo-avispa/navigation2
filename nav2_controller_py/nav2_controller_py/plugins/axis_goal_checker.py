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

"""AxisGoalChecker – Python port of nav2_controller::AxisGoalChecker."""

import math
import sys
import threading

from geometry_msgs.msg import Pose, Twist
from nav2_controller_py.geometry_utils import (calculate_path_length, orientation_around_z_axis,
                                               shortest_angular_distance)
from nav2_core_py.goal_checker import GoalChecker
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult


class AxisGoalChecker(GoalChecker):
    """
    Goal checker that checks progress along the axis defined by the last 2 poses of the path.

    Parameters (under ``<name>.``):
      along_path_tolerance   (float, default 0.25 m)
      cross_track_tolerance  (float, default 0.25 m)
      path_length_tolerance  (float, default 1.0  m)
      is_overshoot_valid     (bool,  default False)
    """

    def __init__(self):
        self._node = None
        self._plugin_name = ''
        self._mutex = threading.RLock()
        self._along_path_tolerance = 0.25
        self._cross_track_tolerance = 0.25
        self._path_length_tolerance = 1.0
        self._is_overshoot_valid = False

    def initialize(self, parent, plugin_name: str, costmap_ros) -> None:
        self._plugin_name = plugin_name
        self._node = parent
        self._logger = parent.get_logger()

        self._along_path_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.along_path_tolerance', 0.25)
        self._cross_track_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.cross_track_tolerance', 0.25)
        self._path_length_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.path_length_tolerance', 1.0)
        self._is_overshoot_valid = self._declare_or_get_parameter(
            f'{plugin_name}.is_overshoot_valid', False)

        parent.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def reset(self) -> None:
        pass

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

            poses = transformed_global_plan.poses
            # Check if we have at least 2 poses to determine path direction
            if len(poses) >= 2:
                # Find a pose before goal that is sufficiently far from goal
                before_goal_pose = None
                dx = 0.0
                dy = 0.0
                for i in range(len(poses) - 2, -1, -1):
                    candidate_pose = poses[i].pose
                    dx = goal_pose.position.x - candidate_pose.position.x
                    dy = goal_pose.position.y - candidate_pose.position.y
                    if math.hypot(dx, dy) >= 1e-6:
                        before_goal_pose = candidate_pose
                        break

                # If all poses are too close to goal, fall back to simple distance check
                if before_goal_pose is None:
                    self._logger.debug(
                        'All poses in path are too close to goal, falling back to simple '
                        'distance check')
                    distance_to_goal = math.hypot(
                        goal_pose.position.x - query_pose.position.x,
                        goal_pose.position.y - query_pose.position.y)
                    tolerance = math.hypot(
                        self._along_path_tolerance, self._cross_track_tolerance)
                    return distance_to_goal < tolerance

                # end of path direction
                end_of_path_yaw = math.atan2(dy, dx)

                # Check if robot is already at goal (would cause atan2(0,0))
                robot_to_goal_dx = goal_pose.position.x - query_pose.position.x
                robot_to_goal_dy = goal_pose.position.y - query_pose.position.y
                distance_to_goal = math.hypot(robot_to_goal_dx, robot_to_goal_dy)

                if distance_to_goal < 1e-6:
                    return True  # Robot is at goal

                robot_to_goal_yaw = math.atan2(robot_to_goal_dy, robot_to_goal_dx)
                projection_angle = shortest_angular_distance(
                    robot_to_goal_yaw, end_of_path_yaw)
                along_path_distance = distance_to_goal * math.cos(projection_angle)
                cross_track_distance = distance_to_goal * math.sin(projection_angle)

                if self._is_overshoot_valid:
                    return (along_path_distance < self._along_path_tolerance
                            and abs(cross_track_distance) < self._cross_track_tolerance)
                else:
                    return (abs(along_path_distance) < self._along_path_tolerance
                            and abs(cross_track_distance) < self._cross_track_tolerance)
            else:
                # Fallback: path has only 1 point, use simple distance check
                self._logger.debug(
                    'Path has fewer than 2 poses, falling back to simple distance check')
                distance_to_goal = math.hypot(
                    goal_pose.position.x - query_pose.position.x,
                    goal_pose.position.y - query_pose.position.y)
                tolerance = math.hypot(self._along_path_tolerance, self._cross_track_tolerance)
                return distance_to_goal < tolerance

    def get_tolerances(self, pose_tolerance: Pose, vel_tolerance: Twist):
        with self._mutex:
            invalid_field = -sys.float_info.max
            min_tol = min(self._along_path_tolerance, self._cross_track_tolerance)
            pose_tolerance.position.x = min_tol
            pose_tolerance.position.y = min_tol
            pose_tolerance.position.z = invalid_field
            pose_tolerance.orientation = orientation_around_z_axis(math.pi / 2.0)

            vel_tolerance.linear.x = invalid_field
            vel_tolerance.linear.y = invalid_field
            vel_tolerance.linear.z = invalid_field
            vel_tolerance.angular.x = invalid_field
            vel_tolerance.angular.y = invalid_field
            vel_tolerance.angular.z = invalid_field
            return True, pose_tolerance, vel_tolerance

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
                if name == prefix + 'along_path_tolerance':
                    self._along_path_tolerance = parameter.value
                elif name == prefix + 'cross_track_tolerance':
                    self._cross_track_tolerance = parameter.value
                elif name == prefix + 'path_length_tolerance':
                    self._path_length_tolerance = parameter.value
                elif name == prefix + 'is_overshoot_valid':
                    self._is_overshoot_valid = parameter.value
        return result
