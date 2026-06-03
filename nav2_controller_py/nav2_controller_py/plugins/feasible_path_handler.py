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

"""FeasiblePathHandler – Python port of nav2_controller::FeasiblePathHandler."""

import copy
import math
import sys
import threading

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from nav2_controller_py.geometry_utils import (euclidean_distance, first_after_integrated_distance,
                                               get_yaw, min_by, shortest_angular_distance)
from nav2_controller_py.path_utils import (remove_poses_after_first_constraint,
                                           transform_pose_in_target_frame)
from nav2_core_py.controller_exceptions import ControllerTFError, InvalidPath
from nav2_core_py.path_handler import PathHandler
from nav_msgs.msg import Path
from rcl_interfaces.msg import SetParametersResult


class FeasiblePathHandler(PathHandler):
    """
    Prune the global plan around the robot and transform the remaining portion.

    Mirrors nav2_controller::FeasiblePathHandler.

    Parameters (under ``<plugin_name>.``):
      reject_unit_path            (bool,  default False)
      max_robot_pose_search_dist  (float, default half the costmap max extent)
      prune_distance              (float, default 2.0 m)
      enforce_path_inversion      (bool,  default False)
      enforce_path_rotation       (bool,  default False)
      inversion_xy_tolerance      (float, default 0.2 m)
      inversion_yaw_tolerance     (float, default 0.4 rad)
      minimum_rotation_angle      (float, default 0.785 rad)
    """

    def __init__(self):
        self._node = None
        self._logger = None
        self._plugin_name = ''
        self._mutex = threading.RLock()
        self._costmap_ros = None
        self._tf = None
        self._transform_tolerance = 0.1

        self._reject_unit_path = False
        self._max_robot_pose_search_dist = float('inf')
        self._prune_distance = 2.0
        self._enforce_path_inversion = False
        self._enforce_path_rotation = False
        self._inversion_xy_tolerance = 0.2
        self._inversion_yaw_tolerance = 0.4
        self._minimum_rotation_angle = 0.785
        self._constraint_locale = 0

        self._global_plan = Path()
        self._global_plan_up_to_constraint = Path()
        self._global_pose = PoseStamped()

    def initialize(self, parent, logger, plugin_name: str, costmap_ros, tf_buffer) -> None:
        self._logger = logger
        self._plugin_name = plugin_name
        self._node = parent
        self._costmap_ros = costmap_ros
        self._tf = tf_buffer
        self._transform_tolerance = costmap_ros.get_transform_tolerance()

        self._reject_unit_path = self._declare_or_get_parameter(
            f'{plugin_name}.reject_unit_path', False)
        self._max_robot_pose_search_dist = self._declare_or_get_parameter(
            f'{plugin_name}.max_robot_pose_search_dist', self._get_costmap_max_extent())
        self._prune_distance = self._declare_or_get_parameter(
            f'{plugin_name}.prune_distance', 2.0)
        self._enforce_path_inversion = self._declare_or_get_parameter(
            f'{plugin_name}.enforce_path_inversion', False)
        self._enforce_path_rotation = self._declare_or_get_parameter(
            f'{plugin_name}.enforce_path_rotation', False)
        self._inversion_xy_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.inversion_xy_tolerance', 0.2)
        self._inversion_yaw_tolerance = self._declare_or_get_parameter(
            f'{plugin_name}.inversion_yaw_tolerance', 0.4)
        self._minimum_rotation_angle = self._declare_or_get_parameter(
            f'{plugin_name}.minimum_rotation_angle', 0.785)

        if self._max_robot_pose_search_dist < 0.0:
            self._logger.warn(
                'Max robot search distance is negative, setting to max to search every point '
                'on path for the closest value.')
            self._max_robot_pose_search_dist = sys.float_info.max

        self._constraint_locale = 0
        if not self._enforce_path_rotation:
            self._minimum_rotation_angle = 0.0

        parent.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def set_plan(self, path: Path) -> None:
        with self._mutex:
            self._global_plan = path
            self._global_plan_up_to_constraint = copy.deepcopy(path)
            if self._enforce_path_inversion or self._enforce_path_rotation:
                self._constraint_locale = remove_poses_after_first_constraint(
                    self._global_plan_up_to_constraint,
                    self._enforce_path_inversion, self._minimum_rotation_angle)

    def find_plan_segment(self, pose: PoseStamped):
        with self._mutex:
            self._global_pose = self._transform_to_global_plan_frame(pose)
            poses = self._global_plan_up_to_constraint.poses

            # Limit the search for the closest pose up to max_robot_pose_search_dist
            closest_pose_upper_bound = first_after_integrated_distance(
                poses, 0, len(poses), self._max_robot_pose_search_dist)

            # Find the closest pose on the path to the robot, bounded by the upper bound
            closest_point = min_by(
                poses[0:closest_pose_upper_bound],
                lambda ps: euclidean_distance(self._global_pose.pose, ps.pose))

            # Make sure we always have at least 2 points on the transformed plan and that we
            # don't prune the global plan below 2 points
            if (closest_pose_upper_bound != 0 and len(poses) > 1
                    and closest_point == closest_pose_upper_bound - 1):
                closest_point = closest_pose_upper_bound - 2

            pruned_plan_end = first_after_integrated_distance(
                poses, closest_point, len(poses), self._prune_distance)

            return closest_point, pruned_plan_end

    def transform_local_plan(self, closest_point: int, pruned_plan_end: int) -> Path:
        with self._mutex:
            transformed_plan = Path()
            transformed_plan.header.frame_id = self._costmap_ros.get_global_frame_id()
            transformed_plan.header.stamp = self._global_pose.header.stamp

            costmap = self._costmap_ros.get_costmap()
            poses = self._global_plan_up_to_constraint.poses
            # Find the furthest relevant pose on the path within costmap bounds,
            # transforming it to the costmap frame in the same loop
            for i in range(closest_point, pruned_plan_end):
                global_plan_pose = poses[i]
                global_plan_pose.header.stamp = self._global_pose.header.stamp
                global_plan_pose.header.frame_id = self._global_plan.header.frame_id
                costmap_plan_pose = transform_pose_in_target_frame(
                    global_plan_pose, self._tf, self._costmap_ros.get_global_frame_id(),
                    self._transform_tolerance)
                if costmap_plan_pose is None:
                    break

                # Check if pose is inside the costmap
                in_map, _, _ = costmap.world_to_map(
                    costmap_plan_pose.pose.position.x, costmap_plan_pose.pose.position.y)
                if not in_map:
                    break

                transformed_plan.poses.append(costmap_plan_pose)

            # Remove the portion of the global plan that we've already passed (path pruning)
            self._prune_plan(self._global_plan_up_to_constraint, closest_point)

            if ((self._enforce_path_inversion or self._enforce_path_rotation)
                    and self._constraint_locale != 0):
                if self._is_within_inversion_tolerances(self._global_pose):
                    self._prune_plan(self._global_plan, self._constraint_locale)
                    self._global_plan_up_to_constraint = copy.deepcopy(self._global_plan)
                    self._constraint_locale = remove_poses_after_first_constraint(
                        self._global_plan_up_to_constraint,
                        self._enforce_path_inversion, self._minimum_rotation_angle)

            if not transformed_plan.poses:
                raise InvalidPath('Resulting plan has 0 poses in it.')

            return transformed_plan

    def get_transformed_goal(self, stamp: Time) -> PoseStamped:
        with self._mutex:
            if not self._global_plan.poses:
                raise ControllerTFError('Received plan with zero length')
            goal = copy.deepcopy(self._global_plan.poses[-1])
            goal.header.frame_id = self._global_plan.header.frame_id
            goal.header.stamp = stamp
            if not goal.header.frame_id:
                raise ControllerTFError('Goal pose has an empty frame_id')
            transformed_goal = transform_pose_in_target_frame(
                goal, self._costmap_ros.get_tf_buffer(),
                self._costmap_ros.get_global_frame_id(), self._transform_tolerance)
            if transformed_goal is None:
                raise ControllerTFError('Unable to transform goal pose into costmap frame')
            return transformed_goal

    # ------------------------------------------------------------------ helpers

    def _get_costmap_max_extent(self) -> float:
        costmap = self._costmap_ros.get_costmap()
        max_costmap_dim_meters = max(costmap.size_x_meters(), costmap.size_y_meters())
        return max_costmap_dim_meters / 2.0

    @staticmethod
    def _prune_plan(plan: Path, end: int) -> None:
        del plan.poses[0:end]

    def _is_within_inversion_tolerances(self, robot_pose: PoseStamped) -> bool:
        # Keep full path if we are within tolerance of the inversion pose
        last_pose = self._global_plan_up_to_constraint.poses[-1]
        distance = math.hypot(
            robot_pose.pose.position.x - last_pose.pose.position.x,
            robot_pose.pose.position.y - last_pose.pose.position.y)
        angle_distance = shortest_angular_distance(
            get_yaw(robot_pose.pose.orientation), get_yaw(last_pose.pose.orientation))
        return (distance <= self._inversion_xy_tolerance
                and abs(angle_distance) <= self._inversion_yaw_tolerance)

    def _transform_to_global_plan_frame(self, pose: PoseStamped) -> PoseStamped:
        if not self._global_plan_up_to_constraint.poses:
            raise InvalidPath('Received plan with zero length')
        if self._reject_unit_path and len(self._global_plan_up_to_constraint.poses) == 1:
            raise InvalidPath('Received plan with length of one')
        robot_pose = transform_pose_in_target_frame(
            pose, self._tf, self._global_plan_up_to_constraint.header.frame_id,
            self._transform_tolerance)
        if robot_pose is None:
            raise ControllerTFError("Unable to transform robot pose into global plan's frame")
        return robot_pose

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
                self._logger.warn(
                    f"The value of parameter '{parameter.name}' is incorrectly set to "
                    f'{parameter.value}, it should be >=0. Ignoring parameter update.')
                result.successful = False
        if not result.successful:
            return result
        with self._mutex:
            for parameter in parameters:
                name = parameter.name
                if name == prefix + 'max_robot_pose_search_dist':
                    self._max_robot_pose_search_dist = parameter.value
                elif name == prefix + 'inversion_xy_tolerance':
                    self._inversion_xy_tolerance = parameter.value
                elif name == prefix + 'inversion_yaw_tolerance':
                    self._inversion_yaw_tolerance = parameter.value
                elif name == prefix + 'prune_distance':
                    self._prune_distance = parameter.value
                elif name == prefix + 'minimum_rotation_angle':
                    self._minimum_rotation_angle = parameter.value
                elif name == prefix + 'enforce_path_inversion':
                    self._enforce_path_inversion = parameter.value
                elif name == prefix + 'enforce_path_rotation':
                    self._enforce_path_rotation = parameter.value
                    if not self._enforce_path_rotation:
                        self._minimum_rotation_angle = 0.0
        return result
