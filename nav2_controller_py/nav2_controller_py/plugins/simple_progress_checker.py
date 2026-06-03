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

"""SimpleProgressChecker – Python port of nav2_controller::SimpleProgressChecker."""

import math
import threading

from geometry_msgs.msg import Pose, PoseStamped
from nav2_core_py.progress_checker import ProgressChecker
from rcl_interfaces.msg import SetParametersResult


class SimpleProgressChecker(ProgressChecker):
    """
    Check that the robot position is moving more than a radius within a time allowance.

    Parameters (under ``<name>.``):
      required_movement_radius  (float, default 0.5 m)
      movement_time_allowance   (float, default 10.0 s)
    """

    def __init__(self):
        self._node = None
        self._plugin_name = ''
        self._mutex = threading.Lock()
        self._radius = 0.5
        self._time_allowance = 10.0
        self._baseline_pose = Pose()
        self._baseline_time = None
        self._baseline_pose_set = False

    def initialize(self, parent, plugin_name: str) -> None:
        self._plugin_name = plugin_name
        self._node = parent
        self._clock = parent.get_clock()
        self._logger = parent.get_logger()

        self._radius = self._declare_or_get_parameter(
            f'{plugin_name}.required_movement_radius', 0.5)
        self._time_allowance = self._declare_or_get_parameter(
            f'{plugin_name}.movement_time_allowance', 10.0)

        parent.add_on_set_parameters_callback(self._on_set_parameters_callback)

    def check(self, current_pose: PoseStamped) -> bool:
        with self._mutex:
            # Relies on short circuit evaluation to not call _is_robot_moved_enough
            # if baseline_pose is not set.
            if not self._baseline_pose_set or self._is_robot_moved_enough(current_pose.pose):
                self._reset_baseline_pose(current_pose.pose)
                return True
            elapsed = (self._clock.now() - self._baseline_time).nanoseconds / 1e9
            return not (elapsed > self._time_allowance)

    def reset(self) -> None:
        self._baseline_pose_set = False

    # ------------------------------------------------------------------ helpers

    def _reset_baseline_pose(self, pose: Pose) -> None:
        self._baseline_pose = pose
        self._baseline_time = self._clock.now()
        self._baseline_pose_set = True

    def _is_robot_moved_enough(self, pose: Pose) -> bool:
        return self._pose_distance(pose, self._baseline_pose) > self._radius

    @staticmethod
    def _pose_distance(pose1: Pose, pose2: Pose) -> float:
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.hypot(dx, dy)

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
                if parameter.name == prefix + 'required_movement_radius':
                    self._radius = parameter.value
                elif parameter.name == prefix + 'movement_time_allowance':
                    self._time_allowance = parameter.value
        return result
