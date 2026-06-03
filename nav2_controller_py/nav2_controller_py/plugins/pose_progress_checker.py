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

"""PoseProgressChecker – Python port of nav2_controller::PoseProgressChecker."""

from geometry_msgs.msg import Pose
from nav2_controller_py.geometry_utils import get_yaw, shortest_angular_distance
from nav2_controller_py.plugins.simple_progress_checker import SimpleProgressChecker
from rcl_interfaces.msg import SetParametersResult


class PoseProgressChecker(SimpleProgressChecker):
    """
    Check that the robot has moved enough in position or orientation within a time allowance.

    Parameters (under ``<name>.``):
      required_movement_radius  (float, default 0.5 m)
      movement_time_allowance   (float, default 10.0 s)
      required_movement_angle   (float, default 0.5 rad)
    """

    def __init__(self):
        super().__init__()
        self._required_movement_angle = 0.5

    def initialize(self, parent, plugin_name: str) -> None:
        super().initialize(parent, plugin_name)
        self._required_movement_angle = self._declare_or_get_parameter(
            f'{plugin_name}.required_movement_angle', 0.5)
        parent.add_on_set_parameters_callback(self._on_set_parameters_callback_pose)

    def _is_robot_moved_enough(self, pose: Pose) -> bool:
        return (self._pose_distance(pose, self._baseline_pose) > self._radius
                or self._pose_angle_distance(pose, self._baseline_pose)
                > self._required_movement_angle)

    @staticmethod
    def _pose_angle_distance(pose1: Pose, pose2: Pose) -> float:
        theta1 = get_yaw(pose1.orientation)
        theta2 = get_yaw(pose2.orientation)
        return abs(shortest_angular_distance(theta1, theta2))

    def _on_set_parameters_callback_pose(self, parameters) -> SetParametersResult:
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
                if parameter.name == prefix + 'required_movement_angle':
                    self._required_movement_angle = parameter.value
        return result
