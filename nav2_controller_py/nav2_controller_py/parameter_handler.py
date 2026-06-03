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
Parameter handler for Nav2 Controller Server.

It mirrors the nav2_controller::ParameterHandler from the C++ implementation.
Manages parameter declarations, validation, and dynamic parameter updates.
"""

from dataclasses import dataclass, field
import threading
from typing import List

from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode


@dataclass
class Parameters:
    """Container for Controller Server parameters."""

    controller_frequency: float = 20.0
    min_x_velocity_threshold: float = 0.0001
    min_y_velocity_threshold: float = 0.0001
    min_theta_velocity_threshold: float = 0.0001
    speed_limit_topic: str = 'speed_limit'
    failure_tolerance: float = 0.0
    use_realtime_priority: bool = False
    publish_zero_velocity: bool = True
    costmap_update_timeout: Duration = field(
        default_factory=lambda: Duration(seconds=0.30))
    odom_topic: str = 'odom'
    odom_duration: float = 0.3
    search_window: float = 2.0
    progress_checker_ids: List[str] = field(default_factory=list)
    progress_checker_types: List[str] = field(default_factory=list)
    goal_checker_ids: List[str] = field(default_factory=list)
    goal_checker_types: List[str] = field(default_factory=list)
    controller_ids: List[str] = field(default_factory=list)
    controller_types: List[str] = field(default_factory=list)
    path_handler_ids: List[str] = field(default_factory=list)
    path_handler_types: List[str] = field(default_factory=list)


class ParameterHandler:
    """
    Handles parameters and dynamic parameter updates for the Controller Server.

    Responsibilities:
        - Declare and retrieve ROS2 parameters
        - Validate parameter updates
        - Apply parameter changes with thread safety
        - Provide current parameter values via mutex-protected access
    """

    def __init__(self, node: LifecycleNode) -> None:
        """
        Initialize the parameter handler.

        Parameters
        ----------
        node : LifecycleNode
            The ROS2 lifecycle node for parameter management.

        """
        self._node = node
        self._logger = node.get_logger()
        self._mutex = threading.RLock()
        self._params = Parameters()

        self._default_progress_checker_ids = ['progress_checker']
        self._default_progress_checker_types = ['nav2_controller_py/SimpleProgressChecker']
        self._default_goal_checker_ids = ['goal_checker']
        self._default_goal_checker_types = ['nav2_controller_py/SimpleGoalChecker']
        self._default_controller_ids = ['FollowPath']
        self._default_controller_types = ['nav2_mppi_controller::MPPIController']
        self._default_path_handler_ids = ['PathHandler']
        self._default_path_handler_types = ['nav2_controller_py/FeasiblePathHandler']

        self._node.add_on_set_parameters_callback(self._on_set_parameters_callback)
        self._load_parameters()

    def _load_parameters(self) -> None:
        """Declare and load parameters from the ROS2 node configuration."""
        p = self._params
        p.controller_frequency = self.declare_or_get_parameter('controller_frequency', 20.0)
        p.min_x_velocity_threshold = self.declare_or_get_parameter(
            'min_x_velocity_threshold', 0.0001)
        p.min_y_velocity_threshold = self.declare_or_get_parameter(
            'min_y_velocity_threshold', 0.0001)
        p.min_theta_velocity_threshold = self.declare_or_get_parameter(
            'min_theta_velocity_threshold', 0.0001)
        p.speed_limit_topic = self.declare_or_get_parameter('speed_limit_topic', 'speed_limit')
        p.failure_tolerance = self.declare_or_get_parameter('failure_tolerance', 0.0)
        p.use_realtime_priority = self.declare_or_get_parameter('use_realtime_priority', False)
        p.publish_zero_velocity = self.declare_or_get_parameter('publish_zero_velocity', True)
        costmap_update_timeout_dbl = self.declare_or_get_parameter('costmap_update_timeout', 0.30)
        p.costmap_update_timeout = Duration(seconds=costmap_update_timeout_dbl)
        p.odom_topic = self.declare_or_get_parameter('odom_topic', 'odom')
        p.odom_duration = self.declare_or_get_parameter('odom_duration', 0.3)
        p.search_window = self.declare_or_get_parameter('search_window', 2.0)

        self._logger.info(
            f'Controller frequency set to {p.controller_frequency:.4f}Hz')

        self._logger.info('getting progress checker plugins..')
        p.progress_checker_ids = self.declare_or_get_parameter(
            'progress_checker_plugins', self._default_progress_checker_ids)
        if p.progress_checker_ids == self._default_progress_checker_ids:
            for pid, ptype in zip(self._default_progress_checker_ids,
                                  self._default_progress_checker_types):
                self.declare_or_get_parameter(f'{pid}.plugin', ptype)

        self._logger.info('getting goal checker plugins..')
        p.goal_checker_ids = self.declare_or_get_parameter(
            'goal_checker_plugins', self._default_goal_checker_ids)
        if p.goal_checker_ids == self._default_goal_checker_ids:
            for pid, ptype in zip(self._default_goal_checker_ids,
                                  self._default_goal_checker_types):
                self.declare_or_get_parameter(f'{pid}.plugin', ptype)

        self._logger.info('getting controller plugins..')
        p.controller_ids = self.declare_or_get_parameter(
            'controller_plugins', self._default_controller_ids)
        if p.controller_ids == self._default_controller_ids:
            for pid, ptype in zip(self._default_controller_ids,
                                  self._default_controller_types):
                self.declare_or_get_parameter(f'{pid}.plugin', ptype)

        self._logger.info('getting path handler plugins..')
        p.path_handler_ids = self.declare_or_get_parameter(
            'path_handler_plugins', self._default_path_handler_ids)
        if p.path_handler_ids == self._default_path_handler_ids:
            for pid, ptype in zip(self._default_path_handler_ids,
                                  self._default_path_handler_types):
                self.declare_or_get_parameter(f'{pid}.plugin', ptype)

        p.progress_checker_types = [''] * len(p.progress_checker_ids)
        p.goal_checker_types = [''] * len(p.goal_checker_ids)
        p.controller_types = [''] * len(p.controller_ids)
        p.path_handler_types = [''] * len(p.path_handler_ids)

        for i, pid in enumerate(p.progress_checker_ids):
            try:
                p.progress_checker_types[i] = self._get_plugin_type_param(pid)
            except RuntimeError as ex:
                raise RuntimeError(
                    f"Failed to get type for progress_checker '{pid}': {ex}") from ex

        for i, pid in enumerate(p.goal_checker_ids):
            try:
                p.goal_checker_types[i] = self._get_plugin_type_param(pid)
            except RuntimeError as ex:
                raise RuntimeError(
                    f"Failed to get type for goal_checker '{pid}': {ex}") from ex

        for i, pid in enumerate(p.controller_ids):
            try:
                p.controller_types[i] = self._get_plugin_type_param(pid)
            except RuntimeError as ex:
                raise RuntimeError(
                    f"Failed to get type for controller plugins '{pid}': {ex}") from ex

        for i, pid in enumerate(p.path_handler_ids):
            try:
                p.path_handler_types[i] = self._get_plugin_type_param(pid)
            except RuntimeError as ex:
                raise RuntimeError(
                    f"Failed to get type for path handler plugins '{pid}': {ex}") from ex

    def _get_plugin_type_param(self, plugin_name: str) -> str:
        """
        Read the '<plugin_name>.plugin' parameter.

        Parameters
        ----------
        plugin_name : str
            Name of the plugin.

        Returns
        -------
        str
            Fully qualified class name of the plugin.

        Raises
        ------
        RuntimeError
            If the parameter is not defined or cannot be retrieved.

        """
        param_name = f'{plugin_name}.plugin'
        plugin_type = self.declare_or_get_parameter(param_name, '')
        if not plugin_type:
            self._logger.fatal(f"Can not get 'plugin' param value for {plugin_name}")
            raise RuntimeError(
                f"No 'plugin' param for plugin '{plugin_name}'. "
                f'Add {param_name}: <fully_qualified_class> to your YAML.'
            )
        return plugin_type

    def _on_set_parameters_callback(self, parameters) -> SetParametersResult:
        """
        Validate and apply parameter changes with mutex protection.

        Parameters
        ----------
        parameters : list
            List of parameters being updated.

        Returns
        -------
        SetParametersResult
            Result indicating whether the update was accepted.

        """
        result = self._validate_parameters(parameters)
        if not result.successful:
            return result
        self._update_parameters(parameters)
        return result

    def _validate_parameters(self, parameters) -> SetParametersResult:
        """
        Validate incoming parameter updates before applying them.

        Parameters
        ----------
        parameters : list
            List of parameters to validate.

        Returns
        -------
        SetParametersResult
            Result indicating whether the update is accepted.

        """
        result = SetParametersResult(successful=True)
        for param in parameters:
            param_name = param.name
            # Plugins handle their own parameter changes
            if '.' in param_name:
                continue
            if isinstance(param.value, float):
                if param.value < 0.0 and param_name != 'failure_tolerance':
                    self._logger.warning(
                        f"The value of parameter '{param_name}' is incorrectly set to "
                        f'{param.value}, it should be >=0. Ignoring parameter update.'
                    )
                    result.successful = False
        return result

    def _update_parameters(self, parameters) -> None:
        """
        Apply validated parameter updates with thread safety.

        Parameters
        ----------
        parameters : list
            List of validated parameters to apply.

        """
        with self._mutex:
            for param in parameters:
                param_name = param.name
                if '.' in param_name:
                    continue
                if not isinstance(param.value, float):
                    continue
                if param_name == 'min_x_velocity_threshold':
                    self._params.min_x_velocity_threshold = param.value
                elif param_name == 'min_y_velocity_threshold':
                    self._params.min_y_velocity_threshold = param.value
                elif param_name == 'min_theta_velocity_threshold':
                    self._params.min_theta_velocity_threshold = param.value
                elif param_name == 'failure_tolerance':
                    self._params.failure_tolerance = param.value
                elif param_name == 'search_window':
                    self._params.search_window = param.value

    def get_parameters(self) -> Parameters:
        """
        Get the current parameter values.

        Returns
        -------
            Parameters: The current parameter values.

        """
        with self._mutex:
            return self._params

    def declare_or_get_parameter(self, name: str, default_value):
        """
        Declare a parameter if not already declared, or get its value if it exists.

        Parameters
        ----------
        name : str
            Parameter name.
        default_value
            Default value for the parameter if it needs to be declared.

        Returns
        -------
        The current or default value of the parameter.

        """
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        try:
            declared = self._node.declare_parameter(name, default_value)
            return declared.value
        except Exception as ex:
            self._logger.warning(
                f"Could not declare parameter '{name}' with default value "
                f"'{default_value}': {ex}. Using default value."
            )
        return default_value

    def get_mutex(self):
        """
        Get the mutex for thread-safe parameter access.

        Returns
        -------
        threading.RLock
            Mutex for synchronizing parameter access.

        """
        return self._mutex
