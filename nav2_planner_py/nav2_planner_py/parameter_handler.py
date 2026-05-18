# Copyright (c) 2025 Nav2 Python Port
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
Parameter handler for Nav2 Planner Server.

It mirrors the nav2_planner::ParameterHandler from the C++ implementation.
Manages parameter declarations, validation, and dynamic parameter updates.
"""

import threading
from dataclasses import dataclass, field
from typing import List

from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode


@dataclass
class Parameters:
    """Container for Planner Server parameters."""

    planner_ids: List[str] = field(default_factory=list)
    planner_types: List[str] = field(default_factory=list)
    max_planner_duration: float = 0.0
    costmap_update_timeout: Duration = field(
        default_factory=lambda: Duration(seconds=0.0))
    partial_plan_allowed: bool = False


class ParameterHandler:
    """
    Handles parameters and dynamic parameter updates for Planner Server.

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
        self._mutex = threading.Lock()
        self._params = Parameters()
        self._default_ids: List[str] = ['']
        self._default_types: List[str] = ['nav2_planner_py/DefaultPlanner']

        # Add callback for parameter updates
        self._node.add_on_set_parameters_callback(
            self._on_set_parameters_callback
        )

        # Declare and load parameters
        self._load_parameters()

    def _load_parameters(self) -> None:
        """Load parameters from ROS2 node configuration."""
        # Declare planner_plugins parameter
        self._params.planner_ids = self.declare_or_get_parameter(
            'planner_plugins', self._default_ids
        )

        # Declare expected_planner_frequency parameter
        expected_planner_frequency = self.declare_or_get_parameter(
            'expected_planner_frequency', 1.0
        )

        # Calculate max_planner_duration from frequency
        if expected_planner_frequency > 0:
            self._params.max_planner_duration = 1.0 / expected_planner_frequency
        else:
            self._logger.warning(
                'The expected planner frequency parameter is %.4f Hz. '
                'The value should be greater than 0.0 to turn on duration '
                f'overrun warning messages {expected_planner_frequency}',
            )
            self._params.max_planner_duration = 0.0

        # Declare costmap_update_timeout parameter
        costmap_update_timeout_dbl = self.declare_or_get_parameter(
            'costmap_update_timeout', 1.0
        )
        self._params.costmap_update_timeout = Duration(
            seconds=costmap_update_timeout_dbl
        )

        # Declare allow_partial_planning parameter
        self._params.partial_plan_allowed = self.declare_or_get_parameter(
            'allow_partial_planning', False
        )

        # Declare default plugin parameters if using defaults
        if self._params.planner_ids == self._default_ids:
            for default_id, default_type in zip(self._default_ids, self._default_types):
                param_name = f'{default_id}.plugin'
                self.declare_or_get_parameter(
                    param_name, default_type)

        # Load planner types from parameters
        self._params.planner_types = [''] * len(self._params.planner_ids)
        for i, planner_id in enumerate(self._params.planner_ids):
            try:
                plugin_type = self._get_plugin_type_param(planner_id)
                self._params.planner_types[i] = plugin_type
            except RuntimeError as ex:
                raise RuntimeError(
                    f'Failed to get plugin type for planner {planner_id}. '
                    f'Exception: {ex}'
                ) from ex

    def _declare_parameter_if_not_declared(
        self, name: str, default_value
    ) -> None:
        """
        Declare a parameter only if it has not been declared yet.

        Parameters
        ----------
        name : str
            Parameter name.
        default_value
            Default value for the parameter.
        """
        if not self._node.has_parameter(name):
            self._node.declare_parameter(name, default_value)

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

        try:
            if not plugin_type:
                self._logger.fatal(
                    f"Can not get 'plugin' param value for {plugin_name}"
                )
                raise RuntimeError(
                    f"No 'plugin' param for plugin '{plugin_name}'. "
                    f'Add {param_name}: <fully_qualified_class> to your YAML.'
                )
            return plugin_type
        except RuntimeError:
            raise
        except Exception as ex:
            self._logger.fatal(
                f"'plugin' param not defined for {plugin_name}"
            )
            raise RuntimeError(
                f"No 'plugin' param for plugin '{plugin_name}'. "
                f'Exception: {ex}'
            ) from ex

    def _on_set_parameters_callback(self, parameters) -> SetParametersResult:
        """
        Callback triggered when parameters are being updated.

        This validates and applies parameter changes with mutex protection.

        Parameters
        ----------
        parameters : list
            List of parameters being updated.

        Returns
        -------
        SetParametersResult
            Result indicating whether the update was accepted.
        """
        # Validate parameters
        if not self._validate_parameters(parameters):
            return SetParametersResult(
                successful=False, reason='Parameter validation failed'
            )

        # Apply validated parameter updates
        self._update_parameters(parameters)
        return SetParametersResult(successful=True)

    def _validate_parameters(self, parameters) -> bool:
        """
        Validate incoming parameter updates.

        Mirrors validateParameterUpdatesCallback in C++.

        Parameters
        ----------
        parameters : list
            List of parameters to validate.

        Returns
        -------
        bool
            True if all parameters are valid, False otherwise.
        """
        for param in parameters:
            param_name = param.name
            param_value = param.value

            # Skip plugin-specific parameters (contain '.')
            if '.' in param_name:
                continue

            # Validate double/float parameters (should be > 0)
            if isinstance(param_value, float):
                if param_value <= 0.0:
                    self._logger.warning(
                        f"The value of parameter '{param_name}' is incorrectly "
                        f'set to {param_value}, it should be >0. '
                        'Ignoring parameter update.'
                    )
                    return False

        return True

    def _update_parameters(self, parameters) -> None:
        """
        Apply validated parameter updates.

        Applies updates with mutex protection for thread safety.

        Parameters
        ----------
        parameters : list
            List of validated parameters to apply.
        """
        with self._mutex:
            for param in parameters:
                param_name = param.name
                param_value = param.value

                # Skip plugin-specific parameters
                if '.' in param_name:
                    continue

                # Update expected_planner_frequency
                if param_name == 'expected_planner_frequency':
                    if isinstance(param_value, (int, float)) and param_value > 0:
                        self._params.max_planner_duration = 1.0 / param_value

                # Update allow_partial_planning
                elif param_name == 'allow_partial_planning':
                    if isinstance(param_value, bool):
                        self._params.partial_plan_allowed = param_value

                # Update costmap_update_timeout
                elif param_name == 'costmap_update_timeout':
                    if isinstance(param_value, (int, float)):
                        self._params.costmap_update_timeout = Duration(
                            seconds=param_value
                        )

    def get_parameters(self) -> Parameters:
        """
        Get current parameters with mutex protection.

        Returns
        -------
        Parameters
            Current parameter values.
        """
        with self._mutex:
            return self._params

    def declare_or_get_parameter(
        self, name: str, default_value
    ):
        """
        Declare a parameter if not already declared, or get its value if it exists.

        If the parameter is already declared, returns its current value.
        Otherwise, declares the parameter with the provided default value
        and returns that default value. If the declaration fails (e.g., empty list
        with indeterminate type), the default value is returned directly.

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
            # declare_parameter returns the actual value after applying overrides (e.g. from YAML)
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
        threading.Lock
            Mutex for synchronizing parameter access.
        """
        return self._mutex
