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
PluginContainerLayer for nav2_costmap_2d_py.

Holds a list of plugins and applies them only to this layer, combining the
result into the master costmap with a configurable combination method.

It mirrors the nav2_costmap_2d::PluginContainerLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/PluginContainerLayer"``
"""

from typing import List

from nav2_costmap_2d_py.core.cost_values import CombinationMethod, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.layer import Layer


class PluginContainerLayer(CostmapLayer):
    """Holds a list of plugins and applies them only to this layer."""

    def __init__(self) -> None:
        """Initialize plugin container layer defaults."""
        super().__init__()
        self._combination_method = CombinationMethod.Max
        self._plugins: List[Layer] = []
        self._plugin_names: List[str] = []
        self._plugin_types: List[str] = []

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer: read parameters and load the contained plugins."""
        node = self._node
        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._plugin_names = list(self._declare_parameter_if_not_declared('plugins', []))
        combination_method_param = self._declare_parameter_if_not_declared(
            'combination_method', 1)
        self._combination_method = self.combination_method_from_int(combination_method_param)

        self._plugin_types = [''] * len(self._plugin_names)
        for i, child_name in enumerate(self._plugin_names):
            full_name = f'{self._name}.{child_name}'
            self._plugin_types[i] = node._get_plugin_type_param(full_name)
            plugin = node._plugin_provider.load(self._plugin_types[i], node)
            self.add_plugin(plugin, child_name)

        self._default_value = NO_INFORMATION

        self.match_size()
        self.set_current(True)

    def add_plugin(self, plugin: Layer, layer_name: str) -> None:
        """
        Add a contained plugin and initialize it under this layer's namespace.

        Parameters
        ----------
        plugin : Layer
            The plugin layer to add.
        layer_name : str
            The (bare) name of the contained plugin.

        """
        self._plugins.append(plugin)
        plugin.initialize(
            self._layered_costmap, f'{self._name}.{layer_name}',
            self._tf_buffer, self._node)

    def update_bounds(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """Update the bounds by polling each contained plugin."""
        for plugin in self._plugins:
            plugin.update_bounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y)

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """Run each contained plugin on this layer, then combine into the master."""
        with self.get_mutex():
            if not self._enabled:
                return

            for plugin in self._plugins:
                plugin.update_costs(self, min_i, min_j, max_i, max_j)

            if self._combination_method == CombinationMethod.Overwrite:
                self.update_with_overwrite(master_grid, min_i, min_j, max_i, max_j)
            elif self._combination_method == CombinationMethod.Max:
                self.update_with_max(master_grid, min_i, min_j, max_i, max_j)
            elif self._combination_method == CombinationMethod.MaxWithoutUnknownOverwrite:
                self.update_with_max_without_unknown_overwrite(
                    master_grid, min_i, min_j, max_i, max_j)

            self.set_current(True)

    def activate(self) -> None:
        """Activate the layer and all contained plugins."""
        for plugin in self._plugins:
            plugin.activate()

    def deactivate(self) -> None:
        """Deactivate the layer and all contained plugins."""
        for plugin in self._plugins:
            plugin.deactivate()

    def reset(self) -> None:
        """Reset all contained plugins and this layer's grid."""
        for plugin in self._plugins:
            plugin.reset()
        self.reset_maps()
        self.set_current(False)

    def on_footprint_changed(self) -> None:
        """Notify all contained plugins of a footprint change."""
        for plugin in self._plugins:
            plugin.on_footprint_changed()

    def match_size(self) -> None:
        """Match the size of the master costmap and propagate to contained plugins."""
        with self.get_mutex():
            master = self._layered_costmap.get_costmap()
            self.resize_map(
                master.size_x, master.size_y, master.resolution,
                master.origin_x, master.origin_y)
            for plugin in self._plugins:
                plugin.match_size()

    def is_clearable(self) -> bool:
        """Return True if any contained plugin is clearable."""
        return any(plugin.is_clearable() for plugin in self._plugins)

    def clear_area(
        self, start_x: int, start_y: int, end_x: int, end_y: int, invert: bool = False
    ) -> None:
        """Clear an area in this layer and in each clearable contained plugin."""
        super().clear_area(start_x, start_y, end_x, end_y, invert)
        for plugin in self._plugins:
            if plugin.is_clearable() and isinstance(plugin, CostmapLayer):
                plugin.clear_area(start_x, start_y, end_x, end_y, invert)

    def get_plugins(self) -> List[Layer]:
        """Return the list of contained plugins."""
        return self._plugins
