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
InflationLayerInterface for nav2_costmap_2d_py.

Abstract interface for inflation layers, providing common methods needed by
navigation components regardless of the inflation implementation.
It mirrors the nav2_costmap_2d::InflationLayerInterface from the C++
implementation.
"""

from abc import abstractmethod
from typing import Any, Optional

from nav2_costmap_2d_py.core.layer import Layer


class InflationLayerInterface(Layer):
    """Abstract interface for inflation layers."""

    @abstractmethod
    def get_cost_scaling_factor(self) -> float:
        """
        Get the cost scaling factor.

        Returns
        -------
        float
            The cost scaling factor.

        """
        ...

    @abstractmethod
    def get_inflation_radius(self) -> float:
        """
        Get the inflation radius.

        Returns
        -------
        float
            The inflation radius in meters.

        """
        ...

    @abstractmethod
    def get_mutex(self) -> Any:
        """
        Get the mutex of the inflation information.

        Returns
        -------
        Any
            The mutex guarding the inflation data.

        """
        ...

    @abstractmethod
    def compute_cost(self, distance: float) -> int:
        """
        Given a distance, compute a cost.

        Parameters
        ----------
        distance : float
            The distance from an obstacle, in cells.

        Returns
        -------
        int
            A cost value for the distance.

        """
        ...

    @staticmethod
    def get_inflation_layer(
        costmap_ros: Any, layer_name: str = ''
    ) -> Optional['InflationLayerInterface']:
        """
        Get the inflation layer from a costmap.

        Checks every plugin and returns the first that is an
        ``InflationLayerInterface`` (optionally matching ``layer_name``).

        Parameters
        ----------
        costmap_ros : Costmap2DROS
            The costmap ROS wrapper.
        layer_name : str
            Optional name of the specific layer to find.

        Returns
        -------
        InflationLayerInterface or None
            The inflation layer interface, or None if not found.

        """
        layered_costmap = costmap_ros.get_layered_costmap()
        if layered_costmap is None:
            return None
        for layer in layered_costmap.get_plugins():
            if isinstance(layer, InflationLayerInterface):
                if not layer_name or layer.get_name() == layer_name:
                    return layer
        return None
