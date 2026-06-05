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
Layer for nav2_costmap_2d_py.

Abstract base class for all costmap layer plugins.

It mirrors the nav2_costmap_2d::Layer from the C++ implementation.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, List, Tuple, TYPE_CHECKING

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D

if TYPE_CHECKING:
    # Imported only for type hints; importing at runtime would create a
    # circular import (layered_costmap imports Layer under TYPE_CHECKING too).
    from nav2_costmap_2d_py.core.layered_costmap import LayeredCostmap


class Layer(ABC):
    """Abstract base class for costmap layer plugins."""

    def __init__(self) -> None:
        """Initialize the layer state with defaults."""
        self._name: str = ''
        self._enabled: bool = False
        self._current: bool = False
        # Typed as Any (not Optional): these are assigned real objects in
        # initialize() before any layer method runs. They come from untyped
        # sources anyway, so Any avoids ~25 spurious mypy union-attr errors
        # across all layer/filter subclasses without per-call asserts.
        self._layered_costmap: Any = None   # LayeredCostmap, set by initialize
        self._tf_buffer: Any = None
        self._node: Any = None
        # Footprint cache (set by LayeredCostmap.set_footprint)
        self._footprint: List[Tuple[float, float]] = []

    # ------------------------------------------------------------------
    # Public lifecycle API – called by LayeredCostmap
    # ------------------------------------------------------------------

    def initialize(
        self,
        layered_costmap: 'LayeredCostmap',
        name: str,
        tf_buffer: Any,
        node: Any,
    ) -> None:
        """
        Initialize the layer on startup. Called once by LayeredCostmap during configuration.

        Do **not** override this; override ``on_initialize()`` instead.

        Parameters
        ----------
        layered_costmap : LayeredCostmap
            The parent LayeredCostmap that owns this layer.
        name : str
            The name of the layer.
        tf_buffer : tf2_ros.Buffer
            The tf2 buffer used for coordinate transforms.
        node : Costmap2DROS
            The lifecycle node that owns the costmap.

        """
        self._layered_costmap = layered_costmap
        self._name = name
        self._tf_buffer = tf_buffer
        self._node = node
        self.on_initialize()

    # ------------------------------------------------------------------
    # Plugin hooks – override these in subclasses
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """
        Implement subclass-specific initialization. Called at the end of ``initialize()``.

        Called after ``initialize()`` sets up the layer references.
        Override this instead of ``initialize()``.
        """
        pass

    @abstractmethod
    def update_bounds(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        min_x: List[float],   # mutable reference via single-element list
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """
        Poll the plugin for how much of the costmap it needs to update.

        Called by LayeredCostmap to expand the bounding box to include this
        layer's dirty region.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists used as mutable references (Python's
            substitute for C++ ``double &`` output params).
            Expand them as needed; do NOT shrink.

        """
        ...

    @abstractmethod
    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """
        Actually update the underlying costmap, only within the bounds from ``update_bounds()``.

        Parameters
        ----------
        master_grid : Costmap2D
            The combined costmap (output of LayeredCostmap).
        min_i, min_j, max_i, max_j : int
            Cell-index bounds (exclusive upper end) of the dirty region.

        """
        ...

    def activate(self) -> None:
        """Restart publishers if they've been stopped."""
        pass

    def deactivate(self) -> None:
        """Stop publishers."""
        pass

    def reset(self) -> None:
        """Reset this costmap layer. Called by resetLayers(); default clears the current flag."""
        self._current = False

    def match_size(self) -> None:
        """
        Make this layer match the size of the parent costmap.

        Default implementation is a no-op; override in layers that keep their
        own Costmap2D.
        """
        pass

    def on_footprint_changed(self) -> None:
        """Handle a footprint change. Called by LayeredCostmap whenever the footprint changes."""
        pass

    def is_clearable(self) -> bool:
        """
        Return whether clearing operations should be processed on this layer.

        Returns
        -------
        bool
            Whether the layer is clearable. Default ``False``; override in
            obstacle layers.

        """
        return False

    # ------------------------------------------------------------------
    # Accessors / mutators
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        """
        Get the name of the costmap layer.

        Returns
        -------
        str
            The layer name.

        """
        return self._name

    @property
    def enabled(self) -> bool:
        """
        Get whether the layer is enabled.

        Returns
        -------
        bool
            Whether the layer is enabled.

        """
        return self._enabled

    @enabled.setter
    def enabled(self, val: bool) -> None:
        """
        Set whether the layer is enabled.

        Parameters
        ----------
        val : bool
            Whether the layer should be enabled.

        """
        self._enabled = val

    def is_enabled(self) -> bool:
        """
        Get whether the layer is enabled.

        Returns
        -------
        bool
            Whether the layer is enabled.

        """
        return self._enabled

    def is_current(self) -> bool:
        """
        Check whether all the data in the layer is up to date (not stale).

        Returns
        -------
        bool
            Whether the layer's data is current.

        """
        return self._current

    def set_current(self, current: bool) -> None:
        """
        Set the current status of the layer.

        Parameters
        ----------
        current : bool
            Whether the layer's data is up to date.

        """
        self._current = current

    def get_name(self) -> str:
        """
        Get the name of the costmap layer.

        Returns
        -------
        str
            The layer name.

        """
        return self._name

    def _declare_parameter_if_not_declared(self, param: str, default: Any) -> Any:
        """
        Declare a parameter if it has not been declared yet.

        Parameters
        ----------
        param : str
            Name of the parameter to declare.
        default: Any
            Default value for the parameter.

        Returns
        -------
        Any
            The value of the parameter after declaration.

        """
        full = f'{self._name}.{param}'
        if not self._node.has_parameter(full):
            self._node.declare_parameter(full, default)
        return self._node.get_parameter(full).value

    def join_with_parent_namespace(self, topic: str) -> str:
        """
        Resolve a relative topic against the costmap's *parent* namespace.

        The costmap node lives in a sub-namespace (e.g. ``/global_costmap``), so
        a relative topic such as ``map`` would resolve to ``/global_costmap/map``
        instead of the ``/map`` published by map_server. Strip the costmap's own
        namespace segment and join the topic to the parent, exactly like the C++
        ``nav2_costmap_2d::Layer::joinWithParentNamespace``. Absolute topics
        (leading ``/``) are returned unchanged.

        Parameters
        ----------
        topic : str
            The (possibly relative) topic name to resolve.

        Returns
        -------
        str
            The topic joined to the parent namespace, or unchanged if absolute.

        """
        if topic and not topic.startswith('/'):
            node_ns = self._node.get_namespace()
            parent_ns = node_ns[:node_ns.rfind('/')]
            return f'{parent_ns}/{topic}'
        return topic

    def get_layered_costmap(self) -> 'LayeredCostmap':
        """
        Return the parent LayeredCostmap that owns this layer.

        Returns
        -------
        LayeredCostmap
            The parent layered costmap.

        """
        return self._layered_costmap

    def get_footprint(self) -> List[Tuple[float, float]]:
        """
        Return the layered costmap's footprint (convenience accessor).

        Returns
        -------
        list of tuple of float
            The robot footprint as ``(x, y)`` points.

        """
        return self._footprint

    def set_footprint(self, footprint: List[Tuple[float, float]]) -> None:
        """
        Set the cached footprint and notify the layer via ``on_footprint_changed()``.

        Parameters
        ----------
        footprint : list of tuple of float
            The robot footprint as a list of ``(x, y)`` points.

        """
        self._footprint = footprint
        self.on_footprint_changed()
