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

Plugin authors subclass this and implement at minimum:
  - ``on_initialize()``
  - ``update_bounds()``
  - ``update_costs()``

The ``initialize()`` method is called by :class:`LayeredCostmap` and must
**not** be overridden – override ``on_initialize()`` instead (same pattern
as the C++ ``onInitialize()`` hook).

It mirrors the nav2_costmap_2d::Layer from the C++ implementation.
"""

from abc import ABC, abstractmethod
from typing import Any, Optional, Tuple

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D


class Layer(ABC):
    """
    Abstract base class for costmap layer plugins.
    """

    def __init__(self) -> None:
        self._name: str = ''
        self._enabled: bool = True
        self._current: bool = False
        self._layered_costmap: Optional[Any] = None   # LayeredCostmap reference (set by initialize)
        self._tf_buffer: Optional[Any] = None
        self._node: Optional[Any] = None
        # Footprint cache (set by LayeredCostmap.set_footprint)
        self._footprint: list = []

    # ------------------------------------------------------------------
    # Public lifecycle API – called by LayeredCostmap
    # ------------------------------------------------------------------

    def initialize(
        self,
        layered_costmap,       # LayeredCostmap
        name: str,
        tf_buffer,
        node,
    ) -> None:
        """
        Called once by LayeredCostmap during configuration.

        Do **not** override this; override ``on_initialize()`` instead.
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
        Plugin-specific initialisation hook.

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
        min_x: list,   # [float] – mutable reference via single-element list
        min_y: list,
        max_x: list,
        max_y: list,
    ) -> None:
        """
        Expand the bounding box to include this layer's dirty region.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw :
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y :
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
        Write this layer's costs into *master_grid* within the given bounds.

        Parameters
        ----------
        master_grid :
            The combined costmap (output of LayeredCostmap).
        min_i, min_j, max_i, max_j :
            Cell-index bounds (exclusive upper end) of the dirty region.
        """
        ...

    def activate(self) -> None:
        """Called during on_activate(). Default: no-op."""
        pass

    def deactivate(self) -> None:
        """Called during on_deactivate(). Default: no-op."""
        pass

    def reset(self) -> None:
        """Reset layer state. Called by resetLayers(). Default: clears current_."""
        self._current = False

    def match_size(self) -> None:
        """
        Resize this layer's internal grid to match the master costmap.

        Default implementation is a no-op; override in layers that keep their
        own Costmap2D.
        """
        pass

    def on_footprint_changed(self) -> None:
        """
        Called when the robot footprint changes.

        Default: no-op.
        """
        pass

    def is_clearable(self) -> bool:
        """
        Whether this layer can be cleared (e.g. by ClearCostmapService).

        Default: False.  Override in obstacle layers.
        """
        return False

    # ------------------------------------------------------------------
    # Accessors / mutators
    # ------------------------------------------------------------------

    @property
    def name(self) -> str:
        return self._name

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, val: bool) -> None:
        self._enabled = val

    def is_enabled(self) -> bool:
        return self._enabled

    def is_current(self) -> bool:
        """
        Return whether the layer's data is current (not stale).
        """
        return self._current

    def get_name(self) -> str:
        return self._name

    def join_with_parent_namespace(self, topic: str) -> str:
        """
        Resolve a relative topic against the costmap's *parent* namespace.

        The costmap node lives in a sub-namespace (e.g. ``/global_costmap``), so
        a relative topic such as ``map`` would resolve to ``/global_costmap/map``
        instead of the ``/map`` published by map_server. Strip the costmap's own
        namespace segment and join the topic to the parent, exactly like the C++
        ``nav2_costmap_2d::Layer::joinWithParentNamespace``. Absolute topics
        (leading ``/``) are returned unchanged.
        """
        if topic and not topic.startswith('/'):
            node_ns = self._node.get_namespace()
            parent_ns = node_ns[:node_ns.rfind('/')]
            return f'{parent_ns}/{topic}'
        return topic

    def get_layered_costmap(self):
        return self._layered_costmap

    def get_footprint(self) -> list:
        return self._footprint

    def set_footprint(self, footprint: list) -> None:
        self._footprint = footprint
        self.on_footprint_changed()
