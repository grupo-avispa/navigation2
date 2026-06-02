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
ObstacleLayer for nav2_costmap_2d_py.

Marks/clears cells from laser scan and point-cloud sensor observations.

It mirrors the nav2_costmap_2d::ObstacleLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/ObstacleLayer"``
"""

import math
import threading
from typing import Any, Dict, List, Optional, Tuple

from sensor_msgs.msg import LaserScan

from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE,
    LETHAL_OBSTACLE,
    NO_INFORMATION,
)


class ObstacleLayer(CostmapLayer):
    """
    Marks laser-scan obstacle cells in the costmap.

    Parameters (under ``<name>.``):
      enabled                  (bool,  default True)
      observation_sources      (str,   default 'scan')
      combination_method       (int,   default 1  → max)
      footprint_clearing_enabled (bool, default True)
      max_obstacle_height      (float, default 2.0 m)

    Each source ``<src>`` has:
      <name>.<src>.topic             (str,   default '/<src>')
      <name>.<src>.data_type         (str,   default 'LaserScan')
      <name>.<src>.obstacle_max_range (float, default 2.5 m)
      <name>.<src>.obstacle_min_range (float, default 0.0 m)
      <name>.<src>.raytrace_max_range (float, default 3.0 m)
      <name>.<src>.raytrace_min_range (float, default 0.0 m)
      <name>.<src>.marking           (bool,  default True)
      <name>.<src>.clearing          (bool,  default True)
    """

    def __init__(self) -> None:
        super().__init__()
        self._combination_method = 1   # 1 = max
        self._footprint_clearing_enabled = True
        self._max_obstacle_height = 2.0
        self._observation_sources: List[str] = []
        self._subs: List[Any] = []
        self._observations: List[Dict[str, Any]] = []   # {buffer, marking, clearing}
        self._obs_lock = threading.Lock()

        # Dirty bounds
        self._min_x = self._min_y = float('inf')
        self._max_x = self._max_y = float('-inf')

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        node = self._node
        name = self._name

        def _p(param, default):
            full = f'{name}.{param}'
            if not node.has_parameter(full):
                node.declare_parameter(full, default)
            return node.get_parameter(full).value

        self._enabled = _p('enabled', True)
        src_string = _p('observation_sources', 'scan')
        self._combination_method = _p('combination_method', 1)
        self._footprint_clearing_enabled = _p('footprint_clearing_enabled', True)
        self._max_obstacle_height = _p('max_obstacle_height', 2.0)

        sources = [s.strip() for s in src_string.split() if s.strip()]
        self._observation_sources = sources

        for src in sources:
            def _ps(param, default, _src=src):
                full = f'{name}.{_src}.{param}'
                if not node.has_parameter(full):
                    node.declare_parameter(full, default)
                return node.get_parameter(full).value

            # Resolve relative topics against the parent namespace;
            # the costmap node lives in the '/global_costmap' sub-namespace.
            topic = self.join_with_parent_namespace(_ps('topic', f'/{src}'))
            marking = _ps('marking', True)
            clearing = _ps('clearing', True)
            obs_max = _ps('obstacle_max_range', 2.5)
            obs_min = _ps('obstacle_min_range', 0.0)
            ray_max = _ps('raytrace_max_range', 3.0)
            ray_min = _ps('raytrace_min_range', 0.0)

            obs_buffer: List = []
            # Closure captures obs_buffer, marking, clearing, ranges

            def _make_cb(buf, mk, cl, o_max, o_min, r_max, r_min):
                def cb(msg):
                    self._laser_callback(
                        msg, buf, mk, cl, o_max, o_min, r_max, r_min
                    )
                return cb

            sub = node.create_subscription(
                LaserScan, topic,
                _make_cb(obs_buffer, marking, clearing,
                         obs_max, obs_min, ray_max, ray_min),
                10,
            )
            self._subs.append(sub)
            with self._obs_lock:
                self._observations.append({
                    'buffer': obs_buffer,
                    'marking': marking,
                    'clearing': clearing,
                })

            node.get_logger().info(
                f'[ObstacleLayer] "{name}" subscribed to "{topic}" '
                f'(mark={marking}, clear={clearing})'
            )

    def update_bounds(self, robot_x, robot_y, robot_yaw,
                      min_x, min_y, max_x, max_y) -> None:
        if not self._enabled:
            return

        # Clear footprint region if enabled
        if self._footprint_clearing_enabled:
            fp = self._layered_costmap.get_footprint()
            self._clear_footprint_region(robot_x, robot_y, robot_yaw, fp)

        # Expand bounds to cover all current observations
        with self._obs_lock:
            for obs in self._observations:
                for pt in obs['buffer']:
                    ox, oy = pt
                    min_x[0] = min(min_x[0], ox)
                    min_y[0] = min(min_y[0], oy)
                    max_x[0] = max(max_x[0], ox)
                    max_y[0] = max(max_y[0], oy)

    def update_costs(self, master_grid: Costmap2D,
                     min_i: int, min_j: int,
                     max_i: int, max_j: int) -> None:
        if not self._enabled:
            return

        with self._obs_lock:
            for obs in self._observations:
                if obs['marking']:
                    for (wx, wy) in obs['buffer']:
                        ok, mx, my = master_grid.world_to_map(wx, wy)
                        if ok and min_i <= mx < max_i and min_j <= my < max_j:
                            master_grid.set_cost(mx, my, LETHAL_OBSTACLE)

        self._current = True

    def reset(self) -> None:
        with self._obs_lock:
            for obs in self._observations:
                obs['buffer'].clear()
        self._current = False

    def is_clearable(self) -> bool:
        return True

    # ------------------------------------------------------------------
    # LaserScan callback
    # ------------------------------------------------------------------

    def _laser_callback(
        self,
        msg: LaserScan,
        buffer: list,
        marking: bool,
        clearing: bool,
        obs_max_range: float,
        obs_min_range: float,
        ray_max_range: float,
        ray_min_range: float,
    ) -> None:
        """
        Extract obstacle points from a LaserScan and store in *buffer*.

        Uses TF to transform to the global frame if available; otherwise
        uses the message header frame directly.
        """
        # Transform origin: try TF, fall back to identity
        ox, oy = 0.0, 0.0
        if self._tf_buffer is not None:
            try:
                import tf2_ros
                from rclpy.duration import Duration
                transform = self._tf_buffer.lookup_transform(
                    self._layered_costmap.get_global_frame_id() if self._layered_costmap is not None else '',
                    msg.header.frame_id,
                    msg.header.stamp,
                    timeout=Duration(seconds=0.1),
                )
                ox = transform.transform.translation.x
                oy = transform.transform.translation.y
            except Exception:
                pass

        angle = msg.angle_min
        new_pts = []
        for r in msg.ranges:
            angle += msg.angle_increment
            if math.isnan(r) or math.isinf(r):
                continue
            if r < obs_min_range or r > obs_max_range:
                continue
            wx = ox + r * math.cos(angle)
            wy = oy + r * math.sin(angle)
            new_pts.append((wx, wy))

        with self._obs_lock:
            buffer.clear()
            buffer.extend(new_pts)

        self._current = True

    def _clear_footprint_region(
        self, rx: float, ry: float, rth: float, footprint: list
    ) -> None:
        """Clear the cells under the robot footprint."""
        if not footprint:
            return
        if self._layered_costmap is None:
            return
        master = self._layered_costmap.get_costmap()
        from nav2_costmap_2d_py.core.layered_costmap import transform_footprint
        world_fp = transform_footprint(rx, ry, rth, footprint)
        master.set_convex_polygon_cost(world_fp, FREE_SPACE)
