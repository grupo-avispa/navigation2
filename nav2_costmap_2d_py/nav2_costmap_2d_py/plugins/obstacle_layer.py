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

Marks/clears cells from laser scan sensor observations.

It mirrors the nav2_costmap_2d::ObstacleLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/ObstacleLayer"``
"""

import math
import threading
from typing import Any, Callable, List, Optional, Tuple

from nav2_costmap_2d_py.core.cost_values import CombinationMethod, FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.layered_costmap import transform_footprint
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class Observation:
    """
    A buffered sensor observation.

    Holds the sensor ``origin`` and the return ``points`` (both already in the
    costmap global frame) plus the per-source marking/clearing ranges.
    """

    def __init__(self) -> None:
        """Initialize an empty observation."""
        self.origin: Optional[Tuple[float, float]] = None
        self.points: List[Tuple[float, float]] = []
        self.marking: bool = True
        self.clearing: bool = True
        self.obstacle_max_range: float = 2.5
        self.obstacle_min_range: float = 0.0
        self.raytrace_max_range: float = 3.0
        self.raytrace_min_range: float = 0.0


class ObstacleLayer(CostmapLayer):
    """
    Take in laser data to populate the 2D costmap.

    Marks and clears cells from laser scan sensor observations.
    """

    def __init__(self) -> None:
        """Initialize obstacle layer defaults."""
        super().__init__()
        self._combination_method = CombinationMethod.Max
        self._footprint_clearing_enabled = True
        self._max_obstacle_height = 2.0
        self._rolling_window = False
        self._observation_sources: List[str] = []
        self._subs: List[Any] = []
        self._observations: List[Observation] = []
        self._obs_lock = threading.Lock()
        self._was_reset = False

        # Footprint clearing cache (transformed footprint in world coordinates)
        self._transformed_footprint: List[Tuple[float, float]] = []

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer: read parameters and subscribe to observation sources."""
        node = self._node
        self._rolling_window = self._layered_costmap.is_rolling()

        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        src_string = self._declare_parameter_if_not_declared('observation_sources', 'scan')
        combination_method = self._declare_parameter_if_not_declared('combination_method', 1)
        self._combination_method = self.combination_method_from_int(combination_method)
        self._footprint_clearing_enabled = self._declare_parameter_if_not_declared(
            'footprint_clearing_enabled', True)
        self._max_obstacle_height = self._declare_parameter_if_not_declared(
            'max_obstacle_height', 2.0)

        self.match_size()
        self._current = True

        sources = [s.strip() for s in src_string.split() if s.strip()]
        self._observation_sources = sources

        for src in sources:
            def _ps(param: str, default: Any, _src: str = src) -> Any:
                full = f'{self._name}.{_src}.{param}'
                if not node.has_parameter(full):
                    node.declare_parameter(full, default)
                return node.get_parameter(full).value

            # Resolve relative topics against the parent namespace;
            # the costmap node lives in the '/global_costmap' sub-namespace.
            topic = self.join_with_parent_namespace(_ps('topic', f'/{src}'))

            obs = Observation()
            obs.marking = _ps('marking', True)
            obs.clearing = _ps('clearing', True)
            obs.obstacle_max_range = _ps('obstacle_max_range', 2.5)
            obs.obstacle_min_range = _ps('obstacle_min_range', 0.0)
            obs.raytrace_max_range = _ps('raytrace_max_range', 3.0)
            obs.raytrace_min_range = _ps('raytrace_min_range', 0.0)

            def _make_cb(o: Observation = obs) -> Callable[[LaserScan], None]:
                def cb(msg: LaserScan) -> None:
                    self._laser_scan_callback(msg, o)
                return cb

            sub = node.create_subscription(LaserScan, topic, _make_cb(), 10)
            self._subs.append(sub)
            with self._obs_lock:
                self._observations.append(obs)

            node.get_logger().info(
                f'[ObstacleLayer] "{self._name}" subscribed to "{topic}" '
                f'(mark={obs.marking}, clear={obs.clearing})'
            )

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
        """
        Update the bounds of the master costmap by this layer's update dimensions.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if self._rolling_window:
            self.update_origin(
                robot_x - self.size_x_meters / 2,
                robot_y - self.size_y_meters / 2)
        if not self._enabled:
            return
        self.use_extra_bounds(min_x, min_y, max_x, max_y)

        with self._obs_lock:
            observations = [self._snapshot(o) for o in self._observations]

        # raytrace freespace for every clearing observation
        for obs in observations:
            if obs.clearing:
                self.raytrace_freespace(obs, min_x, min_y, max_x, max_y)

        # mark obstacles for every marking observation
        for obs in observations:
            if not obs.marking or obs.origin is None:
                continue
            ox, oy = obs.origin
            max_range_cells = self.cell_distance(obs.obstacle_max_range)
            min_range_cells = self.cell_distance(obs.obstacle_min_range)

            ok, x0, y0 = self.world_to_map(ox, oy)
            if not ok:
                continue

            for (px, py) in obs.points:
                ok, mx, my = self.world_to_map(px, py)
                if not ok:
                    continue

                # Pre-filter by world distance to avoid cell-discretization
                # boundary effects.
                wdx = px - ox
                wdy = py - oy
                world_dist_sq = wdx * wdx + wdy * wdy
                if world_dist_sq > obs.obstacle_max_range * obs.obstacle_max_range:
                    continue
                if world_dist_sq < obs.obstacle_min_range * obs.obstacle_min_range:
                    continue

                # Distance in cell space, matching the raytrace clearing.
                dx = mx - x0
                dy = my - y0
                dist = int(math.hypot(dx, dy))
                if dist > max_range_cells:
                    continue
                if dist < min_range_cells:
                    continue

                self._costmap[self.get_index(mx, my)] = LETHAL_OBSTACLE
                self.touch(px, py, min_x, min_y, max_x, max_y)

        self.update_footprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y)

    def update_footprint(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """
        Expand the bounds to include the robot footprint (for footprint clearing).

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : float
            Current robot pose in the global frame.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if not self._footprint_clearing_enabled:
            return
        self._transformed_footprint = transform_footprint(
            robot_x, robot_y, robot_yaw, self._layered_costmap.get_footprint())
        for (fx, fy) in self._transformed_footprint:
            self.touch(fx, fy, min_x, min_y, max_x, max_y)

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int
    ) -> None:
        """
        Update the costs in the master costmap within the given window.

        Clears the footprint region (after marking, so the robot never marks
        itself), then combines this layer into the master with the configured
        combination method. Mirrors nav2_costmap_2d::ObstacleLayer::updateCosts.

        Parameters
        ----------
        master_grid : Costmap2D
            The master costmap to mark obstacles into.
        min_i, min_j : int
            Lower x/y boundary of the window to update, in cells.
        max_i, max_j : int
            Upper x/y boundary of the window to update, in cells.

        """
        if not self._enabled:
            return

        # if not current due to reset, set current now after clearing
        if not self.is_current() and self._was_reset:
            self._was_reset = False
            self.set_current(True)

        if self._footprint_clearing_enabled:
            self.set_convex_polygon_cost(self._transformed_footprint, FREE_SPACE)

        if self._combination_method == CombinationMethod.Overwrite:
            self.update_with_overwrite(master_grid, min_i, min_j, max_i, max_j)
        elif self._combination_method == CombinationMethod.Max:
            self.update_with_max(master_grid, min_i, min_j, max_i, max_j)
        elif self._combination_method == CombinationMethod.MaxWithoutUnknownOverwrite:
            self.update_with_max_without_unknown_overwrite(
                master_grid, min_i, min_j, max_i, max_j)

    def reset(self) -> None:
        """Reset this costmap layer, clearing its accumulated costmap and observations."""
        self.reset_maps()
        with self._obs_lock:
            for obs in self._observations:
                obs.origin = None
                obs.points = []
        self.set_current(False)
        self._was_reset = True

    def is_clearable(self) -> bool:
        """
        Return whether clearing operations should be processed on this layer.

        Returns
        -------
        bool
            Always ``True``; obstacle layers are clearable.

        """
        return True

    # ------------------------------------------------------------------
    # Raytrace clearing
    # ------------------------------------------------------------------

    def raytrace_freespace(
        self,
        clearing_observation: Observation,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """
        Clear free space along the rays from a clearing observation.

        Parameters
        ----------
        clearing_observation : Observation
            The clearing observation (origin + return points).
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        if clearing_observation.origin is None:
            return
        ox, oy = clearing_observation.origin

        ok, x0, y0 = self.world_to_map(ox, oy)
        if not ok:
            self._node.get_logger().warning(
                f'[ObstacleLayer] "{self._name}": sensor origin '
                f'({ox:.2f}, {oy:.2f}) is out of map bounds, cannot raytrace.',
                throttle_duration_sec=2.0)
            return

        origin_x = self.origin_x
        origin_y = self.origin_y
        map_end_x = origin_x + self.size_x * self.resolution
        map_end_y = origin_y + self.size_y * self.resolution

        self.touch(ox, oy, min_x, min_y, max_x, max_y)

        cell_raytrace_max_range = self.cell_distance(clearing_observation.raytrace_max_range)
        cell_raytrace_min_range = self.cell_distance(clearing_observation.raytrace_min_range)

        for (wx, wy) in clearing_observation.points:
            # Make sure the endpoint we're raytracing to isn't off the costmap
            # and scale it back onto the border if necessary.
            a = wx - ox
            b = wy - oy

            if wx < origin_x:
                t = (origin_x - ox) / a
                wx = origin_x
                wy = oy + b * t
            if wy < origin_y:
                t = (origin_y - oy) / b
                wx = ox + a * t
                wy = origin_y
            if wx > map_end_x:
                t = (map_end_x - ox) / a
                wx = map_end_x - 0.001
                wy = oy + b * t
            if wy > map_end_y:
                t = (map_end_y - oy) / b
                wx = ox + a * t
                wy = map_end_y - 0.001

            ok, x1, y1 = self.world_to_map(wx, wy)
            if not ok:
                continue

            self.raytrace_line(
                FREE_SPACE, x0, y0, x1, y1,
                cell_raytrace_max_range, cell_raytrace_min_range)

            self.update_raytrace_bounds(
                ox, oy, wx, wy,
                clearing_observation.raytrace_max_range,
                clearing_observation.raytrace_min_range,
                min_x, min_y, max_x, max_y)

    def update_raytrace_bounds(
        self,
        ox: float, oy: float,
        wx: float, wy: float,
        max_range: float, min_range: float,
        min_x: List[float], min_y: List[float],
        max_x: List[float], max_y: List[float],
    ) -> None:
        """
        Expand the bounds to cover a cleared ray.

        Parameters
        ----------
        ox, oy : float
            Sensor origin, in world coordinates.
        wx, wy : float
            Ray endpoint, in world coordinates.
        max_range, min_range : float
            Raytrace max/min ranges, in metres.
        min_x, min_y, max_x, max_y : list of float
            Single-element lists holding the update window bounds, expanded in
            place.

        """
        dx = wx - ox
        dy = wy - oy
        full_distance = math.hypot(dx, dy)
        if full_distance < min_range:
            return
        scale = 1.0 if full_distance == 0.0 else min(1.0, max_range / full_distance)
        ex = ox + dx * scale
        ey = oy + dy * scale
        self.touch(ex, ey, min_x, min_y, max_x, max_y)

    # ------------------------------------------------------------------
    # LaserScan callback
    # ------------------------------------------------------------------

    def _snapshot(self, obs: Observation) -> Observation:
        """
        Return a shallow copy of an observation's volatile data (origin/points).

        Parameters
        ----------
        obs : Observation
            The observation to snapshot.

        """
        snap = Observation()
        snap.origin = obs.origin
        snap.points = list(obs.points)
        snap.marking = obs.marking
        snap.clearing = obs.clearing
        snap.obstacle_max_range = obs.obstacle_max_range
        snap.obstacle_min_range = obs.obstacle_min_range
        snap.raytrace_max_range = obs.raytrace_max_range
        snap.raytrace_min_range = obs.raytrace_min_range
        return snap

    def _laser_scan_callback(self, msg: LaserScan, obs: Observation) -> None:
        """
        Buffer a LaserScan: store the sensor origin and the return points.

        Transforms the scan into the costmap global frame using the full sensor
        TF (translation + 3D rotation). The stored origin/points are consumed by
        ``update_bounds`` for raytrace clearing and marking.

        Parameters
        ----------
        msg : LaserScan
            The incoming laser scan.
        obs : Observation
            The observation record for this source (``origin``/``points`` are
            replaced in place).

        """
        ray_min_range = obs.raytrace_min_range
        ray_max_range = obs.raytrace_max_range
        if self._tf_buffer is None:
            return
        target_frame = (
            self._layered_costmap.get_global_frame_id()
            if self._layered_costmap is not None
            else ''
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=0.1),
            )
        except Exception:  # noqa: B902
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame, msg.header.frame_id, Time(),
                    timeout=Duration(seconds=0.1))
            except Exception as ex:  # noqa: BLE001
                self._node.get_logger().warning(
                    f'[ObstacleLayer] "{self._name}": cannot transform scan from '
                    f"'{msg.header.frame_id}' to '{target_frame}': {ex}",
                    throttle_duration_sec=2.0)
                return

        ox = transform.transform.translation.x
        oy = transform.transform.translation.y
        q = transform.transform.rotation
        xx, yy, zz = q.x * q.x, q.y * q.y, q.z * q.z
        xy = q.x * q.y
        wz_ = q.w * q.z
        r00, r01 = 1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz_)
        r10, r11 = 2.0 * (xy + wz_), 1.0 - 2.0 * (xx + zz)

        new_pts = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < ray_min_range or r > ray_max_range:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            wx = ox + r00 * px + r01 * py
            wy = oy + r10 * px + r11 * py
            new_pts.append((wx, wy))

        with self._obs_lock:
            obs.origin = (ox, oy)
            obs.points = new_pts

        self.set_current(True)
