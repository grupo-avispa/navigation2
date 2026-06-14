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

Takes in laser and pointcloud data to populate the 2D costmap. Mirrors the
nav2_costmap_2d::ObstacleLayer pipeline: sensor messages are buffered into
``ObservationBuffer`` instances (as ``PointCloud2`` clouds transformed to the
global frame), then marking/clearing observations are consumed during
``update_bounds``.

It mirrors the nav2_costmap_2d::ObstacleLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/ObstacleLayer"``
"""

import math
from typing import Any, Iterator, List, Tuple

from nav2_costmap_2d_py.core.cost_values import CombinationMethod, FREE_SPACE, LETHAL_OBSTACLE
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_costmap_2d_py.core.footprint import transform_footprint
from nav2_costmap_2d_py.core.observation import Observation
from nav2_costmap_2d_py.core.observation_buffer import ObservationBuffer
from sensor_msgs.msg import LaserScan, PointCloud2


def _read_xyz(cloud: PointCloud2) -> Iterator[Tuple[float, float, float]]:
    """Yield the ``(x, y, z)`` points of a PointCloud2."""
    from sensor_msgs_py import point_cloud2
    for p in point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z'), skip_nans=True):
        yield float(p[0]), float(p[1]), float(p[2])


class ObstacleLayer(CostmapLayer):
    """Takes in laser and pointcloud data to populate the 2D costmap."""

    def __init__(self) -> None:
        """Initialize obstacle layer defaults."""
        super().__init__()
        self._combination_method = CombinationMethod.Max
        self._footprint_clearing_enabled = True
        self._min_obstacle_height = 0.0
        self._max_obstacle_height = 2.0
        self._global_frame = ''
        self._rolling_window = False
        self._was_reset = False

        self._observation_buffers: List[ObservationBuffer] = []
        self._marking_buffers: List[ObservationBuffer] = []
        self._clearing_buffers: List[ObservationBuffer] = []
        self._static_marking_observations: List[Observation] = []
        self._static_clearing_observations: List[Observation] = []

        self._observation_subscribers: List[Any] = []
        self._transformed_footprint: List[Tuple[float, float]] = []

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer: read parameters and subscribe to observation sources."""
        node = self._node
        self._rolling_window = self._layered_costmap.is_rolling()

        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._footprint_clearing_enabled = self._declare_parameter_if_not_declared(
            'footprint_clearing_enabled', True)
        self._min_obstacle_height = self._declare_parameter_if_not_declared(
            'min_obstacle_height', 0.0)
        self._max_obstacle_height = self._declare_parameter_if_not_declared(
            'max_obstacle_height', 2.0)
        combination_method = self._declare_parameter_if_not_declared('combination_method', 1)
        self._combination_method = self.combination_method_from_int(combination_method)
        topics_string = self._declare_parameter_if_not_declared('observation_sources', '')

        track_unknown_space = False
        if node.has_parameter('track_unknown_space'):
            track_unknown_space = node.get_parameter('track_unknown_space').value
        transform_tolerance = 0.1
        if node.has_parameter('transform_tolerance'):
            transform_tolerance = node.get_parameter('transform_tolerance').value

        from nav2_costmap_2d_py.core.cost_values import NO_INFORMATION
        self._default_value = NO_INFORMATION if track_unknown_space else FREE_SPACE

        self.match_size()
        self.set_current(True)
        self._was_reset = False
        self._global_frame = self._layered_costmap.get_global_frame_id()

        for source in topics_string.split():
            def _ps(param: str, default: Any, _src: str = source) -> Any:
                full = f'{self._name}.{_src}.{param}'
                if not node.has_parameter(full):
                    node.declare_parameter(full, default)
                return node.get_parameter(full).value

            topic = self.join_with_parent_namespace(_ps('topic', source))
            sensor_frame = _ps('sensor_frame', '')
            observation_keep_time = _ps('observation_persistence', 0.0)
            expected_update_rate = _ps('expected_update_rate', 0.0)
            data_type = _ps('data_type', 'LaserScan')
            min_obstacle_height = _ps('min_obstacle_height', 0.0)
            max_obstacle_height = _ps('max_obstacle_height', 0.0)
            inf_is_valid = _ps('inf_is_valid', False)
            marking = _ps('marking', True)
            clearing = _ps('clearing', False)
            obstacle_max_range = _ps('obstacle_max_range', 2.5)
            obstacle_min_range = _ps('obstacle_min_range', 0.0)
            raytrace_max_range = _ps('raytrace_max_range', 3.0)
            raytrace_min_range = _ps('raytrace_min_range', 0.0)

            if data_type not in ('PointCloud2', 'LaserScan'):
                raise RuntimeError(
                    'Only topics that use point cloud2s or laser scans are supported')

            buffer = ObservationBuffer(
                node, topic, observation_keep_time, expected_update_rate,
                min_obstacle_height, max_obstacle_height, obstacle_max_range,
                obstacle_min_range, raytrace_max_range, raytrace_min_range,
                self._tf_buffer, self._global_frame, sensor_frame, transform_tolerance)
            self._observation_buffers.append(buffer)
            if marking:
                self._marking_buffers.append(buffer)
            if clearing:
                self._clearing_buffers.append(buffer)

            if data_type == 'LaserScan':
                def _laser_cb(msg: LaserScan, b: ObservationBuffer = buffer,
                              valid_inf: bool = inf_is_valid) -> None:
                    if valid_inf:
                        self.laser_scan_valid_inf_callback(msg, b)
                    else:
                        self.laser_scan_callback(msg, b)
                sub = node.create_subscription(LaserScan, topic, _laser_cb, 50)
            else:
                def _pc_cb(msg: PointCloud2, b: ObservationBuffer = buffer) -> None:
                    self.point_cloud2_callback(msg, b)
                sub = node.create_subscription(PointCloud2, topic, _pc_cb, 50)
            self._observation_subscribers.append(sub)
            node.get_logger().info(
                f'[ObstacleLayer] "{self._name}" subscribed to "{topic}" '
                f'(mark={marking}, clear={clearing})')

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
        """Update the bounds of the master costmap by this layer's dimensions."""
        if self._rolling_window:
            self.update_origin(
                robot_x - self.size_x_meters / 2,
                robot_y - self.size_y_meters / 2)
        if not self._enabled:
            return
        self.use_extra_bounds(min_x, min_y, max_x, max_y)

        marking_current, observations = self.get_marking_observations()
        clearing_current, clearing_observations = self.get_clearing_observations()
        self.set_current(marking_current and clearing_current)

        for clearing_observation in clearing_observations:
            self.raytrace_freespace(clearing_observation, min_x, min_y, max_x, max_y)

        for obs in observations:
            max_range_cells = self.cell_distance(obs.obstacle_max_range)
            min_range_cells = self.cell_distance(obs.obstacle_min_range)

            ok, x0, y0 = self.world_to_map(obs.origin.x, obs.origin.y)
            if not ok:
                continue

            for px, py, pz in _read_xyz(obs.cloud):
                if pz < self._min_obstacle_height or pz > self._max_obstacle_height:
                    continue
                ok, mx, my = self.world_to_map(px, py)
                if not ok:
                    continue

                wdx = px - obs.origin.x
                wdy = py - obs.origin.y
                world_dist_sq = wdx * wdx + wdy * wdy
                if world_dist_sq > obs.obstacle_max_range * obs.obstacle_max_range:
                    continue
                if world_dist_sq < obs.obstacle_min_range * obs.obstacle_min_range:
                    continue

                dx = mx - x0
                dy = my - y0
                dist = int(math.hypot(dx, dy))
                if dist > max_range_cells or dist < min_range_cells:
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
        """Expand the bounds to include the robot footprint (for footprint clearing)."""
        if not self._footprint_clearing_enabled:
            return
        self._transformed_footprint = transform_footprint(
            robot_x, robot_y, robot_yaw, self.get_footprint())
        for fx, fy in self._transformed_footprint:
            self.touch(fx, fy, min_x, min_y, max_x, max_y)

    def update_costs(
        self,
        master_grid: Costmap2D,
        min_i: int,
        min_j: int,
        max_i: int,
        max_j: int,
    ) -> None:
        """Update the costs in the master costmap within the given window."""
        if not self._enabled:
            return

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

    # ------------------------------------------------------------------
    # Observations
    # ------------------------------------------------------------------

    def add_static_observation(
        self, obs: Observation, marking: bool, clearing: bool
    ) -> None:
        """Add a static observation (for testing) to the marking/clearing lists."""
        if marking:
            self._static_marking_observations.append(obs)
        if clearing:
            self._static_clearing_observations.append(obs)

    def clear_static_observations(self, marking: bool, clearing: bool) -> None:
        """Clear the static marking/clearing observations."""
        if marking:
            self._static_marking_observations = []
        if clearing:
            self._static_clearing_observations = []

    def get_marking_observations(self) -> Tuple[bool, List[Observation]]:
        """Return ``(current, observations)`` used to mark space."""
        current = True
        marking_observations: List[Observation] = []
        for marking_buffer in self._marking_buffers:
            marking_buffer.lock()
            marking_observations.extend(marking_buffer.get_observations())
            current = marking_buffer.is_current() and current
            marking_buffer.unlock()
        marking_observations.extend(self._static_marking_observations)
        return current, marking_observations

    def get_clearing_observations(self) -> Tuple[bool, List[Observation]]:
        """Return ``(current, observations)`` used to clear space."""
        current = True
        clearing_observations: List[Observation] = []
        for clearing_buffer in self._clearing_buffers:
            clearing_buffer.lock()
            clearing_observations.extend(clearing_buffer.get_observations())
            current = clearing_buffer.is_current() and current
            clearing_buffer.unlock()
        clearing_observations.extend(self._static_clearing_observations)
        return current, clearing_observations

    def raytrace_freespace(
        self,
        clearing_observation: Observation,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """Clear free space along the rays from a clearing observation."""
        ox = clearing_observation.origin.x
        oy = clearing_observation.origin.y

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

        cell_raytrace_max_range = self.cell_distance(
            clearing_observation.raytrace_max_range)
        cell_raytrace_min_range = self.cell_distance(
            clearing_observation.raytrace_min_range)

        for wx, wy, _ in _read_xyz(clearing_observation.cloud):
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
        """Expand the bounds to cover a cleared ray."""
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
    # Lifecycle
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset this costmap layer, clearing its grid and flagging a reset."""
        self.reset_maps()
        self.reset_buffers_last_updated()
        self.set_current(False)
        self._was_reset = True

    def reset_buffers_last_updated(self) -> None:
        """Trigger the update of the observation buffers' last-updated timestamp."""
        for observation_buffer in self._observation_buffers:
            observation_buffer.reset_last_updated()

    def activate(self) -> None:
        """Activate the layer."""
        self.reset_buffers_last_updated()

    def deactivate(self) -> None:
        """Deactivate the layer."""
        pass

    def is_clearable(self) -> bool:
        """Return whether clearing operations should be processed on this layer."""
        return True

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------

    def laser_scan_callback(
        self, message: LaserScan, buffer: ObservationBuffer
    ) -> None:
        """Project a LaserScan into a point cloud and buffer it."""
        cloud = self._project_laser(message)
        buffer.lock()
        buffer.buffer_cloud(cloud)
        buffer.unlock()

    def laser_scan_valid_inf_callback(
        self, raw_message: LaserScan, buffer: ObservationBuffer
    ) -> None:
        """Buffer a LaserScan, turning +Inf ranges into ``range_max``."""
        epsilon = 0.0001
        ranges = list(raw_message.ranges)
        for i, r in enumerate(ranges):
            if math.isinf(r) and r > 0:
                ranges[i] = raw_message.range_max - epsilon
        raw_message.ranges = ranges
        cloud = self._project_laser(raw_message)
        buffer.lock()
        buffer.buffer_cloud(cloud)
        buffer.unlock()

    def point_cloud2_callback(
        self, message: PointCloud2, buffer: ObservationBuffer
    ) -> None:
        """Buffer a PointCloud2 message."""
        buffer.lock()
        buffer.buffer_cloud(message)
        buffer.unlock()

    def _project_laser(self, scan: LaserScan) -> PointCloud2:
        """Project a LaserScan into a PointCloud2 in the scan frame."""
        from sensor_msgs_py import point_cloud2
        points = []
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            points.append((r * math.cos(angle), r * math.sin(angle), 0.0))
        return point_cloud2.create_cloud_xyz32(scan.header, points)
