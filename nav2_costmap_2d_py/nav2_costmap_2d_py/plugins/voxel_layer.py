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
VoxelLayer for nav2_costmap_2d_py.

Takes laser and pointcloud data to populate a 3D voxel representation of the
environment, projecting marked voxels into the 2D costmap.

It mirrors the nav2_costmap_2d::VoxelLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/VoxelLayer"``
"""

import math
from typing import Any, List, Tuple

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.observation import Observation
from nav2_costmap_2d_py.core.voxel_grid import VoxelGrid
from nav2_costmap_2d_py.plugins.obstacle_layer import ObstacleLayer, _read_xyz

VOXEL_BITS = 16


class VoxelLayer(ObstacleLayer):
    """Takes laser/pointcloud data to populate a 3D voxel grid and the 2D costmap."""

    def __init__(self) -> None:
        """Initialize voxel layer defaults."""
        super().__init__()
        self._publish_voxel = False
        self._voxel_grid = VoxelGrid(0, 0, 0)
        self._z_resolution = 0.2
        self._origin_z = 0.0
        self._unknown_threshold = 15
        self._mark_threshold = 0
        self._size_z = 10
        self._voxel_pub: Any = None

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer: read obstacle + voxel parameters."""
        super().on_initialize()

        self._enabled = self._declare_parameter_if_not_declared('enabled', True)
        self._footprint_clearing_enabled = self._declare_parameter_if_not_declared(
            'footprint_clearing_enabled', True)
        self._min_obstacle_height = self._declare_parameter_if_not_declared(
            'min_obstacle_height', 0.0)
        self._max_obstacle_height = self._declare_parameter_if_not_declared(
            'max_obstacle_height', 2.0)
        self._size_z = self._declare_parameter_if_not_declared('z_voxels', 10)
        self._origin_z = self._declare_parameter_if_not_declared('origin_z', 0.0)
        self._z_resolution = self._declare_parameter_if_not_declared('z_resolution', 0.2)
        self._unknown_threshold = self._declare_parameter_if_not_declared(
            'unknown_threshold', 15)
        self._mark_threshold = self._declare_parameter_if_not_declared('mark_threshold', 0)
        self._publish_voxel = self._declare_parameter_if_not_declared(
            'publish_voxel_map', False)

        if self._publish_voxel:
            try:
                from nav2_msgs.msg import VoxelGrid as VoxelGridMsg
                self._voxel_pub = self._node.create_publisher(
                    VoxelGridMsg, 'voxel_grid', 1)
            except ImportError:
                self._publish_voxel = False

        self._unknown_threshold += (VOXEL_BITS - self._size_z)
        self.match_size()

    def match_size(self) -> None:
        """Match the size of the master costmap and resize the voxel grid."""
        super().match_size()
        self._voxel_grid.resize(self.size_x, self.size_y, self._size_z)

    def reset(self) -> None:
        """Reset this costmap layer and the voxel grid."""
        super().reset()
        self.reset_maps()

    def reset_maps(self) -> None:
        """Reset the 2D costmap and the voxel grid."""
        super().reset_maps()
        self._voxel_grid.reset()

    def is_clearable(self) -> bool:
        """Return whether clearing operations should be processed on this layer."""
        return True

    # ------------------------------------------------------------------
    # 3D coordinate helpers
    # ------------------------------------------------------------------

    def world_to_map_3d_float(
        self, wx: float, wy: float, wz: float
    ) -> Tuple[bool, float, float, float]:
        """Convert world coordinates into continuous 3D map coordinates."""
        if wx < self.origin_x or wy < self.origin_y or wz < self._origin_z:
            return False, 0.0, 0.0, 0.0
        mx = (wx - self.origin_x) / self.resolution
        my = (wy - self.origin_y) / self.resolution
        mz = (wz - self._origin_z) / self._z_resolution
        if mx < self.size_x and my < self.size_y and mz < self._size_z:
            return True, mx, my, mz
        return False, 0.0, 0.0, 0.0

    def world_to_map_3d(
        self, wx: float, wy: float, wz: float
    ) -> Tuple[bool, int, int, int]:
        """Convert world coordinates into integer 3D map coordinates."""
        if wx < self.origin_x or wy < self.origin_y or wz < self._origin_z:
            return False, 0, 0, 0
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        mz = int((wz - self._origin_z) / self._z_resolution)
        if mx < self.size_x and my < self.size_y and mz < self._size_z:
            return True, mx, my, mz
        return False, 0, 0, 0

    def map_to_world_3d(
        self, mx: int, my: int, mz: int
    ) -> Tuple[float, float, float]:
        """Convert integer 3D map coordinates into world coordinates (cell centre)."""
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        wz = self._origin_z + (mz + 0.5) * self._z_resolution
        return wx, wy, wz

    @staticmethod
    def dist(
        x0: float, y0: float, z0: float, x1: float, y1: float, z1: float
    ) -> float:
        """Return the L2 norm distance in 3D."""
        return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)

    def get_size_in_meters_z(self) -> float:
        """Return the height of the voxel grid, in meters."""
        return self._size_z * self._z_resolution

    def is_discretized(self) -> bool:
        """Return whether the layer is discretized."""
        return True

    # ------------------------------------------------------------------
    # Update
    # ------------------------------------------------------------------

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
        """Update the bounds, populating the voxel grid and projecting obstacles."""
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
            sq_obstacle_max_range = obs.obstacle_max_range ** 2
            sq_obstacle_min_range = obs.obstacle_min_range ** 2
            for px, py, pz in _read_xyz(obs.cloud):
                if pz < self._min_obstacle_height or pz > self._max_obstacle_height:
                    continue
                sq_dist = ((px - obs.origin.x) ** 2 + (py - obs.origin.y) ** 2
                           + (pz - obs.origin.z) ** 2)
                if sq_dist >= sq_obstacle_max_range or sq_dist < sq_obstacle_min_range:
                    continue

                if pz < self._origin_z:
                    ok, mx, my, mz = self.world_to_map_3d(px, py, self._origin_z)
                else:
                    ok, mx, my, mz = self.world_to_map_3d(px, py, pz)
                if not ok:
                    continue

                if self._voxel_grid.mark_voxel_in_map(mx, my, mz, self._mark_threshold):
                    index = self.get_index(mx, my)
                    self._costmap[index] = LETHAL_OBSTACLE
                    self.touch(px, py, min_x, min_y, max_x, max_y)

        if self._publish_voxel:
            self._publish_voxel_grid()

        self.update_footprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y)

    def raytrace_freespace(
        self,
        clearing_observation: Observation,
        min_x: List[float],
        min_y: List[float],
        max_x: List[float],
        max_y: List[float],
    ) -> None:
        """Use 3D raycasting to clear free voxels between the sensor and each hit."""
        ox = clearing_observation.origin.x
        oy = clearing_observation.origin.y
        oz = clearing_observation.origin.z

        ok, sensor_x, sensor_y, sensor_z = self.world_to_map_3d_float(ox, oy, oz)
        if not ok:
            self._node.get_logger().warning(
                f'[VoxelLayer] Sensor origin at ({ox:.2f}, {oy:.2f}, {oz:.2f}) '
                'is out of map bounds. Cannot raytrace.', throttle_duration_sec=2.0)
            return

        origin_x = self.origin_x
        origin_y = self.origin_y
        map_end_x = origin_x + self.size_x_meters
        map_end_y = origin_y + self.size_y_meters
        map_end_z = self._origin_z + self.get_size_in_meters_z()

        for wpx, wpy, wpz in _read_xyz(clearing_observation.cloud):
            distance = self.dist(ox, oy, oz, wpx, wpy, wpz)
            scaling_fact = 1.0
            if distance > 0.0:
                scaling_fact = max(min(scaling_fact,
                                       (distance - 2 * self.resolution) / distance), 0.0)
            wpx = scaling_fact * (wpx - ox) + ox
            wpy = scaling_fact * (wpy - oy) + oy
            wpz = scaling_fact * (wpz - oz) + oz

            a = wpx - ox
            b = wpy - oy
            c = wpz - oz
            t = 1.0
            wp_outside = False

            if wpz > map_end_z:
                t = max(0.0, min(t, (map_end_z - 0.01 - oz) / c))
                wp_outside = True
            elif wpz < self._origin_z:
                t = min(t, (self._origin_z - oz) / c)
                wp_outside = True
            if wpx < origin_x:
                t = min(t, (origin_x - ox) / a)
                wp_outside = True
            if wpy < origin_y:
                t = min(t, (origin_y - oy) / b)
                wp_outside = True
            if wpx > map_end_x:
                t = min(t, (map_end_x - ox) / a)
                wp_outside = True
            if wpy > map_end_y:
                t = min(t, (map_end_y - oy) / b)
                wp_outside = True

            if wp_outside:
                if t > 0.0:
                    t -= 1e-5
                elif t < 0.0:
                    t += 1e-5

            wpx = ox + a * t
            wpy = oy + b * t
            wpz = oz + c * t

            ok, point_x, point_y, point_z = self.world_to_map_3d_float(wpx, wpy, wpz)
            if not ok:
                continue

            cell_raytrace_max_range = self.cell_distance(
                clearing_observation.raytrace_max_range)
            cell_raytrace_min_range = self.cell_distance(
                clearing_observation.raytrace_min_range)

            self._voxel_grid.clear_voxel_line_in_map(
                sensor_x, sensor_y, sensor_z, point_x, point_y, point_z,
                self._costmap, self._unknown_threshold, self._mark_threshold,
                FREE_SPACE, NO_INFORMATION,
                cell_raytrace_max_range, cell_raytrace_min_range)

            self.update_raytrace_bounds(
                ox, oy, wpx, wpy, clearing_observation.raytrace_max_range,
                clearing_observation.raytrace_min_range, min_x, min_y, max_x, max_y)

    def update_origin(self, new_origin_x: float, new_origin_y: float) -> None:
        """Move the origin, shifting both the 2D costmap and the voxel grid."""
        cell_ox = int((new_origin_x - self.origin_x) / self.resolution)
        cell_oy = int((new_origin_y - self.origin_y) / self.resolution)
        if cell_ox == 0 and cell_oy == 0:
            return

        new_grid_ox = self.origin_x + cell_ox * self.resolution
        new_grid_oy = self.origin_y + cell_oy * self.resolution

        size_x = self.size_x
        size_y = self.size_y
        lower_left_x = min(max(cell_ox, 0), size_x)
        lower_left_y = min(max(cell_oy, 0), size_y)
        upper_right_x = min(max(cell_ox + size_x, 0), size_x)
        upper_right_y = min(max(cell_oy + size_y, 0), size_y)
        cell_size_x = upper_right_x - lower_left_x
        cell_size_y = upper_right_y - lower_left_y

        # Snapshot the overlapping 2D and voxel windows.
        local_map = bytearray(cell_size_x * cell_size_y)
        for row in range(cell_size_y):
            s = (lower_left_y + row) * size_x + lower_left_x
            d = row * cell_size_x
            local_map[d:d + cell_size_x] = self._costmap[s:s + cell_size_x]
        voxel = self._voxel_grid.get_data()
        local_voxel = voxel[
            lower_left_y:lower_left_y + cell_size_y,
            lower_left_x:lower_left_x + cell_size_x, :].copy()

        self.reset_maps()
        self._origin_x = new_grid_ox
        self._origin_y = new_grid_oy

        start_x = lower_left_x - cell_ox
        start_y = lower_left_y - cell_oy
        for row in range(cell_size_y):
            s = row * cell_size_x
            d = (start_y + row) * size_x + start_x
            self._costmap[d:d + cell_size_x] = local_map[s:s + cell_size_x]
        voxel[start_y:start_y + cell_size_y, start_x:start_x + cell_size_x, :] = local_voxel

    def _publish_voxel_grid(self) -> None:
        """Publish the voxel grid (best-effort marked-bitmask per column)."""
        if self._voxel_pub is None:
            return
        from nav2_msgs.msg import VoxelGrid as VoxelGridMsg
        import numpy as np
        msg = VoxelGridMsg()
        msg.header.frame_id = self._global_frame
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.size_x = self.size_x
        msg.size_y = self.size_y
        msg.size_z = self._size_z
        msg.origin.x = self.origin_x
        msg.origin.y = self.origin_y
        msg.origin.z = self._origin_z
        msg.resolutions.x = self.resolution
        msg.resolutions.y = self.resolution
        msg.resolutions.z = self._z_resolution
        data = self._voxel_grid.get_data()
        marked = (data == 1)
        bits = (marked * (1 << np.arange(data.shape[2]))).sum(axis=2)
        msg.data = bits.astype(np.uint32).ravel().tolist()
        self._voxel_pub.publish(msg)
