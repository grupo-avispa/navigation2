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
RangeSensorLayer for nav2_costmap_2d_py.

Takes in IR/Sonar/similar point measurement sensors and populates a costmap
using a probabilistic sensor model.

It mirrors the nav2_costmap_2d::RangeSensorLayer from the C++ implementation.

Plugin type string:
    ``"nav2_costmap_2d_py/RangeSensorLayer"``
"""

import enum
import math
import threading
from typing import Any, Callable, List, Optional, Tuple

from nav2_costmap_2d_py.core.cost_values import FREE_SPACE, LETHAL_OBSTACLE, NO_INFORMATION
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Range

_INF = float('inf')


class InputSensorType(enum.Enum):
    """The type of range sensor input."""

    VARIABLE = 0
    FIXED = 1
    ALL = 2


def _normalize_angle(a: float) -> float:
    """Normalize an angle to ``[-pi, pi]``."""
    return math.atan2(math.sin(a), math.cos(a))


class RangeSensorLayer(CostmapLayer):
    """Takes in IR/Sonar/similar point sensors and populates the costmap."""

    def __init__(self) -> None:
        """Initialize range sensor layer defaults."""
        super().__init__()
        self._max_angle = 0.0
        self._phi_v = 1.2
        self._inflate_cone = 1.0
        self._global_frame = ''
        self._clear_threshold = 0.2
        self._mark_threshold = 0.8
        self._clear_on_max_reading = False
        self._was_reset = False
        self._transform_tolerance = 0.0
        self._no_readings_timeout = 0.0
        self._last_reading_time: Any = None
        self._buffered_readings = 0
        self._range_subs: List[Any] = []
        self._min_x = _INF
        self._min_y = _INF
        self._max_x = -_INF
        self._max_y = -_INF
        self._range_message_mutex = threading.Lock()
        self._range_msgs_buffer: List[Range] = []
        self._process_range_message_func: Optional[Callable[[
            Range], None]] = None

    # ------------------------------------------------------------------
    # Probability helpers
    # ------------------------------------------------------------------

    def to_prob(self, c: int) -> float:
        """Find the probability value of a cost."""
        return float(c) / LETHAL_OBSTACLE

    def to_cost(self, p: float) -> int:
        """Find the cost value of a probability."""
        return int(p * LETHAL_OBSTACLE)

    # ------------------------------------------------------------------
    # Layer interface
    # ------------------------------------------------------------------

    def on_initialize(self) -> None:
        """Initialize the layer: read parameters and subscribe to range topics."""
        node = self._node
        self.set_current(True)
        self._was_reset = False
        self._buffered_readings = 0
        self._last_reading_time = node.get_clock().now()
        self._default_value = self.to_cost(0.5)

        self.match_size()
        self.reset_range()

        self._enabled = self._declare_parameter_if_not_declared(
            'enabled', True)
        self._phi_v = self._declare_parameter_if_not_declared('phi', 1.2)
        self._inflate_cone = self._declare_parameter_if_not_declared(
            'inflate_cone', 1.0)
        self._no_readings_timeout = self._declare_parameter_if_not_declared(
            'no_readings_timeout', 0.0)
        self._clear_threshold = self._declare_parameter_if_not_declared(
            'clear_threshold', 0.2)
        self._mark_threshold = self._declare_parameter_if_not_declared(
            'mark_threshold', 0.8)
        self._clear_on_max_reading = self._declare_parameter_if_not_declared(
            'clear_on_max_reading', False)
        if node.has_parameter('transform_tolerance'):
            self._transform_tolerance = node.get_parameter(
                'transform_tolerance').value

        topic_names = self._declare_parameter_if_not_declared('topics', [])
        sensor_type_name = str(self._declare_parameter_if_not_declared(
            'input_sensor_type', 'ALL')).upper()

        if sensor_type_name == 'VARIABLE':
            input_sensor_type = InputSensorType.VARIABLE
            self._process_range_message_func = self.process_variable_range_msg
        elif sensor_type_name == 'FIXED':
            input_sensor_type = InputSensorType.FIXED
            self._process_range_message_func = self.process_fixed_range_msg
        else:
            input_sensor_type = InputSensorType.ALL
            self._process_range_message_func = self.process_range_msg
        del input_sensor_type

        if not topic_names:
            node.get_logger().fatal(
                'Invalid topic names list: it must be a non-empty list of strings')
            return

        for topic_name in topic_names:
            topic = self.join_with_parent_namespace(topic_name)
            sub = node.create_subscription(
                Range, topic, self.buffer_incoming_range_msg, 10)
            self._range_subs.append(sub)
            node.get_logger().info(
                f'RangeSensorLayer: subscribed to topic {topic}')

        self._global_frame = self._layered_costmap.get_global_frame_id()

    # ------------------------------------------------------------------
    # Sensor model
    # ------------------------------------------------------------------

    def gamma(self, theta: float) -> float:
        """Get the gamma value for an angle theta."""
        if abs(theta) > self._max_angle:
            return 0.0
        return 1 - (theta / self._max_angle) ** 2

    def delta(self, phi: float) -> float:
        """Get the delta value for an angle phi."""
        return 1 - (1 + math.tanh(2 * (phi - self._phi_v))) / 2

    def get_deltas(self, angle: float) -> Tuple[float, float]:
        """Get the (dx, dy) step deltas for a cone angle."""
        ta = math.tan(angle)
        dx = 0.0 if ta == 0 else self._resolution / ta
        dx = math.copysign(dx, math.cos(angle))
        dy = math.copysign(self._resolution, math.sin(angle))
        return dx, dy

    def sensor_model(self, r: float, phi: float, theta: float) -> float:
        """Apply the sensor model of the layer for range sensors."""
        lbda = self.delta(phi) * self.gamma(theta)
        delta = self._resolution

        if 0.0 <= phi < r - 2 * delta * r:
            return (1 - lbda) * 0.5
        elif phi < r - delta * r:
            return (lbda * 0.5 * ((phi - (r - 2 * delta * r)) / (delta * r)) ** 2
                    + (1 - lbda) * 0.5)
        elif phi < r + delta * r:
            j = (r - phi) / (delta * r)
            return lbda * ((1 - 0.5 * j ** 2) - 0.5) + 0.5
        else:
            return 0.5

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def area(x1: int, y1: int, x2: int, y2: int, x3: int, y3: int) -> float:
        """Find the area of a triangle defined by three points."""
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0)

    @staticmethod
    def orient2d(ax: int, ay: int, bx: int, by: int, cx: int, cy: int) -> int:
        """Find the cross product (orientation) of three points A, B, C."""
        return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)

    # ------------------------------------------------------------------
    # Range message handling
    # ------------------------------------------------------------------

    def buffer_incoming_range_msg(self, range_message: Range) -> None:
        """Handle an incoming Range message by buffering it."""
        with self._range_message_mutex:
            self._range_msgs_buffer.append(range_message)

    def update_costmap(
        self,
        range_message: Optional[Range] = None,
        clear_sensor_cone: bool = False,
    ) -> None:
        """
        Process buffered sensors, or a single range message into the costmap.

        When ``range_message`` is None, drains the buffer and dispatches each
        message; otherwise processes the single message into the costmap.

        Parameters
        ----------
        range_message : sensor_msgs.msg.Range, optional
            The range message to process, or None to drain the buffer.
        clear_sensor_cone : bool
            Whether to clear the sensor cone instead of marking it.

        """
        if range_message is None:
            with self._range_message_mutex:
                buffer_copy = list(self._range_msgs_buffer)
                self._range_msgs_buffer.clear()
            assert self._process_range_message_func is not None
            for msg in buffer_copy:
                self._process_range_message_func(msg)
            return
        self._update_costmap_with_range(range_message, clear_sensor_cone)

    def process_range_msg(self, range_message: Range) -> None:
        """Process a generic range message (fixed if min==max, else variable)."""
        if range_message.min_range == range_message.max_range:
            self.process_fixed_range_msg(range_message)
        else:
            self.process_variable_range_msg(range_message)

    def process_fixed_range_msg(self, range_message: Range) -> None:
        """Process a fixed-range (min==max) incoming range message."""
        if not math.isinf(range_message.range):
            self._node.get_logger().error(
                'Fixed distance ranger sent invalid value. Only -Inf and Inf '
                'are valid.')
            return

        clear_sensor_cone = False
        if range_message.range > 0:  # +inf
            if not self._clear_on_max_reading:
                return
            clear_sensor_cone = True

        range_message.range = range_message.min_range
        self.update_costmap(range_message, clear_sensor_cone)

    def process_variable_range_msg(self, range_message: Range) -> None:
        """Process a variable-range incoming range message."""
        if (range_message.range < range_message.min_range
                or range_message.range > range_message.max_range):
            return

        clear_sensor_cone = False
        if range_message.range >= range_message.max_range and self._clear_on_max_reading:
            clear_sensor_cone = True

        self.update_costmap(range_message, clear_sensor_cone)

    def _update_costmap_with_range(
        self, range_message: Range, clear_sensor_cone: bool
    ) -> None:
        """Update the costmap with a single range message (the sensor cone)."""
        self._max_angle = range_message.field_of_view / 2

        origin = self._transform_point(
            range_message.header.frame_id, range_message.header.stamp, 0.0)
        if origin is None:
            self._node.get_logger().info(
                f"Range sensor layer can't transform from {self._global_frame} "
                f'to {range_message.header.frame_id}')
            return
        ox, oy = origin
        target = self._transform_point(
            range_message.header.frame_id, range_message.header.stamp,
            range_message.range)
        if target is None:
            return
        tx, ty = target

        dx = tx - ox
        dy = ty - oy
        theta = math.atan2(dy, dx)
        d = math.hypot(dx, dy)

        ox_cell, oy_cell = self.world_to_map_no_bounds(ox, oy)
        bx0 = bx1 = ox_cell
        by0 = by1 = oy_cell
        self._touch_point(ox, oy)

        ok, aa, ab = self.world_to_map(tx, ty)
        if ok:
            self.set_cost(aa, ab, 233)
            self._touch_point(tx, ty)

        mx = ox + math.cos(theta - self._max_angle) * d * 1.2
        my = oy + math.sin(theta - self._max_angle) * d * 1.2
        ax_cell, ay_cell = self.world_to_map_no_bounds(mx, my)
        bx0, bx1 = min(bx0, ax_cell), max(bx1, ax_cell)
        by0, by1 = min(by0, ay_cell), max(by1, ay_cell)
        self._touch_point(mx, my)

        mx = ox + math.cos(theta + self._max_angle) * d * 1.2
        my = oy + math.sin(theta + self._max_angle) * d * 1.2
        bx_cell, by_cell = self.world_to_map_no_bounds(mx, my)
        bx0, bx1 = min(bx0, bx_cell), max(bx1, bx_cell)
        by0, by1 = min(by0, by_cell), max(by1, by_cell)
        self._touch_point(mx, my)

        bx0 = max(0, bx0)
        by0 = max(0, by0)
        bx1 = min(self._size_x, bx1)
        by1 = min(self._size_y, by1)

        for x in range(bx0, bx1 + 1):
            for y in range(by0, by1 + 1):
                update_xy_cell = True
                if self._inflate_cone < 1.0:
                    w0 = self.orient2d(
                        ax_cell, ay_cell, bx_cell, by_cell, x, y)
                    w1 = self.orient2d(
                        bx_cell, by_cell, ox_cell, oy_cell, x, y)
                    w2 = self.orient2d(
                        ox_cell, oy_cell, ax_cell, ay_cell, x, y)
                    bcciath = -float(self._inflate_cone) * self.area(
                        ax_cell, ay_cell, bx_cell, by_cell, ox_cell, oy_cell)
                    update_xy_cell = w0 >= bcciath and w1 >= bcciath and w2 >= bcciath

                if update_xy_cell:
                    wx, wy = self.map_to_world(x, y)
                    self.update_cell(ox, oy, theta, range_message.range, wx, wy,
                                     clear_sensor_cone)

        self._buffered_readings += 1
        self._last_reading_time = self._node.get_clock().now()

    def update_cell(
        self, ox: float, oy: float, ot: float, r: float,
        nx: float, ny: float, clear: bool,
    ) -> None:
        """Update the cost in a cell with the probabilistic sensor information."""
        ok, x, y = self.world_to_map(nx, ny)
        if ok:
            dx = nx - ox
            dy = ny - oy
            theta = _normalize_angle(math.atan2(dy, dx) - ot)
            phi = math.hypot(dx, dy)
            sensor = 0.0
            if not clear:
                sensor = self.sensor_model(r, phi, theta)
            prior = self.to_prob(self.get_cost(x, y))
            prob_occ = sensor * prior
            prob_not = (1 - sensor) * (1 - prior)
            new_prob = prob_occ / (prob_occ + prob_not)
            self.set_cost(x, y, self.to_cost(new_prob))

    def reset_range(self) -> None:
        """Reset the cached min/max x and y values."""
        self._min_x = _INF
        self._min_y = _INF
        self._max_x = -_INF
        self._max_y = -_INF

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
        if self._layered_costmap.is_rolling():
            self.update_origin(
                robot_x - self.size_x_meters / 2,
                robot_y - self.size_y_meters / 2)

        self.update_costmap()

        min_x[0] = min(min_x[0], self._min_x)
        min_y[0] = min(min_y[0], self._min_y)
        max_x[0] = max(max_x[0], self._max_x)
        max_y[0] = max(max_y[0], self._max_y)

        self.reset_range()

        if not self._enabled:
            self.set_current(True)
            return

        if self._buffered_readings == 0:
            if self._no_readings_timeout > 0.0:
                elapsed = (self._node.get_clock().now()
                           - self._last_reading_time).nanoseconds / 1e9
                if elapsed > self._no_readings_timeout:
                    self._node.get_logger().warning(
                        f'No range readings received for {elapsed:.2f} seconds.')
                    self.set_current(False)

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

        master_array = master_grid.get_char_map()
        span = master_grid.size_x
        clear = self.to_cost(self._clear_threshold)
        mark = self.to_cost(self._mark_threshold)

        for j in range(min_j, max_j):
            it = j * span + min_i
            for _ in range(min_i, max_i):
                prob = self._costmap[it]
                if prob == NO_INFORMATION:
                    it += 1
                    continue
                elif prob > mark:
                    current = LETHAL_OBSTACLE
                elif prob < clear:
                    current = FREE_SPACE
                else:
                    it += 1
                    continue

                old_cost = master_array[it]
                if old_cost == NO_INFORMATION or old_cost < current:
                    master_array[it] = current
                it += 1

        self._buffered_readings = 0

        if not self.is_current() and self._was_reset:
            self._was_reset = False
            self.set_current(True)

    def reset(self) -> None:
        """Reset this costmap layer."""
        self.deactivate()
        self.reset_maps()
        self._was_reset = True
        self.activate()

    def deactivate(self) -> None:
        """Deactivate the layer, clearing the message buffer."""
        self._range_msgs_buffer.clear()

    def activate(self) -> None:
        """Activate the layer, clearing the message buffer."""
        self._range_msgs_buffer.clear()

    def is_clearable(self) -> bool:
        """Return whether clearing operations should be processed on this layer."""
        return True

    # ------------------------------------------------------------------
    # Bounds-tracking helpers (Python substitute for the C++ double& touch)
    # ------------------------------------------------------------------

    def _touch_point(self, wx: float, wy: float) -> None:
        """Expand the cached min/max bounds to include the world point."""
        self._min_x = min(self._min_x, wx)
        self._min_y = min(self._min_y, wy)
        self._max_x = max(self._max_x, wx)
        self._max_y = max(self._max_y, wy)

    def _transform_point(
        self, frame_id: str, stamp: Any, x: float
    ) -> Optional[Tuple[float, float]]:
        """Transform a point ``(x, 0, 0)`` from ``frame_id`` to the global frame."""
        if self._tf_buffer is None:
            return None
        from geometry_msgs.msg import PointStamped
        import tf2_geometry_msgs  # noqa: F401
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = stamp
        point.point.x = float(x)
        try:
            out = self._tf_buffer.transform(
                point, self._global_frame,
                timeout=Duration(seconds=self._transform_tolerance))
        except Exception:  # noqa: BLE001
            try:
                point.header.stamp = Time().to_msg()
                out = self._tf_buffer.transform(point, self._global_frame)
            except Exception:  # noqa: BLE001
                return None
        return out.point.x, out.point.y
