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
Costmap2DROS for nav2_costmap_2d_py.

ROS2 lifecycle node wrapping a :class:`LayeredCostmap`.

It mirrors the nav2_costmap_2d::Costmap2DROS from the C++ implementation.

Plugin loading uses :class:`~nav2_costmap_2d_py.core.PluginProvider.PluginProvider`
with the entry_points group ``"nav2_costmap_2d_py_plugins"``.
"""

import math
import threading
import time
import traceback
from typing import Dict, List, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from geometry_msgs.msg import PolygonStamped, Polygon, Point32, PoseStamped

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.layered_costmap import (
    LayeredCostmap,
    make_footprint_from_radius,
    make_footprint_from_string,
    pad_footprint,
    transform_footprint,
)
from nav2_costmap_2d_py.core.costmap_2d_publisher import Costmap2DPublisher
from nav2_costmap_2d_py.core.clear_costmap_service import ClearCostmapService
from nav2_core_py.plugin_provider import PluginProvider

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401


class Costmap2DROS(LifecycleNode):
    """
    Python lifecycle node wrapping a LayeredCostmap.

    Drop-in replacement for the C++ ``Costmap2DROS`` node.

    YAML parameters (under ``<node_name>.ros__parameters``):
      global_frame                (str,   default 'map')
      robot_base_frame            (str,   default 'base_link')
      resolution                  (float, default 0.05)
      width                       (int,   default 5   [metres])
      height                      (int,   default 5   [metres])
      origin_x                    (float, default 0.0)
      origin_y                    (float, default 0.0)
      rolling_window              (bool,  default False)
      track_unknown_space         (bool,  default False)
      update_frequency            (float, default 5.0)
      publish_frequency           (float, default 1.0)
      transform_tolerance         (float, default 0.3)
      initial_transform_timeout   (float, default 60.0)
      always_send_full_costmap    (bool,  default False)
      map_vis_z                   (float, default 0.0)
      robot_radius                (float, default 0.1)
      footprint                   (str,   default '[]')
      footprint_padding           (float, default 0.01)
      plugins                     (list,  default ['static_layer',
                                                    'obstacle_layer',
                                                    'inflation_layer'])
      filters                     (list,  default [])
      subscribe_to_stamped_footprint (bool, default False)
    """

    # Default plugins
    DEFAULT_PLUGIN_NAMES = ['static_layer', 'obstacle_layer', 'inflation_layer']
    DEFAULT_PLUGIN_TYPES = [
        'nav2_costmap_2d_py/StaticLayer',
        'nav2_costmap_2d_py/ObstacleLayer',
        'nav2_costmap_2d_py/InflationLayer',
    ]

    def __init__(
        self,
        name: str = 'costmap',
        parent_namespace: str = '',
        use_sim_time: bool = False,
    ) -> None:
        # Must be set before super().__init__() because get_name() is overridden
        # and ROS2 may call it during node initialization.
        self._name = name

        # Build the costmap's fully-qualified namespace exactly like the C++
        # Costmap2DROS (nav2_util::add_namespaces(parent_namespace, name)):
        costmap_namespace = _add_namespaces(parent_namespace, name)

        # TF is published in the *parent* namespace ('/tf', or '/<robot_ns>/tf'
        # for a namespaced robot).
        tf_topic = _add_namespaces(parent_namespace, 'tf')
        tf_static_topic = _add_namespaces(parent_namespace, 'tf_static')

        # Pass __ns / __node as *node-local* arguments.
        super().__init__(
            name,
            cli_args=[
                '--ros-args',
                '-r', f'__ns:={costmap_namespace}',
                '-r', f'__node:={name}',
                '-r', f'/tf:={tf_topic}',
                '-r', f'/tf_static:={tf_static_topic}',
            ],
        )

        # Set use_sim_time before declaring other parameters so that time
        # sources are configured correctly during on_configure.
        if use_sim_time:
            import rclpy.parameter as _rp
            self.set_parameters([
                _rp.Parameter('use_sim_time', _rp.Parameter.Type.BOOL, True)
            ])
        self.get_logger().info('Creating Costmap')

        # ------ Declare all parameters ------
        self.declare_parameter('always_send_full_costmap', False)
        self.declare_parameter('map_vis_z', 0.0)
        self.declare_parameter('footprint_padding', 0.01)
        self.declare_parameter('footprint', '[]')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('height', 5)
        self.declare_parameter('width', 5)
        self.declare_parameter('lethal_cost_threshold', 100)
        self.declare_parameter('observation_sources', '')
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('plugins', self.DEFAULT_PLUGIN_NAMES)
        self.declare_parameter(
            'filters', [], ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('publish_frequency', 1.0)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_radius', 0.1)
        self.declare_parameter('rolling_window', False)
        self.declare_parameter('track_unknown_space', False)
        self.declare_parameter('transform_tolerance', 0.3)
        self.declare_parameter('initial_transform_timeout', 60.0)
        self.declare_parameter('trinary_costmap', True)
        self.declare_parameter('unknown_cost_value', 255)
        self.declare_parameter('update_frequency', 5.0)
        self.declare_parameter('use_maximum', False)
        self.declare_parameter('subscribe_to_stamped_footprint', False)

        # Runtime state
        self._layered_costmap: Optional[LayeredCostmap] = None
        self._tf_buffer: Optional[Buffer] = None
        self._tf_listener: Optional[TransformListener] = None
        self._costmap_publisher: Optional[Costmap2DPublisher] = None
        self._layer_publishers: List[Costmap2DPublisher] = []
        self._footprint_pub = None
        self._footprint_sub = None
        self._clear_costmap_service: Optional[ClearCostmapService] = None
        self._get_cost_service = None

        # Update thread
        self._map_update_thread: Optional[threading.Thread] = None
        self._map_update_thread_shutdown = False
        self._stopped = True
        self._stop_updates = False

        # Plugin provider (nav2_core_py plugin.xml discovery pattern)
        self._plugin_provider = PluginProvider(
            export_tag='nav2_costmap_2d_py',
            base_class_type='nav2_costmap_2d_py.core.layer.Layer',
        )

        # Parameters cache (filled in _get_parameters)
        self._global_frame = 'map'
        self._robot_base_frame = 'base_link'
        self._resolution = 0.05
        self._map_width_meters = 5
        self._map_height_meters = 5
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._rolling_window = False
        self._track_unknown_space = False
        self._update_frequency = 5.0
        self._publish_frequency = 1.0
        self._transform_tolerance = 0.3
        self._initial_transform_timeout = 60.0
        self._always_send_full = False
        self._map_vis_z = 0.0
        self._robot_radius = 0.1
        self._footprint_str = '[]'
        self._footprint_padding = 0.01
        self._use_radius = True
        self._plugin_names: List[str] = []
        self._plugin_types: List[str] = []
        self._filter_names: List[str] = []
        self._filter_types: List[str] = []
        self._subscribe_to_stamped_footprint = False

        # Footprint state
        self._padded_footprint: list = []
        self._unpadded_footprint: list = []

        # Dynamic parameter handler
        self._dyn_params_handler = None

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring')

        try:
            self._get_parameters()
        except Exception as e:
            self.get_logger().error(f'Failed to configure costmap! {e}')
            return TransitionCallbackReturn.FAILURE

        # Discover Python plugins via entry_points
        self._plugin_provider.discover(self)

        # ----- Build LayeredCostmap -----
        self._layered_costmap = LayeredCostmap(
            self._global_frame,
            self._rolling_window,
            self._track_unknown_space,
        )

        if not self._layered_costmap.is_size_locked():
            self._layered_costmap.resize_map(
                int(self._map_width_meters / self._resolution),
                int(self._map_height_meters / self._resolution),
                self._resolution,
                self._origin_x,
                self._origin_y,
            )

        # ----- TF -----
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ----- Load layer plugins -----
        for i, pname in enumerate(self._plugin_names):
            ptype = self._plugin_types[i]
            self.get_logger().info(f'Using plugin "{pname}"')

            plugin = self._plugin_provider.load(ptype, self)
            if plugin is None:
                self.get_logger().fatal(
                    f'Failed to create layer plugin: {pname} of type {ptype}'
                )
                return TransitionCallbackReturn.FAILURE

            with self._layered_costmap.get_costmap().get_mutex():
                self._layered_costmap.add_plugin(plugin)
                try:
                    plugin.initialize(
                        self._layered_costmap, pname, self._tf_buffer, self
                    )
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to initialize costmap plugin {pname}: {e}'
                    )
                    return TransitionCallbackReturn.FAILURE

            self.get_logger().info(f'Initialized plugin "{pname}"')

        # ----- Load filter plugins -----
        for i, fname in enumerate(self._filter_names):
            ftype = self._filter_types[i]
            self.get_logger().info(f'Using costmap filter "{fname}"')

            filt = self._plugin_provider.load(ftype, self)
            if filt is None:
                self.get_logger().fatal(
                    f'Failed to create filter plugin: {fname} of type {ftype}'
                )
                return TransitionCallbackReturn.FAILURE

            with self._layered_costmap.get_costmap().get_mutex():
                self._layered_costmap.add_filter(filt)
                try:
                    filt.initialize(
                        self._layered_costmap, fname, self._tf_buffer, self
                    )
                except Exception as e:
                    self.get_logger().error(
                        f'Failed to initialize costmap filter {fname}: {e}'
                    )
                    return TransitionCallbackReturn.FAILURE

            self.get_logger().info(f'Initialized costmap filter "{fname}"')

        # ----- Publishers / Subscribers -----
        if self._subscribe_to_stamped_footprint:
            from geometry_msgs.msg import PolygonStamped as PS
            self._footprint_sub = self.create_subscription(
                PS, 'footprint',
                lambda msg: self.set_robot_footprint_polygon(msg.polygon),
                1,
            )
        else:
            self._footprint_sub = self.create_subscription(
                Polygon, 'footprint',
                self.set_robot_footprint_polygon,
                1,
            )

        self._footprint_pub = self.create_publisher(
            PolygonStamped, 'published_footprint', 1
        )

        self._costmap_publisher = Costmap2DPublisher(
            self,
            self._layered_costmap.get_costmap(),
            self._global_frame,
            'costmap',
            self._always_send_full,
            self._map_vis_z,
        )

        # Per-layer publishers (for CostmapLayer instances)
        from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
        for layer in self._layered_costmap.get_plugins():
            if isinstance(layer, CostmapLayer):
                pub = Costmap2DPublisher(
                    self, layer,
                    self._global_frame,
                    layer.get_name(),
                    self._always_send_full,
                    self._map_vis_z,
                )
                self._layer_publishers.append(pub)

        # ----- Footprint -----
        if self._use_radius:
            self.set_robot_footprint(
                make_footprint_from_radius(self._robot_radius)
            )
        else:
            fp = make_footprint_from_string(self._footprint_str)
            if fp:
                self.set_robot_footprint(fp)
            else:
                self.get_logger().error(
                    f'Invalid footprint string: "{self._footprint_str}", '
                    f'using radius ({self._robot_radius:.3f})'
                )
                self.set_robot_footprint(
                    make_footprint_from_radius(self._robot_radius)
                )

        # ----- Services -----
        self._clear_costmap_service = ClearCostmapService(self, self)

        from nav2_msgs.srv import GetCosts
        self._get_cost_service = self.create_service(
            GetCosts,
            f'get_cost_{self.get_name()}',
            self._get_costs_callback,
        )

        self.get_logger().info('Costmap configured successfully')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Activating')

        # Wait for TF
        self.get_logger().info('Checking transform')
        deadline = self.get_clock().now().nanoseconds / 1e9 + self._initial_transform_timeout
        rate_sleep = 0.5  # 2 Hz like C++ rclcpp::Rate r(2)
        while rclpy.ok():
            try:
                self._tf_buffer.lookup_transform(
                    self._global_frame,
                    self._robot_base_frame,
                    rclpy.time.Time(),
                )
                break
            except Exception as e:
                self.get_logger().info(
                    f'Waiting for transform {self._robot_base_frame} → '
                    f'{self._global_frame}: {e}'
                )
                if self.get_clock().now().nanoseconds / 1e9 > deadline:
                    self.get_logger().error(
                        f'Failed to activate {self.get_name()}: '
                        f'transform {self._robot_base_frame} → '
                        f'{self._global_frame} not available before timeout'
                    )
                    return TransitionCallbackReturn.FAILURE
                time.sleep(rate_sleep)

        # Activate publishers
        self._costmap_publisher.on_activate()
        for pub in self._layer_publishers:
            pub.on_activate()

        # Start update thread
        self._stopped = True   # plugins not yet activated
        self._stop_updates = False
        self._map_update_thread_shutdown = False
        self._map_update_thread = threading.Thread(
            target=self._map_update_loop,
            args=(self._update_frequency,),
            daemon=True,
        )
        self._map_update_thread.start()

        self._start()   # calls plugin.activate() and sets stopped_ = False

        # Dynamic parameter callback
        self._dyn_params_handler = self.add_on_set_parameters_callback(
            self._dynamic_parameters_callback
        )

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating')

        if self._dyn_params_handler:
            self.remove_on_set_parameters_callback(self._dyn_params_handler)
            self._dyn_params_handler = None

        self._stop()

        # Stop update thread
        self._map_update_thread_shutdown = True
        if self._map_update_thread and self._map_update_thread.is_alive():
            self._map_update_thread.join(timeout=5.0)
        self._map_update_thread = None

        if self._costmap_publisher:
            self._costmap_publisher.on_deactivate()
        for pub in self._layer_publishers:
            pub.on_deactivate()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up')

        self._costmap_publisher = None
        self._clear_costmap_service = None
        self._get_cost_service = None
        self._layer_publishers.clear()
        self._layered_costmap = None
        self._tf_listener = None
        self._tf_buffer = None
        self._footprint_sub = None
        self._footprint_pub = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down')
        return TransitionCallbackReturn.SUCCESS

    # ------------------------------------------------------------------
    # Parameter reading
    # ------------------------------------------------------------------

    def _get_parameters(self) -> None:
        """Read all ROS2 parameters into member variables."""
        self.get_logger().debug('getParameters')

        def _g(name):
            return self.get_parameter(name).value

        self._always_send_full = _g('always_send_full_costmap')
        self._map_vis_z = _g('map_vis_z')
        self._footprint_str = _g('footprint')
        self._footprint_padding = _g('footprint_padding')
        self._global_frame = _g('global_frame')
        self._map_height_meters = _g('height')
        self._origin_x = _g('origin_x')
        self._origin_y = _g('origin_y')
        self._publish_frequency = _g('publish_frequency')
        self._resolution = _g('resolution')
        self._robot_base_frame = _g('robot_base_frame')
        self._robot_radius = _g('robot_radius')
        self._rolling_window = _g('rolling_window')
        self._track_unknown_space = _g('track_unknown_space')
        self._transform_tolerance = _g('transform_tolerance')
        self._initial_transform_timeout = _g('initial_transform_timeout')
        self._update_frequency = _g('update_frequency')
        self._map_width_meters = _g('width')
        self._plugin_names = _g('plugins')
        self._filter_names = _g('filters')
        self._subscribe_to_stamped_footprint = _g('subscribe_to_stamped_footprint')

        # Declare default plugin.plugin params if using defaults
        if self._plugin_names == self.DEFAULT_PLUGIN_NAMES:
            for pname, ptype in zip(self.DEFAULT_PLUGIN_NAMES,
                                    self.DEFAULT_PLUGIN_TYPES):
                param = f'{pname}.plugin'
                if not self.has_parameter(param):
                    self.declare_parameter(param, ptype)

        # Resolve plugin types
        self._plugin_types = [
            self._get_plugin_type_param(n) for n in self._plugin_names
        ]
        self._filter_types = [
            self._get_plugin_type_param(n) for n in self._filter_names
        ]

        # Footprint mode
        self._use_radius = True
        if self._footprint_str and self._footprint_str not in ('', '[]'):
            if make_footprint_from_string(self._footprint_str):
                self._use_radius = False

        # Validate map dimensions
        if self._map_width_meters <= 0:
            self.get_logger().error('Map width must be positive')
        if self._map_height_meters <= 0:
            self.get_logger().error('Map height must be positive')

    def _get_plugin_type_param(self, plugin_name: str) -> str:
        param_name = f'{plugin_name}.plugin'
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, '')
        val = self.get_parameter(param_name).value
        if not val:
            raise RuntimeError(
                f"Parameter '{param_name}' is not defined. "
                f"Set it in your YAML config."
            )
        return val

    # ------------------------------------------------------------------
    # Update loop
    # ------------------------------------------------------------------

    def _map_update_loop(self, frequency: float) -> None:
        """
        Background thread that calls ``update_map`` at *frequency* Hz.
        """
        self.get_logger().debug(f'mapUpdateLoop frequency: {frequency}')
        if frequency == 0.0:
            return

        self.get_logger().debug('Entering update loop')
        period = 1.0 / frequency
        last_publish = time.monotonic()
        publish_period = (
            1.0 / self._publish_frequency if self._publish_frequency > 0 else -1.0
        )

        while rclpy.ok() and not self._map_update_thread_shutdown:
            start = time.monotonic()

            if not self._stopped:
                try:
                    self._update_map()
                except Exception:  # noqa: BLE001
                    self.get_logger().error(
                        f'Costmap update_map() failed:\n{traceback.format_exc()}',
                        throttle_duration_sec=2.0,
                    )

                # Publish at publish_frequency
                now = time.monotonic()
                if publish_period > 0 and (now - last_publish) >= publish_period:
                    try:
                        if self._costmap_publisher:
                            self._costmap_publisher.publish_costmap()
                        for pub in self._layer_publishers:
                            pub.publish_costmap()
                        self._publish_footprint()
                    except Exception:  # noqa: BLE001
                        self.get_logger().error(
                            f'Costmap publish failed:\n{traceback.format_exc()}',
                            throttle_duration_sec=2.0,
                        )
                    last_publish = now

            elapsed = time.monotonic() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self.get_logger().warning(
                    f'Map update loop missed its desired rate of {frequency:.4f} Hz. '
                    f'Actual rate: {1.0 / max(elapsed, 1e-9):.4f} Hz.'
                )

    def _update_map(self) -> None:
        """
        Single costmap update step.
        """
        if self._stop_updates or self._layered_costmap is None:
            return

        pose = self.get_robot_pose()
        if pose is None:
            return

        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = _yaw_from_pose_stamped(pose)

        self._layered_costmap.update_map(x, y, yaw)

    # ------------------------------------------------------------------
    # Start / Stop helpers
    # ------------------------------------------------------------------

    def _start(self) -> None:
        """
        Activate all layer plugins and start the update loop.
        """
        if self._stopped:
            for plugin in self._layered_costmap.get_plugins():
                plugin.activate()
            for f in self._layered_costmap.get_filters():
                f.activate()
            self._stopped = False

    def _stop(self) -> None:
        """
        Deactivate all layer plugins.
        """
        self._stop_updates = True
        for plugin in self._layered_costmap.get_plugins():
            plugin.deactivate()
        for f in self._layered_costmap.get_filters():
            f.deactivate()
        self._stopped = True
        self._stop_updates = False

    # ------------------------------------------------------------------
    # Footprint management
    # ------------------------------------------------------------------

    def set_robot_footprint(self, points: list) -> None:
        """
        Set padded and unpadded footprints and propagate to LayeredCostmap.
        """
        self._unpadded_footprint = list(points)
        self._padded_footprint = pad_footprint(points, self._footprint_padding)
        if self._layered_costmap:
            self._layered_costmap.set_footprint(self._padded_footprint)

    def set_robot_footprint_polygon(self, polygon: Polygon) -> None:
        """
        Set footprint from a geometry_msgs/Polygon message.
        """
        pts = [(p.x, p.y) for p in polygon.points]
        self.set_robot_footprint(pts)

    def get_oriented_footprint(self) -> list:
        """
        Return the footprint rotated/translated to the robot's current pose.
        """
        pose = self.get_robot_pose()
        if pose is None:
            return []
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = _yaw_from_pose_stamped(pose)
        return transform_footprint(x, y, yaw, self._padded_footprint)

    def _publish_footprint(self) -> None:
        if self._footprint_pub is None:
            return
        pose = self.get_robot_pose()
        if pose is None:
            return
        x = pose.pose.position.x
        y = pose.pose.position.y
        yaw = _yaw_from_pose_stamped(pose)
        oriented = transform_footprint(x, y, yaw, self._padded_footprint)
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame
        for wx, wy in oriented:
            pt = Point32()
            pt.x = wx
            pt.y = wy
            msg.polygon.points.append(pt)
        self._footprint_pub.publish(msg)

    # ------------------------------------------------------------------
    # TF / robot pose
    # ------------------------------------------------------------------

    def get_robot_pose(self) -> Optional[PoseStamped]:
        """
        Look up the current robot pose in the global frame via TF.

        Returns None on failure.
        """
        if self._tf_buffer is None:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(
                    seconds=self._transform_tolerance
                ),
            )
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception:
            return None

    def transform_pose_to_global_frame(
        self,
        input_pose: PoseStamped,
    ) -> Optional[PoseStamped]:
        """
        Transform *input_pose* into the global frame.
        """
        if self._tf_buffer is None:
            return None
        try:
            return self._tf_buffer.transform(
                input_pose,
                self._global_frame,
                timeout=rclpy.duration.Duration(
                    seconds=self._transform_tolerance
                ),
            )
        except Exception as e:
            self.get_logger().error(f'transformPoseToGlobalFrame failed: {e}')
            return None

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _get_costs_callback(self, request, response):
        """
        Return cost at one or more world-frame points.
        """
        costmap = self._layered_costmap.get_costmap()
        costs = []
        for pose in request.poses:
            # Transform to global frame if needed
            ps = PoseStamped()
            ps.header = request.header
            ps.pose = pose
            global_ps = self.transform_pose_to_global_frame(ps)
            if global_ps is None:
                costs.append(-1.0)
                continue
            wx = global_ps.pose.position.x
            wy = global_ps.pose.position.y
            ok, mx, my = costmap.world_to_map(wx, wy)
            if not ok:
                costs.append(-1.0)
            else:
                costs.append(float(costmap.get_cost(mx, my)))
        response.costs = costs
        return response

    def reset_layers(self) -> None:
        """
        Reset each individual layer.
        """
        if self._layered_costmap:
            for layer in self._layered_costmap.get_plugins():
                layer.reset()
            for f in self._layered_costmap.get_filters():
                f.reset()

    # ------------------------------------------------------------------
    # Dynamic parameters
    # ------------------------------------------------------------------

    def _dynamic_parameters_callback(self, params) -> SetParametersResult:
        """
        Handle dynamic parameter changes at runtime.
        """
        result = SetParametersResult()
        result.successful = True

        for param in params:
            if param.name == 'update_frequency':
                self._update_frequency = param.value
            elif param.name == 'publish_frequency':
                self._publish_frequency = param.value
            elif param.name == 'robot_radius':
                self._robot_radius = param.value
                if self._use_radius:
                    self.set_robot_footprint(
                        make_footprint_from_radius(param.value)
                    )
            elif param.name == 'footprint':
                fp = make_footprint_from_string(param.value)
                if fp:
                    self._footprint_str = param.value
                    self._use_radius = False
                    self.set_robot_footprint(fp)
                else:
                    result.successful = False
                    result.reason = f'Invalid footprint string: {param.value}'
            elif param.name == 'footprint_padding':
                self._footprint_padding = param.value
                self.set_robot_footprint(self._unpadded_footprint)
            elif param.name in ('width', 'height', 'resolution',
                                'origin_x', 'origin_y'):
                # Trigger resize on next update
                if param.name == 'width':
                    self._map_width_meters = param.value
                elif param.name == 'height':
                    self._map_height_meters = param.value
                elif param.name == 'resolution':
                    self._resolution = param.value
                elif param.name == 'origin_x':
                    self._origin_x = param.value
                elif param.name == 'origin_y':
                    self._origin_y = param.value
                if self._layered_costmap:
                    self._layered_costmap.resize_map(
                        int(self._map_width_meters / self._resolution),
                        int(self._map_height_meters / self._resolution),
                        self._resolution,
                        self._origin_x,
                        self._origin_y,
                    )
                    self._update_map()

        return result

    # ------------------------------------------------------------------
    # Public accessors
    # ------------------------------------------------------------------

    def get_costmap(self) -> Optional[Costmap2D]:
        if self._layered_costmap:
            return self._layered_costmap.get_costmap()
        return None

    def get_layered_costmap(self) -> Optional[LayeredCostmap]:
        return self._layered_costmap

    def get_name(self) -> str:
        return self._name

    def get_global_frame_id(self) -> str:
        return self._global_frame

    def get_base_frame_id(self) -> str:
        return self._robot_base_frame

    def get_tf_buffer(self):
        """Return the TF2 buffer."""
        return self._tf_buffer

    def get_transform_tolerance(self) -> float:
        return self._transform_tolerance

    def get_robot_radius(self) -> float:
        return self._robot_radius

    def is_current(self) -> bool:
        if self._layered_costmap:
            return self._layered_costmap.is_current()
        return False

    def wait_until_current(self, timeout) -> None:
        """
        Block until the costmap is current or *timeout* expires.

        Parameters
        ----------
        timeout :
            ``rclpy.duration.Duration`` maximum time to wait.

        Raises
        ------
        RuntimeError
            If the timeout expires before the costmap becomes current.
        """
        period = 1.0 / 100.0  # 100 Hz
        waiting_start = self.get_clock().now()
        while not self.is_current():
            if self.get_clock().now() - waiting_start > timeout:
                raise RuntimeError('Costmap timed out waiting for update')
            time.sleep(period)

    def get_footprint(self) -> list:
        return self._padded_footprint

    def get_unpadded_footprint(self) -> list:
        return self._unpadded_footprint

    def is_using_radius(self) -> bool:
        return self._use_radius


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _add_namespaces(parent: str, local: str) -> str:
    """
    Join a parent namespace and a local name into a fully-qualified namespace.

    Handles the root namespace and any trailing slashes in the parent.
    """
    if not parent or parent == '/':
        return '/' + local
    return parent.rstrip('/') + '/' + local


def _yaw_from_pose_stamped(pose: PoseStamped) -> float:
    """Extract yaw from a PoseStamped quaternion."""
    q = pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    try:
        node = Costmap2DROS()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
