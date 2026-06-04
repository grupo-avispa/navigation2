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
Nav2 Controller Server.

It mirrors the nav2_controller::ControllerServer from the C++ implementation,
using rclpy LifecycleNode, action_msgs, and the nav2_msgs action interfaces.

It hosts a variety of plugins of different algorithms (controllers, goal
checkers, progress checkers and path handlers) to complete control tasks from
the exposed FollowPath action server.

Plugins must be Python classes registered via a plugin_provider (see
nav2_core_py.plugin_provider) that implement the corresponding nav2_core_py
interface.
"""

import math
import threading
import time
import traceback
from typing import Dict, Optional

from bondpy import bondpy  # type: ignore[import-untyped]
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_controller_py.geometry_utils import (calculate_path_length, euclidean_distance, get_yaw,
                                               shortest_angular_distance)
from nav2_controller_py.parameter_handler import ParameterHandler, Parameters
from nav2_controller_py.path_utils import distance_from_path, transform_pose_in_target_frame
from nav2_core_py.controller import Controller
from nav2_core_py.controller_exceptions import (ControllerException, ControllerTFError,
                                                ControllerTimedOut, FailedToMakeProgress,
                                                InvalidController, InvalidPath, NoValidControl,
                                                PatienceExceeded)
from nav2_core_py.goal_checker import GoalChecker
from nav2_core_py.path_handler import PathHandler
from nav2_core_py.plugin_provider import PluginProvider
from nav2_core_py.progress_checker import ProgressChecker
from nav2_costmap_2d_py import Costmap2DROS
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit, TrackingFeedback
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

# Type aliases
ControllerMap = Dict[str, Controller]
GoalCheckerMap = Dict[str, GoalChecker]
ProgressCheckerMap = Dict[str, ProgressChecker]
PathHandlerMap = Dict[str, PathHandler]

# Base-class type strings used by the PluginProvider to filter plugin manifests.
_CONTROLLER_BASE = 'nav2_core_py.controller.Controller'
_GOAL_CHECKER_BASE = 'nav2_core_py.goal_checker.GoalChecker'
_PROGRESS_CHECKER_BASE = 'nav2_core_py.progress_checker.ProgressChecker'
_PATH_HANDLER_BASE = 'nav2_core_py.path_handler.PathHandler'


class ControllerServer(LifecycleNode):
    """
    ROS2 Lifecycle Node implementing the Nav2 Controller Server.

    It hosts:
      - Maps of Controller / GoalChecker / ProgressChecker / PathHandler plugins.
      - A FollowPath action server (/follow_path).
      - A local costmap (nav2_costmap_2d_py.Costmap2DROS).
    """

    def __init__(self) -> None:
        super().__init__('controller_server')

        # Plugin registries {plugin_id: plugin_instance}
        self._controllers: ControllerMap = {}
        self._goal_checkers: GoalCheckerMap = {}
        self._progress_checkers: ProgressCheckerMap = {}
        self._path_handlers: PathHandlerMap = {}

        # Concatenated id strings for diagnostics
        self._controller_ids_concat = ''
        self._goal_checker_ids_concat = ''
        self._progress_checker_ids_concat = ''
        self._path_handler_ids_concat = ''

        # Currently selected plugin ids (set per-goal)
        self._current_controller = ''
        self._current_goal_checker = ''
        self._current_progress_checker = ''
        self._current_path_handler = ''

        # Plugin providers (XML manifest discovery, same mechanism as planner)
        self._controller_plugin_provider = PluginProvider(
            export_tag='nav2_controller_py', base_class_type=_CONTROLLER_BASE)
        self._goal_checker_plugin_provider = PluginProvider(
            export_tag='nav2_controller_py', base_class_type=_GOAL_CHECKER_BASE)
        self._progress_checker_plugin_provider = PluginProvider(
            export_tag='nav2_controller_py', base_class_type=_PROGRESS_CHECKER_BASE)
        self._path_handler_plugin_provider = PluginProvider(
            export_tag='nav2_controller_py', base_class_type=_PATH_HANDLER_BASE)

        # Parameter handler
        self._param_handler: Optional[ParameterHandler] = None
        self._params: Optional[Parameters] = None

        # TF / costmap interfaces
        self._tf_buffer = None
        self._costmap_ros: Optional[Costmap2DROS] = None
        self._costmap_executor: Optional[rclpy.executors.SingleThreadedExecutor] = None
        self._costmap_thread: Optional[threading.Thread] = None
        self._costmap_activated = threading.Event()
        self._costmap_activate_thread: Optional[threading.Thread] = None
        self._transform_tolerance = 0.1

        # Path / control state
        self._start_index = 0
        self._end_pose: Optional[PoseStamped] = None
        self._transformed_end_pose: Optional[PoseStamped] = None
        self._current_path: Path = Path()
        self._transformed_global_plan: Path = Path()
        self._last_valid_cmd_time = None

        # Odometry
        self._odom_twist = Twist()
        self._odom_lock = threading.Lock()

        # ROS interfaces (created in on_configure)
        self._action_server: Optional[ActionServer] = None
        self._vel_publisher = None
        self._transformed_plan_pub = None
        self._tracking_feedback_pub = None
        self._speed_limit_sub = None
        self._odom_sub = None
        self._callback_group = ReentrantCallbackGroup()

        # Bond to lifecycle manager
        self._bond: Optional[bondpy.Bond] = None
        self._bond_heartbeat_period = 0.1

        self.get_logger().info('Creating controller server')

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure costmap, parameters and plugins."""
        self.get_logger().info('Configuring controller interface')

        # --- Local costmap --------------------------------------------
        try:
            self._costmap_ros = Costmap2DROS(
                name='local_costmap',
                parent_namespace=str(self.get_namespace()),
                use_sim_time=self.get_parameter_or('use_sim_time', rclpy.parameter.Parameter(
                    'use_sim_time', rclpy.Parameter.Type.BOOL, False)).value,
            )
            result = self._costmap_ros.on_configure(None)  # type: ignore[arg-type]
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError('Failed to configure costmap')
            self._costmap_executor = rclpy.executors.SingleThreadedExecutor()
            self._costmap_executor.add_node(self._costmap_ros)
            self._costmap_thread = threading.Thread(
                target=self._costmap_executor.spin, daemon=True, name='costmap_executor')
            self._costmap_thread.start()
            self._tf_buffer = self._costmap_ros.get_tf_buffer()
            self._transform_tolerance = self._costmap_ros.get_transform_tolerance()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().fatal(f'Failed to initialize Costmap2DROS: {ex}')
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

        # --- Parameter handler ----------------------------------------
        try:
            self._param_handler = ParameterHandler(self)
        except RuntimeError as ex:
            self.get_logger().fatal(
                f'Failed to initialize parameter handler: {ex}\n' + traceback.format_exc())
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE
        self._params = self._param_handler.get_parameters()

        # --- Discover plugins -----------------------------------------
        self._controller_plugin_provider.discover(self)
        self._goal_checker_plugin_provider.discover(self)
        self._progress_checker_plugin_provider.discover(self)
        self._path_handler_plugin_provider.discover(self)

        params = self._params

        # --- Progress checkers ----------------------------------------
        for pid, ptype in zip(params.progress_checker_ids, params.progress_checker_types):
            pc = self._progress_checker_plugin_provider.load(ptype, self)
            if pc is None:
                self.get_logger().fatal(
                    f'Failed to create progress_checker: {pid} of type {ptype}')
                self.on_cleanup(state)
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'Created progress_checker : {pid} of type {ptype}')
            self._progress_checkers[pid] = pc
        self._progress_checker_ids_concat = ' '.join(params.progress_checker_ids) or '(none)'
        self.get_logger().info(
            f'Controller Server has {self._progress_checker_ids_concat} '
            'progress checkers available.')

        # --- Goal checkers --------------------------------------------
        for pid, ptype in zip(params.goal_checker_ids, params.goal_checker_types):
            gc = self._goal_checker_plugin_provider.load(ptype, self)
            if gc is None:
                self.get_logger().fatal(
                    f'Failed to create goal_checker: {pid} of type {ptype}')
                self.on_cleanup(state)
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'Created goal checker : {pid} of type {ptype}')
            self._goal_checkers[pid] = gc
        self._goal_checker_ids_concat = ' '.join(params.goal_checker_ids)
        self.get_logger().info(
            f'Controller Server has {self._goal_checker_ids_concat} goal checkers available.')

        # --- Path handlers --------------------------------------------
        for pid, ptype in zip(params.path_handler_ids, params.path_handler_types):
            ph = self._path_handler_plugin_provider.load(ptype, self)
            if ph is None:
                self.get_logger().fatal(
                    f'Failed to create path handler: {pid} of type {ptype}')
                self.on_cleanup(state)
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'Created path handler : {pid} of type {ptype}')
            self._path_handlers[pid] = ph
        self._path_handler_ids_concat = ' '.join(params.path_handler_ids)
        self.get_logger().info(
            f'Controller Server has {self._path_handler_ids_concat} path handlers available.')

        # --- Controllers ----------------------------------------------
        for pid, ptype in zip(params.controller_ids, params.controller_types):
            controller = self._controller_plugin_provider.load(ptype, self)
            if controller is None:
                self.get_logger().fatal(
                    f'Failed to create controller: {pid} of type {ptype}')
                self.on_cleanup(state)
                return TransitionCallbackReturn.FAILURE
            try:
                controller.configure(self, pid, self._tf_buffer, self._costmap_ros)
            except Exception as ex:  # noqa: BLE001
                self.get_logger().fatal(
                    f'Failed to configure controller {pid}: {ex}\n{traceback.format_exc()}')
                self.on_cleanup(state)
                return TransitionCallbackReturn.FAILURE
            self.get_logger().info(f'Created controller : {pid} of type {ptype}')
            self._controllers[pid] = controller
        self._controller_ids_concat = ' '.join(params.controller_ids)
        self.get_logger().info(
            f'Controller Server has {self._controller_ids_concat} controllers available.')

        # --- Publishers / subscribers / action server -----------------
        self._odom_sub = self.create_subscription(
            Odometry, params.odom_topic, self._odom_callback, 10)
        self._vel_publisher = self.create_publisher(TwistStamped, 'cmd_vel', 1)
        self._transformed_plan_pub = self.create_publisher(Path, 'transformed_global_plan', 1)
        self._tracking_feedback_pub = self.create_publisher(
            TrackingFeedback, 'tracking_feedback', 1)

        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self._compute_control,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self._speed_limit_sub = self.create_subscription(
            SpeedLimit, params.speed_limit_topic, self._speed_limit_callback, 10)

        self.get_logger().info('Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate costmap and all plugins."""
        self.get_logger().info('Activating')
        self._costmap_activated.clear()

        self.create_bond()

        self._costmap_activate_thread = threading.Thread(
            target=self._activate_background, args=(state,), daemon=True,
            name='controller_activate')
        self._costmap_activate_thread.start()

        self._last_valid_cmd_time = self.get_clock().now()
        return TransitionCallbackReturn.SUCCESS

    def _activate_background(self, state: LifecycleState) -> None:
        """Run costmap and plugin activation in a background thread."""
        try:
            if self._costmap_ros is not None:
                result = self._costmap_ros.on_activate(state)
                if result == TransitionCallbackReturn.SUCCESS:
                    self._costmap_activated.set()
                    self.get_logger().info('Costmap activated (background)')
                else:
                    self.get_logger().error('Background costmap activation failed.')
            else:
                self._costmap_activated.set()

            for controller in self._controllers.values():
                controller.activate()

            # Initialize goal checker, progress checker and path handler
            for pid, pc in self._progress_checkers.items():
                pc.initialize(self, pid)
            for pid, gc in self._goal_checkers.items():
                gc.initialize(self, pid, self._costmap_ros)
            for pid, ph in self._path_handlers.items():
                ph.initialize(self, self.get_logger(), pid, self._costmap_ros, self._tf_buffer)

            self.get_logger().info('Activated (background)')
        except Exception:  # noqa: BLE001
            self.get_logger().fatal(f'Background activation failed:\n{traceback.format_exc()}')

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate plugins and costmap."""
        self.get_logger().info('Deactivating')

        if self._costmap_activate_thread is not None and self._costmap_activate_thread.is_alive():
            self.get_logger().info('Waiting for background activation to complete...')
            self._costmap_activate_thread.join(timeout=10.0)

        for controller in self._controllers.values():
            controller.deactivate()

        if self._costmap_ros is not None:
            self._costmap_ros.on_deactivate(state)

        self._publish_zero_velocity()
        self.destroy_bond()

        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup plugins, action server and costmap."""
        self.get_logger().info('Cleaning up')

        for controller in self._controllers.values():
            controller.cleanup()
        self._controllers.clear()
        self._goal_checkers.clear()
        self._progress_checkers.clear()
        self._path_handlers.clear()

        if self._action_server is not None:
            self._action_server.destroy()
            self._action_server = None

        if self._costmap_activate_thread is not None \
                and self._costmap_activate_thread.is_alive() \
                and threading.current_thread() is not self._costmap_activate_thread:
            self._costmap_activate_thread.join(timeout=5.0)
        self._costmap_activate_thread = None
        self._costmap_activated.clear()

        if self._costmap_ros is not None:
            self._costmap_ros.on_cleanup(state)
            self._costmap_ros = None
        if self._costmap_executor is not None:
            self._costmap_executor.shutdown(timeout_sec=2.0)
            self._costmap_executor = None
        if self._costmap_thread is not None:
            self._costmap_thread.join(timeout=2.0)
            self._costmap_thread = None

        self._tf_buffer = None
        self._vel_publisher = None
        self._transformed_plan_pub = None
        self._tracking_feedback_pub = None
        self._odom_sub = None
        self._speed_limit_sub = None

        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the node."""
        self.get_logger().info('Shutting down')
        return TransitionCallbackReturn.SUCCESS

    def create_bond(self) -> None:
        """Create bond connection to lifecycle manager."""
        if self._bond_heartbeat_period > 0.0:
            self.get_logger().info(
                f'Creating bond ({self.get_name()}) to lifecycle manager.')
            self._bond = bondpy.Bond(node=self, topic='bond', bond_id=self.get_name())
            self._bond.heartbeat_period = self._bond_heartbeat_period
            self._bond.heartbeat_timeout = 4.0
            self._bond.start()

    def destroy_bond(self) -> None:
        """Destroy bond connection to lifecycle manager."""
        if self._bond_heartbeat_period > 0.0 and self._bond is not None:
            self.get_logger().info(
                f'Destroying bond ({self.get_name()}) to lifecycle manager.')
            self._bond.shutdown()
            self._bond = None

    # ------------------------------------------------------------------
    # Action server callbacks
    # ------------------------------------------------------------------

    def _goal_callback(self, goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def _compute_control(self, goal_handle):
        """Handle the FollowPath action; mirrors ControllerServer::computeControl()."""
        with self._param_handler.get_mutex():
            return self._compute_control_impl(goal_handle)

    def _compute_control_impl(self, goal_handle):
        self.get_logger().info('Received a goal, begin computing control effort.')
        result = FollowPath.Result()
        try:
            goal = goal_handle.request

            current_controller = self._find_controller_id(goal.controller_id)
            if current_controller is None:
                raise InvalidController(
                    f'Failed to find controller name: {goal.controller_id}')
            self._current_controller = current_controller

            current_goal_checker = self._find_goal_checker_id(goal.goal_checker_id)
            if current_goal_checker is None:
                raise ControllerException(
                    f'Failed to find goal checker name: {goal.goal_checker_id}')
            self._current_goal_checker = current_goal_checker

            current_progress_checker = self._find_progress_checker_id(goal.progress_checker_id)
            if current_progress_checker is None:
                raise ControllerException(
                    f'Failed to find progress checker name: {goal.progress_checker_id}')
            self._current_progress_checker = current_progress_checker

            current_path_handler = self._find_path_handler_id(goal.path_handler_id)
            if current_path_handler is None:
                raise ControllerException(
                    f'Failed to find path handler name: {goal.path_handler_id}')
            self._current_path_handler = current_path_handler

            self._set_planner_path(goal.path)
            if self._current_progress_checker:
                self._progress_checkers[self._current_progress_checker].reset()

            self._last_valid_cmd_time = self.get_clock().now()
            period = 1.0 / self._params.controller_frequency

            while rclpy.ok():
                start_time = self.get_clock().now()

                if self._action_server is None or not goal_handle.is_active:
                    self.get_logger().debug('Action server unavailable or inactive. Stopping.')
                    return result

                if goal_handle.is_cancel_requested:
                    if self._controllers[self._current_controller].cancel():
                        self.get_logger().info('Cancellation was successful. Stopping the robot.')
                        goal_handle.canceled()
                        self._on_goal_exit(True)
                        return FollowPath.Result()
                    else:
                        self.get_logger().info(
                            'Waiting for the controller to finish cancellation')

                costmap_wait = self._wait_for_costmap()
                self._update_global_path(goal_handle)
                self._compute_and_publish_velocity(goal_handle)

                if self._is_goal_reached():
                    self.get_logger().info('Reached the goal!')
                    break

                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    wait_msg = (f' Waited {costmap_wait:.4f}s for costmap update.'
                                if costmap_wait > 0.0 else '')
                    self.get_logger().warning(
                        'Control loop missed its desired rate of '
                        f'{self._params.controller_frequency:.4f} Hz. Current loop rate is '
                        f'{1.0 / max(elapsed, 1e-9):.4f} Hz.{wait_msg}')

        except InvalidController as e:
            return self._terminate_goal(
                goal_handle, result, FollowPath.Result.INVALID_CONTROLLER, e)
        except ControllerTFError as e:
            return self._terminate_goal(goal_handle, result, FollowPath.Result.TF_ERROR, e)
        except NoValidControl as e:
            return self._terminate_goal(goal_handle, result, FollowPath.Result.NO_VALID_CONTROL, e)
        except FailedToMakeProgress as e:
            return self._terminate_goal(
                goal_handle, result, FollowPath.Result.FAILED_TO_MAKE_PROGRESS, e)
        except PatienceExceeded as e:
            return self._terminate_goal(
                goal_handle, result, FollowPath.Result.PATIENCE_EXCEEDED, e)
        except InvalidPath as e:
            return self._terminate_goal(goal_handle, result, FollowPath.Result.INVALID_PATH, e)
        except ControllerTimedOut as e:
            return self._terminate_goal(
                goal_handle, result, FollowPath.Result.CONTROLLER_TIMED_OUT, e)
        except ControllerException as e:
            return self._terminate_goal(goal_handle, result, FollowPath.Result.UNKNOWN, e)
        except Exception as e:  # noqa: BLE001
            return self._terminate_goal(goal_handle, result, FollowPath.Result.UNKNOWN, e)

        self.get_logger().debug('Controller succeeded, setting result')
        self._on_goal_exit(False)
        goal_handle.succeed()
        return result

    def _terminate_goal(self, goal_handle, result, error_code, exception):
        """Log, stop the robot and abort the goal with the given error code."""
        self.get_logger().error(str(exception))
        self._on_goal_exit(True)
        result.error_code = error_code
        result.error_msg = str(exception)
        goal_handle.abort()
        return result

    # ------------------------------------------------------------------
    # Control helpers (mirror private methods of controller_server.cpp)
    # ------------------------------------------------------------------

    def _wait_for_costmap(self) -> float:
        """Wait for the costmap to become current; mirrors waitForCostmap()."""
        timeout = self._params.costmap_update_timeout
        if self._costmap_ros is not None and timeout.nanoseconds > 0:
            waiting_start = self.get_clock().now()
            was_waiting = not self._costmap_ros.is_current()
            try:
                self._costmap_ros.wait_until_current(timeout)
            except Exception as ex:
                raise ControllerTimedOut(str(ex))
            if was_waiting:
                return (self.get_clock().now() - waiting_start).nanoseconds / 1e9
        return 0.0

    def _set_planner_path(self, path: Path) -> None:
        """Assign the path to the controller; mirrors setPlannerPath()."""
        self.get_logger().debug(
            f'Providing path to the controller {self._current_controller}')
        if not path.poses:
            raise InvalidPath('Path is empty.')

        self._controllers[self._current_controller].new_path_received(path)
        self._path_handlers[self._current_path_handler].set_plan(path)

        self._end_pose = path.poses[-1]
        self._end_pose.header.frame_id = path.header.frame_id
        self._goal_checkers[self._current_goal_checker].reset()

        self.get_logger().debug(
            f'Path end point is ({self._end_pose.pose.position.x:.2f}, '
            f'{self._end_pose.pose.position.y:.2f})')

        self._start_index = 0
        self._current_path = path

    def _compute_and_publish_velocity(self, goal_handle) -> None:
        """Compute and publish velocity; mirrors computeAndPublishVelocity()."""
        pose = self._get_robot_pose()
        if pose is None:
            raise ControllerTFError('Failed to obtain robot pose')

        if self._current_progress_checker:
            if not self._progress_checkers[self._current_progress_checker].check(pose):
                raise FailedToMakeProgress('Failed to make progress')

        twist = self._get_thresholded_twist(self._get_raw_twist())

        ph = self._path_handlers[self._current_path_handler]
        goal = ph.get_transformed_goal(pose.header.stamp)
        closest_point, pruned_plan_end = ph.find_plan_segment(pose)
        self._transformed_global_plan = ph.transform_local_plan(closest_point, pruned_plan_end)
        if self._transformed_plan_pub.get_subscription_count() > 0:
            self._transformed_plan_pub.publish(self._transformed_global_plan)

        cmd_vel_2d = TwistStamped()
        try:
            cmd_vel_2d = self._controllers[self._current_controller].compute_velocity_commands(
                pose, twist, self._goal_checkers[self._current_goal_checker],
                self._transformed_global_plan, goal)
            self._last_valid_cmd_time = self.get_clock().now()
            cmd_vel_2d.header.frame_id = self._costmap_ros.get_base_frame_id()
            cmd_vel_2d.header.stamp = self._last_valid_cmd_time.to_msg()
        except NoValidControl as e:
            failure_tolerance = self._params.failure_tolerance
            if failure_tolerance > 0 or failure_tolerance == -1.0:
                self.get_logger().warning(str(e))
                cmd_vel_2d = TwistStamped()
                cmd_vel_2d.header.frame_id = self._costmap_ros.get_base_frame_id()
                cmd_vel_2d.header.stamp = self.get_clock().now().to_msg()
                elapsed = (self.get_clock().now() - self._last_valid_cmd_time).nanoseconds / 1e9
                if elapsed > failure_tolerance and failure_tolerance != -1.0:
                    raise PatienceExceeded('Controller patience exceeded')
            else:
                raise NoValidControl(str(e))

        self.get_logger().debug(
            f'Publishing velocity at time {self.get_clock().now().nanoseconds / 1e9:.2f}')
        self._publish_velocity(cmd_vel_2d)

        # --- Tracking feedback ----------------------------------------
        current_tracking_feedback = TrackingFeedback()
        self._end_pose.header.stamp = pose.header.stamp
        self._transformed_end_pose = transform_pose_in_target_frame(
            self._end_pose, self._tf_buffer, self._costmap_ros.get_global_frame_id(),
            self._transform_tolerance)
        if self._transformed_end_pose is None:
            raise ControllerTFError('Failed to transform end pose to global frame')

        if len(self._current_path.poses) >= 2:
            current_distance_to_goal = euclidean_distance(
                pose.pose, self._transformed_end_pose.pose)

            robot_pose_in_path_frame = transform_pose_in_target_frame(
                pose, self._tf_buffer, self._current_path.header.frame_id,
                self._transform_tolerance)
            if robot_pose_in_path_frame is None:
                raise ControllerTFError('Failed to transform robot pose to path frame')

            path_search_result = distance_from_path(
                self._current_path, robot_pose_in_path_frame.pose,
                self._start_index, self._params.search_window)

            heading_tracking_error = 0.0
            if path_search_result.closest_segment_index < len(self._current_path.poses) - 1:
                seg_start = self._current_path.poses[
                    path_search_result.closest_segment_index].pose
                seg_end = self._current_path.poses[
                    path_search_result.closest_segment_index + 1].pose
                path_yaw = math.atan2(
                    seg_end.position.y - seg_start.position.y,
                    seg_end.position.x - seg_start.position.x)
                robot_yaw = get_yaw(robot_pose_in_path_frame.pose.orientation)
                heading_tracking_error = shortest_angular_distance(robot_yaw, path_yaw)

            current_tracking_feedback.header = pose.header
            current_tracking_feedback.position_tracking_error = float(
                path_search_result.distance)
            current_tracking_feedback.heading_tracking_error = float(heading_tracking_error)
            current_tracking_feedback.current_path_index = int(
                path_search_result.closest_segment_index)
            current_tracking_feedback.robot_pose = pose
            current_tracking_feedback.distance_to_goal = float(current_distance_to_goal)
            current_tracking_feedback.speed = float(
                math.hypot(twist.linear.x, twist.linear.y))
            self._start_index = path_search_result.closest_segment_index
            current_tracking_feedback.remaining_path_length = float(
                calculate_path_length(self._current_path, self._start_index))

            if self._tracking_feedback_pub.get_subscription_count() > 0:
                self._tracking_feedback_pub.publish(current_tracking_feedback)

        feedback = FollowPath.Feedback()
        feedback.tracking_feedback = current_tracking_feedback
        goal_handle.publish_feedback(feedback)

    def _update_global_path(self, goal_handle) -> None:
        """
        Handle goal preemption; mirrors updateGlobalPath().

        rclpy action servers do not expose the C++ preempt/accept-pending-goal
        API: a new FollowPath goal starts a fresh execute callback instead.
        Kept as a structural no-op for parity with the C++ implementation.
        """
        pass

    def _publish_velocity(self, velocity: TwistStamped) -> None:
        """Validate and publish a velocity command; mirrors publishVelocity()."""
        t = velocity.twist
        vals = (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)
        if any(math.isnan(v) or math.isinf(v) for v in vals):
            self.get_logger().error(
                'Velocity message contains NaNs or Infs! Ignoring as invalid!')
            return
        if self._vel_publisher is not None and self._vel_publisher.get_subscription_count() > 0:
            self._vel_publisher.publish(velocity)

    def _publish_zero_velocity(self) -> None:
        """Publish a zero velocity command; mirrors publishZeroVelocity()."""
        velocity = TwistStamped()
        if self._costmap_ros is not None:
            velocity.header.frame_id = self._costmap_ros.get_base_frame_id()
        velocity.header.stamp = self.get_clock().now().to_msg()
        self._publish_velocity(velocity)

    def _on_goal_exit(self, force_stop: bool) -> None:
        """Reset state on goal exit; mirrors onGoalExit()."""
        if self._params.publish_zero_velocity or force_stop:
            self._publish_zero_velocity()
        for controller in self._controllers.values():
            controller.reset()

    def _is_goal_reached(self) -> bool:
        """Check if the goal is reached; mirrors isGoalReached()."""
        pose = self._get_robot_pose()
        if pose is None:
            return False
        velocity = self._get_thresholded_twist(self._get_raw_twist())
        return self._goal_checkers[self._current_goal_checker].is_goal_reached(
            pose.pose, self._transformed_end_pose.pose, velocity, self._transformed_global_plan)

    def _get_robot_pose(self) -> Optional[PoseStamped]:
        """Obtain the current robot pose in the costmap frame; mirrors getRobotPose()."""
        if self._costmap_ros is None:
            return None
        return self._costmap_ros.get_robot_pose()

    # ------------------------------------------------------------------
    # Velocity helpers
    # ------------------------------------------------------------------

    def _get_raw_twist(self) -> Twist:
        with self._odom_lock:
            twist = Twist()
            twist.linear.x = self._odom_twist.linear.x
            twist.linear.y = self._odom_twist.linear.y
            twist.linear.z = self._odom_twist.linear.z
            twist.angular.x = self._odom_twist.angular.x
            twist.angular.y = self._odom_twist.angular.y
            twist.angular.z = self._odom_twist.angular.z
            return twist

    @staticmethod
    def _get_thresholded_velocity(velocity: float, threshold: float) -> float:
        return velocity if abs(velocity) > threshold else 0.0

    def _get_thresholded_twist(self, twist: Twist) -> Twist:
        out = Twist()
        out.linear.x = self._get_thresholded_velocity(
            twist.linear.x, self._params.min_x_velocity_threshold)
        out.linear.y = self._get_thresholded_velocity(
            twist.linear.y, self._params.min_y_velocity_threshold)
        out.angular.z = self._get_thresholded_velocity(
            twist.angular.z, self._params.min_theta_velocity_threshold)
        return out

    # ------------------------------------------------------------------
    # Plugin id lookup helpers (mirror findControllerId etc.)
    # ------------------------------------------------------------------

    def _find_controller_id(self, c_name: str) -> Optional[str]:
        if c_name not in self._controllers:
            if len(self._controllers) == 1 and not c_name:
                self.get_logger().warning(
                    'No controller was specified in action call. Server will use only plugin '
                    f'loaded {self._controller_ids_concat}. This warning will appear once.')
                return next(iter(self._controllers))
            self.get_logger().error(
                f'FollowPath called with controller name {c_name}, which does not exist. '
                f'Available controllers are: {self._controller_ids_concat}.')
            return None
        self.get_logger().debug(f'Selected controller: {c_name}.')
        return c_name

    def _find_goal_checker_id(self, c_name: str) -> Optional[str]:
        if c_name not in self._goal_checkers:
            if len(self._goal_checkers) == 1 and not c_name:
                self.get_logger().warning(
                    "No goal checker was specified in parameter 'current_goal_checker'. Server "
                    f'will use only plugin loaded {self._goal_checker_ids_concat}. This warning '
                    'will appear once.')
                return next(iter(self._goal_checkers))
            self.get_logger().error(
                f'FollowPath called with goal_checker name {c_name} in parameter '
                "'current_goal_checker', which does not exist. Available goal checkers are: "
                f'{self._goal_checker_ids_concat}.')
            return None
        self.get_logger().debug(f'Selected goal checker: {c_name}.')
        return c_name

    def _find_progress_checker_id(self, c_name: str) -> Optional[str]:
        if not self._progress_checkers:
            if not c_name:
                self.get_logger().debug(
                    'No progress checker configured and none requested. Progress checking will '
                    'be bypassed.')
                return ''
            self.get_logger().error(
                f'FollowPath called with progress_checker name {c_name} in parameter '
                "'current_progress_checker', but no progress checkers are configured.")
            return None
        if c_name not in self._progress_checkers:
            if len(self._progress_checkers) == 1 and not c_name:
                self.get_logger().warning(
                    "No progress checker was specified in parameter 'current_progress_checker'. "
                    f'Server will use only plugin loaded {self._progress_checker_ids_concat}. '
                    'This warning will appear once.')
                return next(iter(self._progress_checkers))
            self.get_logger().error(
                f'FollowPath called with progress_checker name {c_name} in parameter '
                "'current_progress_checker', which does not exist. Available progress checkers "
                f'are: {self._progress_checker_ids_concat}.')
            return None
        self.get_logger().debug(f'Selected progress checker: {c_name}.')
        return c_name

    def _find_path_handler_id(self, c_name: str) -> Optional[str]:
        if c_name not in self._path_handlers:
            if len(self._path_handlers) == 1 and not c_name:
                self.get_logger().warning(
                    "No path handler was specified in parameter 'current_path_handler'. Server "
                    f'will use only plugin loaded {self._path_handler_ids_concat}. This warning '
                    'will appear once.')
                return next(iter(self._path_handlers))
            self.get_logger().error(
                f'FollowPath called with path_handler name {c_name} in parameter '
                "'current_path_handler', which does not exist. Available path handlers are: "
                f'{self._path_handler_ids_concat}.')
            return None
        self.get_logger().debug(f'Selected path handler: {c_name}.')
        return c_name

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry) -> None:
        with self._odom_lock:
            self._odom_twist = msg.twist.twist

    def _speed_limit_callback(self, msg: SpeedLimit) -> None:
        for controller in self._controllers.values():
            controller.set_speed_limit(msg.speed_limit, msg.percentage)


def main(args=None):
    """Run the Nav2 Controller Server node."""
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    controller_server = ControllerServer()
    executor.add_node(controller_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
