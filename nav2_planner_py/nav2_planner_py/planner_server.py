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
Nav2 Planner Server.

It mirrors the nav2_planner::PlannerServer from the C++ implementation,
using rclpy LifecycleNode, action_msgs, and the nav2_msgs action interfaces.

Plugins (GlobalPlanner) must be Python classes registered via a plugin_provider
(see plugin_provider.py) that implement the GlobalPlanner interface.

Action servers:
    - ComputePathToPose   (/compute_path_to_pose)
    - ComputePathThroughPoses (/compute_path_through_poses)
"""

import math
import threading
import time
import traceback
from typing import Any, Dict, List, Optional

from bondpy import bondpy
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
import tf2_ros
from nav2_costmap_2d_py import Costmap2DROS
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, ComputePathThroughPoses
from nav_msgs.msg import Path

from nav2_core_py.planner_exceptions import (  # noqa: F401 (re-exported)
    PlannerException,
    InvalidPlanner,
    StartOccupied,
    GoalOccupied,
    NoValidPathCouldBeFound,
    PlannerTimedOut,
    StartOutsideMapBounds,
    GoalOutsideMapBounds,
    TFError,
    NoViableRoute,
    PlannerTFError,
    NoViapointsGiven,
    PlannerCancelled,
)
from nav2_core_py.global_planner import GlobalPlanner
from nav2_core_py.plugin_provider import PluginProvider
from nav2_planner_py.parameter_handler import ParameterHandler, Parameters
from nav2_planner_py.is_path_valid_service import IsPathValidService


# ---------------------------------------------------------------------------
# PlannerServer
# ---------------------------------------------------------------------------

class PlannerServer(LifecycleNode):
    """
    ROS2 Lifecycle Node implementing the Nav2 Planner Server.

    It hosts:
      - A map of GlobalPlanner plugin instances.
      - Two action servers:
          * ComputePathToPose   (/compute_path_to_pose)
          * ComputePathThroughPoses (/compute_path_through_poses)
      - A global costmap (nav2_costmap_2d::Costmap2DROS).

    Lifecycle transitions:
      on_configure  → load plugins, create action servers (inactive)
      on_activate   → activate plugins and costmap, activate action servers
      on_deactivate → deactivate plugins and costmap
      on_cleanup    → destroy action servers, cleanup plugins
      on_shutdown   → shutdown
    """

    def __init__(self) -> None:
        super().__init__('nav2_planner')

        # Plugin registry  {planner_id: GlobalPlanner}
        self._planners: Dict[str, GlobalPlanner] = {}
        # Concatenated planner IDs string for diagnostics
        self._planner_ids_concat: str = ''
        self._plugin_provider = PluginProvider(
            export_tag='nav2_planner',
            base_class_type='nav2_core_py.global_planner.GlobalPlanner',
        )
        # Pre-scan the ament resource index for plugins
        # to warm the provider's cache and speed up on_configure.
        self._plugin_provider.discover(self)

        # Parameter handler (manages all parameters)
        self._param_handler: Optional[ParameterHandler] = None
        self._params: Optional[Parameters] = None

        # Action servers
        self._action_server_pose: Optional[ActionServer] = None
        self._action_server_poses: Optional[ActionServer] = None

        # TF / costmap interfaces
        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._costmap_ros: Optional[Costmap2DROS] = None
        self._costmap_executor: Optional[rclpy.executors.SingleThreadedExecutor] = None
        self._costmap_thread: Optional[threading.Thread] = None
        self._costmap_activated: threading.Event = threading.Event()
        self._costmap_activate_thread: Optional[threading.Thread] = None

        # Callback group for action servers
        self._callback_group = ReentrantCallbackGroup()

        # Plan publisher
        self._plan_publisher: Optional[Any] = None

        # Service to determine if a path is valid
        self._is_path_valid_service: Optional[IsPathValidService] = None

        # Bond to lifecycle manager
        self._bond: Optional[bondpy.Bond] = None
        self._bond_heartbeat_period: float = 0.1

        self.get_logger().info('Creating nav2_planner')

    # ------------------------------------------------------------------
    # Lifecycle callbacks
    # ------------------------------------------------------------------

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure: load TF, costmap and plugins synchronously."""
        return self._on_configure_impl(state)

    def _on_configure_impl(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Internal configure implementation."""
        self.get_logger().info('Configuring')

        # --- Global Costmap --------------------------------------------
        try:
            self._costmap_ros = Costmap2DROS(
                name='global_costmap',
                parent_namespace=str(self.get_namespace()),
                use_sim_time=self.get_parameter_or('use_sim_time', rclpy.parameter.Parameter(
                    'use_sim_time', rclpy.Parameter.Type.BOOL, False)).value,
            )
            # Configure the costmap before spinning its executor so that the
            # lifecycle_manager cannot race-configure it via the lifecycle
            # service while on_configure(None) is still running.
            result = self._costmap_ros.on_configure(None)  # type: ignore[arg-type]
            if result != TransitionCallbackReturn.SUCCESS:
                raise RuntimeError('Failed to configure costmap')
            # Only AFTER configure succeeds, start the dedicated executor so
            # that TF/topic subscriptions are served during on_activate and
            # steady-state operation.
            self._costmap_executor = rclpy.executors.SingleThreadedExecutor()
            self._costmap_executor.add_node(self._costmap_ros)
            self._costmap_thread = threading.Thread(
                target=self._costmap_executor.spin,
                daemon=True,
                name='costmap_executor',
            )
            self._costmap_thread.start()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().fatal(
                f'Failed to initialize Costmap2DROS: {ex}.')
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

        # --- TF buffer -------------------------------------------------
        try:
            if self._costmap_ros is not None:
                self._tf_buffer = self._costmap_ros.get_tf_buffer()
        except Exception as ex:  # noqa: BLE001
            self.get_logger().fatal(
                f'Failed to get TF buffer from costmap: {ex}')
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

        # --- Initialize parameter handler -----------------------------------
        try:
            self._param_handler = ParameterHandler(self)
        except RuntimeError as ex:
            self.get_logger().fatal(
                f'Failed to initialize parameter handler: {ex}\n'
                + traceback.format_exc()
            )
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

        self._params = self._param_handler.get_parameters()

        # --- Planner plugins ------------------------------------------
        # Use pre-discovered plugins from __init__ if available (warm cache),
        # otherwise do a full scan (e.g. first call after cleanup reset).
        self.get_logger().info(f'[{self.get_name()}] Discovering plugins...')
        cached = self._plugin_provider.get_plugin_descriptors()
        if cached:
            plugin_descriptors = list(cached.values())
        else:
            plugin_descriptors = self._plugin_provider.discover(self)
        if not plugin_descriptors:
            self.get_logger().warning(f'[{self.get_name()}] No plugins found!')

        if self._params.planner_ids:
            for i, planner_id in enumerate(self._params.planner_ids):
                try:
                    plugin_type = self._params.planner_types[i]

                    # Load plugin instance via provider
                    planner = self._plugin_provider.load(plugin_type, self)

                    if planner is None:
                        self.get_logger().fatal(
                            f'[{self.get_name()}] Failed to create '
                            f"planner: '{planner_id}' of type '{plugin_type}'"
                        )
                        self.on_cleanup(state)
                        return TransitionCallbackReturn.FAILURE

                    planner.configure(
                        self, planner_id, self._tf_buffer, self._costmap_ros
                    )
                    self._planners[planner_id] = planner
                    self.get_logger().info(
                        f'[{self.get_name()}] Created planner: '
                        f"'{planner_id}' of type '{plugin_type}'"
                    )
                except Exception as ex:  # noqa: BLE001
                    self.get_logger().fatal(
                        f"[{self.get_name()}] Failed to create planner "
                        f"'{planner_id}': {type(ex).__name__}: {ex}\n"
                        f'{traceback.format_exc()}'
                    )
                    self.on_cleanup(state)
                    return TransitionCallbackReturn.FAILURE
        else:
            self.get_logger().warning(
                f'[{self.get_name()}] No planner_plugins specified. '
                'No planners will be loaded.'
            )

        # Build planner_ids_concat for diagnostics
        planner_ids_concat = ', '.join(
            f'{pid}=[{ptype}]'
            for pid, ptype in zip(self._params.planner_ids, self._params.planner_types)
        )
        self._planner_ids_concat = ' '.join(self._params.planner_ids)
        self.get_logger().info(
            f'Planner Server has {len(self._planners)} planners available. '
            f'{planner_ids_concat}'
        )

        # Initialize plan publisher, services and action servers
        try:
            self._plan_publisher = self.create_publisher(Path, 'plan', 10)
            self.get_logger().debug('Publisher created')

            self._is_path_valid_service = IsPathValidService(
                self, self._costmap_ros, self._params.costmap_update_timeout)
            self.get_logger().debug('IsPathValidService created')

            # --- Action servers -------------------------------------------
            self._action_server_pose = ActionServer(
                self,
                ComputePathToPose,
                'compute_path_to_pose',
                execute_callback=self._compute_path_to_pose_callback,
                callback_group=self._callback_group,
            )
            self.get_logger().debug('ActionServer ComputePathToPose created')

            self._action_server_poses = ActionServer(
                self,
                ComputePathThroughPoses,
                'compute_path_through_poses',
                execute_callback=self._compute_path_through_poses_callback,
                callback_group=self._callback_group,
            )
            self.get_logger().debug('ActionServer ComputePathThroughPoses created')

        except Exception as ex:  # noqa: BLE001
            self.get_logger().fatal(
                f'[on_configure] Failed creating publisher/service/action-server: '
                f'{type(ex).__name__}: {ex}\n{traceback.format_exc()}'
            )
            self.on_cleanup(state)
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info('Configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate costmap and all planner plugins."""
        self.get_logger().info('Activating')
        self._costmap_activated.clear()

        self.create_bond()

        self._costmap_activate_thread = threading.Thread(
            target=self._activate_background,
            args=(state,),
            daemon=True,
            name='planner_activate',
        )
        self._costmap_activate_thread.start()
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
                    self.get_logger().error(
                        'Background costmap activation failed; planning will be unavailable.'
                    )
            else:
                self._costmap_activated.set()

            for _, planner in self._planners.items():
                planner.activate()

            if self._is_path_valid_service:
                self._is_path_valid_service.initialize()

            self.get_logger().info('Activated (background)')
        except Exception:  # noqa: BLE001
            self.get_logger().fatal(
                f'Background activation failed:\n{traceback.format_exc()}'
            )

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate all planner plugins and costmap."""
        self.get_logger().info('Deactivating')

        # Wait for background activate thread to settle before touching
        # shared objects (planners, costmap).
        if self._costmap_activate_thread is not None and self._costmap_activate_thread.is_alive():
            self.get_logger().info('Waiting for background activation to complete...')
            self._costmap_activate_thread.join(timeout=10.0)

        for _, planner in self._planners.items():
            planner.deactivate()

        if self._costmap_ros is not None:
            result = self._costmap_ros.on_deactivate(state)
            if result != TransitionCallbackReturn.SUCCESS:
                self.get_logger().warning('Failed to deactivate costmap')

        if self._is_path_valid_service:
            self._is_path_valid_service.reset()

        self.destroy_bond()

        self.get_logger().info('Deactivated')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup plugins, action servers, costmap."""
        self.get_logger().info('Cleaning up')

        for _, planner in self._planners.items():
            planner.cleanup()
        self._planners.clear()

        if self._action_server_pose is not None:
            self._action_server_pose.destroy()
            self._action_server_pose = None

        if self._action_server_poses is not None:
            self._action_server_poses.destroy()
            self._action_server_poses = None

        if self._costmap_activate_thread is not None \
                and self._costmap_activate_thread.is_alive() \
                and threading.current_thread() is not self._costmap_activate_thread:
            self._costmap_activate_thread.join(timeout=5.0)
        self._costmap_activate_thread = None
        self._costmap_activated.clear()

        if self._costmap_ros is not None:
            result = self._costmap_ros.on_cleanup(state)
            if result != TransitionCallbackReturn.SUCCESS:
                self.get_logger().warning('Failed to cleanup costmap')
            self._costmap_ros = None
        if self._costmap_executor is not None:
            self._costmap_executor.shutdown(timeout_sec=2.0)
            self._costmap_executor = None
        if self._costmap_thread is not None:
            self._costmap_thread.join(timeout=2.0)
            self._costmap_thread = None

        self._tf_buffer = None
        self.get_logger().info('Cleaned up')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down')
        return TransitionCallbackReturn.SUCCESS

    def create_bond(self) -> None:
        """
        Create bond connection to lifecycle manager.

        """
        if self._bond_heartbeat_period > 0.0:
            self.get_logger().info(
                f'Creating bond ({self.get_name()}) to lifecycle manager.'
            )
            self._bond = bondpy.Bond(
                node=self,
                topic='bond',
                bond_id=self.get_name(),
            )
            self._bond.heartbeat_period = self._bond_heartbeat_period
            self._bond.heartbeat_timeout = 4.0
            self._bond.start()

    def destroy_bond(self) -> None:
        """
        Destroy bond connection to lifecycle manager.
        """
        if self._bond_heartbeat_period > 0.0:
            self.get_logger().info(
                f'Destroying bond ({self.get_name()}) to lifecycle manager.'
            )
            if self._bond is not None:
                self._bond.shutdown()
                self._bond = None

    # --- ComputePathToPose --------------------------------------------
    async def _compute_path_to_pose_callback(self, goal_handle):
        """Execute callback for ComputePathToPose action."""
        self.get_logger().info('Computing path to goal')
        result = ComputePathToPose.Result()
        start_time = self.get_clock().now()

        goal = goal_handle.request
        start_pose = None
        goal_pose = None

        try:
            # Lock parameter handler to safely access parameters
            with self._param_handler.get_mutex():
                params = self._param_handler.get_parameters()
                planner_id = goal.planner_id if goal.planner_id else params.planner_ids[0]
                max_duration = params.max_planner_duration

            # Check if action server is still active
            if not goal_handle.is_active:
                self.get_logger().debug('Action server inactive')
                return result

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal was canceled')
                goal_handle.canceled()
                return result

            # Wait for costmap to be current
            costmap_wait = self._wait_for_costmap()

            # Get start pose
            if goal.use_start:
                start_pose = goal.start
            elif self._costmap_ros:
                start_pose = self._costmap_ros.get_robot_pose()
                if start_pose is None:
                    raise PlannerTFError('Unable to get start pose')
            else:
                raise PlannerTFError('Unable to get start pose')

            # Transform poses to global frame
            goal_pose = goal.goal
            if not self._transform_poses_to_global_frame(start_pose, goal_pose):
                raise PlannerTFError(
                    'Unable to transform poses to global frame')

            # Create cancel checker
            def cancel_checker(): return goal_handle.is_cancel_requested

            # Get plan
            result.path = self._get_plan(
                start_pose,
                goal_pose,
                planner_id,
                list(goal.viapoints) if hasattr(goal, 'viapoints') else [],
                cancel_checker,
            )

            # Validate path
            if not result.path.poses:
                raise NoValidPathCouldBeFound(
                    f"Planning failed for planner '{planner_id}': No valid path found"
                )

            # Publish plan
            self._publish_plan(result.path)

            # Calculate planning time
            cycle_duration = self.get_clock().now() - start_time
            result.planning_time = self._build_duration_msg(
                cycle_duration.nanoseconds / 1e9
            )

            # Check if we missed the desired rate
            if max_duration > 0.0 and cycle_duration.nanoseconds / 1e9 > max_duration:
                wait_msg = (
                    f" Waited {costmap_wait:.2f}s for costmap update."
                    if costmap_wait > 0.0 else ""
                )
                self.get_logger().warning(
                    f'Planner loop missed its desired rate of '
                    f'{1.0 / max_duration:.4f} Hz. '
                    f'Current loop rate is {1.0 / (cycle_duration.nanoseconds / 1e9):.4f} Hz{wait_msg}'
                )

            goal_handle.succeed(result)
            self.get_logger().info('Path computed successfully')

        except InvalidPlanner as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.INVALID_PLANNER
            goal_handle.abort(result)
        except StartOccupied as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.START_OCCUPIED
            goal_handle.abort(result)
        except GoalOccupied as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.GOAL_OCCUPIED
            goal_handle.abort(result)
        except NoValidPathCouldBeFound as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.NO_VALID_PATH
            goal_handle.abort(result)
        except PlannerTimedOut as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.TIMEOUT
            goal_handle.abort(result)
        except StartOutsideMapBounds as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.START_OUTSIDE_MAP
            goal_handle.abort(result)
        except GoalOutsideMapBounds as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.GOAL_OUTSIDE_MAP
            goal_handle.abort(result)
        except PlannerTFError as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.TF_ERROR
            goal_handle.abort(result)
        except PlannerCancelled:
            result.error_msg = 'Goal was canceled. Canceling planning action.'
            self.get_logger().info(result.error_msg)
            goal_handle.canceled()
        except Exception as ex:  # noqa: BLE001
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathToPose.Result.UNKNOWN
            goal_handle.abort(result)

        return result

    # --- ComputePathThroughPoses --------------------------------------

    async def _compute_path_through_poses_callback(self, goal_handle):
        """
        Execute callback for ComputePathThroughPoses action.

        Concatenates paths between consecutive waypoints.
        """
        self.get_logger().info('Computing path through poses')
        result = ComputePathThroughPoses.Result()
        start_time = self.get_clock().now()

        goal = goal_handle.request
        start_pose = None
        goal_pose = None
        concat_path = None

        try:
            # Lock parameter handler to safely access parameters
            with self._param_handler.get_mutex():
                params = self._param_handler.get_parameters()
                planner_id = goal.planner_id if goal.planner_id else params.planner_ids[0]
                max_duration = params.max_planner_duration
                partial_plan_allowed = params.partial_plan_allowed

            # Check if action server is still active
            if not goal_handle.is_active:
                self.get_logger().debug('Action server inactive')
                return result

            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal was canceled')
                goal_handle.canceled()
                return result

            # Wait for costmap to be current
            costmap_wait = self._wait_for_costmap()

            # Validate goals list is not empty
            if not goal.goals:
                raise NoViapointsGiven('No goals provided')

            # Get start pose
            if goal.use_start:
                start_pose = goal.start
            elif self._costmap_ros:
                start_pose = self._costmap_ros.get_robot_pose()
                if start_pose is None:
                    raise PlannerTFError('Unable to get start pose')
            else:
                raise PlannerTFError('Unable to get start pose')

            # Create cancel checker
            def cancel_checker(): return goal_handle.is_cancel_requested

            # Initialize concatenated path
            concat_path = Path()

            # Get consecutive paths through waypoints
            for i, waypoint in enumerate(goal.goals):
                # Set current start
                if i == 0:
                    curr_start = start_pose
                else:
                    # Use end of last path as start for next one
                    curr_start = concat_path.poses[-1] if concat_path.poses else start_pose
                    curr_start.header = concat_path.header

                curr_goal = waypoint

                # Transform poses to global frame
                if not self._transform_poses_to_global_frame(curr_start, curr_goal):
                    raise PlannerTFError(
                        'Unable to transform poses to global frame')

                # Get plan from current start to current goal
                # ComputePathThroughPoses does not carry per-segment viapoints;
                # the waypoints array itself acts as the path skeleton.
                try:
                    curr_path = self._get_plan(
                        curr_start,
                        curr_goal,
                        planner_id,
                        [],
                        cancel_checker,
                    )
                except PlannerException as ex:
                    if i == 0 or not partial_plan_allowed:
                        raise
                    # Log warning and continue with partial path
                    self._exception_warning(
                        curr_start, curr_goal, planner_id, ex, result)
                    self.get_logger().warning(
                        f'Planner server failed to compute full path at waypoint {i}. '
                        'Outputting partial path instead.'
                    )
                    result.last_reached_index = i - 1
                    break

                # Validate path
                if not curr_path.poses:
                    exception = NoValidPathCouldBeFound(
                        f"Planning failed for planner '{planner_id}' at waypoint {i}: No valid path found"
                    )
                    if i == 0 or not partial_plan_allowed:
                        raise exception
                    # Log warning and continue with partial path
                    self._exception_warning(
                        curr_start, curr_goal, planner_id, exception, result)
                    self.get_logger().warning(
                        f'Planner server failed to compute full path at waypoint {i}. '
                        'Outputting partial path instead.'
                    )
                    result.last_reached_index = i - 1
                    break

                # Concatenate paths (skip first pose of subsequent paths to avoid duplication)
                if i == 0:
                    concat_path.poses.extend(curr_path.poses)
                elif len(curr_path.poses) > 1:
                    concat_path.poses.extend(curr_path.poses[1:])

                concat_path.header = curr_path.header

                # Update last reached index
                if i == len(goal.goals) - 1:
                    result.last_reached_index = ComputePathThroughPoses.Result.ALL_GOALS
                else:
                    result.last_reached_index = i

            # Set result path
            result.path = concat_path

            # Publish plan
            self._publish_plan(result.path)

            # Calculate planning time
            cycle_duration = self.get_clock().now() - start_time
            result.planning_time = self._build_duration_msg(
                cycle_duration.nanoseconds / 1e9
            )

            # Check if we missed the desired rate
            if max_duration > 0.0 and cycle_duration.nanoseconds / 1e9 > max_duration:
                wait_msg = (
                    f" Waited {costmap_wait:.2f}s for costmap update."
                    if costmap_wait > 0.0 else ""
                )
                self.get_logger().warning(
                    f'Planner loop missed its desired rate of '
                    f'{1.0 / max_duration:.4f} Hz. '
                    f'Current loop rate is {1.0 / (cycle_duration.nanoseconds / 1e9):.4f} Hz{wait_msg}'
                )

            goal_handle.succeed(result)
            self.get_logger().info('Path through poses computed successfully')

        except InvalidPlanner as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.INVALID_PLANNER
            goal_handle.abort(result)
        except StartOccupied as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.START_OCCUPIED
            goal_handle.abort(result)
        except GoalOccupied as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.GOAL_OCCUPIED
            goal_handle.abort(result)
        except NoValidPathCouldBeFound as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.NO_VALID_PATH
            goal_handle.abort(result)
        except PlannerTimedOut as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.TIMEOUT
            goal_handle.abort(result)
        except StartOutsideMapBounds as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.START_OUTSIDE_MAP
            goal_handle.abort(result)
        except GoalOutsideMapBounds as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.GOAL_OUTSIDE_MAP
            goal_handle.abort(result)
        except PlannerTFError as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.TF_ERROR
            goal_handle.abort(result)
        except NoViapointsGiven as ex:
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.NO_VIAPOINTS_GIVEN
            goal_handle.abort(result)
        except PlannerCancelled:
            result.error_msg = 'Goal was canceled. Canceling planning action.'
            self.get_logger().info(result.error_msg)
            goal_handle.canceled()
        except Exception as ex:  # noqa: BLE001
            self._exception_warning(
                start_pose, goal_pose, goal.planner_id, ex, result)
            result.error_code = ComputePathThroughPoses.Result.UNKNOWN
            goal_handle.abort(result)

        return result

    # ------------------------------------------------------------------
    # Core planning logic
    # ------------------------------------------------------------------

    def _wait_for_costmap(self) -> float:
        """
        Wait for the costmap to become current.

        Returns
        -------
        float
            Time waited in seconds (0.0 if already current).

        Raises
        ------
        PlannerTimedOut
            If timeout is exceeded.
        """
        with self._param_handler.get_mutex():
            params = self._param_handler.get_parameters()
            timeout = params.costmap_update_timeout

        if self._costmap_ros and timeout.nanoseconds > 0:
            import time as time_module
            waiting_start = time_module.time()
            try:
                self._costmap_ros.wait_until_current(timeout)
                return time_module.time() - waiting_start
            except Exception as ex:
                raise PlannerTimedOut(f'Costmap update timeout: {ex}')

        return 0.0

    def _transform_poses_to_global_frame(
        self,
        start: PoseStamped,
        goal: PoseStamped,
    ) -> bool:
        """
        Transform start and goal poses to global frame.

        Parameters
        ----------
        start : PoseStamped
            Start pose to transform.
        goal : PoseStamped
            Goal pose to transform.

        Returns
        -------
        bool
            True if transformation successful, False otherwise.
        """
        if not self._costmap_ros:
            return False

        try:
            start_transformed = self._costmap_ros.transform_pose_to_global_frame(
                start)
            goal_transformed = self._costmap_ros.transform_pose_to_global_frame(
                goal)

            if start_transformed is None or goal_transformed is None:
                return False

            # Update in place
            start.pose = start_transformed.pose
            start.header = start_transformed.header
            goal.pose = goal_transformed.pose
            goal.header = goal_transformed.header

            return True
        except Exception as ex:
            self.get_logger().error(f'Failed to transform poses: {ex}')
            return False

    def _publish_plan(self, path: Path) -> None:
        """
        Publish the plan for visualization.

        Parameters
        ----------
        path : Path
            The path to publish.
        """
        if self._plan_publisher and path.poses:
            try:
                self._plan_publisher.publish(path)
            except Exception as ex:
                self.get_logger().warning(f'Failed to publish plan: {ex}')

    def _get_plan(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        planner_id: str,
        viapoints: List[PoseStamped],
        cancel_checker,
    ) -> Path:
        """
        Call the plugin to compute a path.

        Parameters
        ----------
        start : PoseStamped
            Start pose.
        goal : PoseStamped
            Goal pose.
        planner_id : str
            Which plugin to use. If empty and only one planner is loaded,
            that planner is used automatically.
        viapoints : List[PoseStamped]
            Intermediate viapoints the path should pass through.
        cancel_checker : callable
            Returns True if the action has been cancelled.

        Returns
        -------
        Path
            The computed path.
        """
        # Resolve planner: use the single available one when ID is omitted
        if planner_id not in self._planners:
            if not planner_id and len(self._planners) == 1:
                self.get_logger().warning(
                    'No planners specified in action call. '
                    f'Server will use only plugin {self._planner_ids_concat} in server. '
                    'This warning will appear once.'
                )
                planner_id = next(iter(self._planners))
            else:
                raise InvalidPlanner(
                    f"Planner '{planner_id}' is not loaded. "
                    f'Available planners: {self._planner_ids_concat}'
                )

        # Lock parameter handler to safely access parameters
        with self._param_handler.get_mutex():
            params = self._param_handler.get_parameters()
            max_planner_duration = params.max_planner_duration

        planner = self._planners[planner_id]

        t_start = time.monotonic()
        path = planner.create_plan(start, goal, viapoints, cancel_checker)
        elapsed = time.monotonic() - t_start

        if max_planner_duration > 0.0 and elapsed > max_planner_duration:
            self.get_logger().warning(
                f"Planner '{planner_id}' missed its time budget of "
                f'{max_planner_duration:.2f}s (took {elapsed:.2f}s)'
            )

        if not path.poses:
            raise NoValidPathCouldBeFound(
                f"Planner '{planner_id}' failed to produce a path."
            )

        # Stamp header
        path.header.frame_id = goal.header.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        return path

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_current_pose(self) -> Optional[PoseStamped]:
        """
        Look up the current robot pose from TF.

        Returns None if TF lookup fails.
        """
        if self._tf_buffer is None:
            self.get_logger().warning(
                'TF buffer is not available; cannot get current pose.'
            )
            return None

        try:
            import tf2_geometry_msgs  # noqa: F401 (registers the conversion)
            from geometry_msgs.msg import TransformStamped
            transform: TransformStamped = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as ex:  # noqa: BLE001
            self.get_logger().error(f'Could not get robot pose via TF: {ex}')
            return None

    def _exception_warning(
        self,
        start: Optional[PoseStamped],
        goal: Optional[PoseStamped],
        planner_id: str,
        ex: Exception,
        result: Optional[object] = None,
    ) -> None:
        """Log a standardised warning for planner exceptions, including yaw angles."""
        import io

        def _quat_to_yaw(q) -> float:
            """Convert a geometry_msgs Quaternion to yaw (rad)."""
            # Standard ZYX Euler extraction for yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny_cosp, cosy_cosp)

        ss = io.StringIO()

        if start and goal:
            sq = start.pose.orientation
            gq = goal.pose.orientation
            ss.write(f'{planner_id} plugin failed to plan from ')
            ss.write(
                f'({start.pose.position.x:.2f}, {start.pose.position.y:.2f}) ')
            ss.write(
                f'[q: ({sq.x:.2f}, {sq.y:.2f}, {sq.z:.2f}, {sq.w:.2f})] '
            )
            ss.write(f'(yaw: {_quat_to_yaw(sq):.2f}) ')
            ss.write(
                f'to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) ')
            ss.write(
                f'[q: ({gq.x:.2f}, {gq.y:.2f}, {gq.z:.2f}, {gq.w:.2f})] '
            )
            ss.write(f'(yaw: {_quat_to_yaw(gq):.2f}): ')
        else:
            ss.write(f'{planner_id} plugin failed: ')

        ss.write(f'"{ex}"')
        error_msg = ss.getvalue()

        if result is not None and hasattr(result, 'error_msg'):
            result.error_msg = error_msg

        self.get_logger().warning(error_msg)

    @staticmethod
    def _build_duration_msg(seconds: float) -> DurationMsg:
        """Convert float seconds to builtin_interfaces/Duration."""
        msg = DurationMsg()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds - msg.sec) * 1e9)
        return msg


def main(args=None):
    """Run the Nav2 Planner Server node."""
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor()
    planner_server = PlannerServer()
    executor.add_node(planner_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        planner_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
