# Copyright (c) 2024 nav2_py_pure_pursuit
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
Pure Pursuit controller plugin for nav2_controller_py.

Implements the classic Pure Pursuit algorithm:
  1. Find the lookahead point on the global path.
  2. Compute curvature to drive the robot toward that point.
  3. Return a TwistStamped command.

This plugin is intentionally self-contained (no costmap required) so it
can be used as a starting template for more sophisticated controllers.

Plugin type string (use in YAML):
    FollowPath:
      plugin: "nav2_py_pure_pursuit/PurePursuitController"
"""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav2_core_py.controller import Controller
from nav2_core_py.controller_exceptions import InvalidPath, NoValidControl
from nav_msgs.msg import Path


class PurePursuitController(Controller):
    """
    Pure Pursuit local controller.

    Parameters (under ``<plugin_id>.`` namespace in YAML):
      lookahead_dist      (float, default 0.4 m)
      max_linear_vel      (float, default 0.5 m/s)
      min_linear_vel      (float, default 0.05 m/s)
      max_angular_vel     (float, default 1.0 rad/s)
      transform_tolerance (float, default 0.5 s)  – unused here, kept for parity

    The controller operates in the path frame without a full costmap:
      - It iterates path poses and picks the first one beyond ``lookahead_dist``.
      - It computes a curvature command to steer toward that waypoint.
      - Linear velocity is reduced near the goal (proportional control).
    """

    def __init__(self):
        self._node = None
        self._name = ''

        # Parameters
        self._lookahead_dist = 0.4
        self._max_linear_vel = 0.5
        self._min_linear_vel = 0.05
        self._max_angular_vel = 1.0
        self._transform_tolerance = 0.5

        # Speed limit (set via set_speed_limit)
        self._speed_limit = 1.0        # fraction of max (1.0 = unlimited)
        self._speed_limit_absolute = False

        # Current path
        self._path: Optional[Path] = None
        self._path_index: int = 0
        self._cancelled: bool = False

    # ------------------------------------------------------------------
    # ControllerBase interface
    # ------------------------------------------------------------------

    def configure(self, node, name: str, tf_buffer, costmap_ros) -> None:
        self._node = node
        self._name = name

        def _p(param, default):
            full = f'{name}.{param}'
            if not node.has_parameter(full):
                node.declare_parameter(full, default)
            return node.get_parameter(full).value

        self._lookahead_dist = _p('lookahead_dist', 0.4)
        self._max_linear_vel = _p('max_linear_vel', 0.5)
        self._min_linear_vel = _p('min_linear_vel', 0.05)
        self._max_angular_vel = _p('max_angular_vel', 1.0)
        self._transform_tolerance = _p('transform_tolerance', 0.5)

        node.get_logger().info(
            f'[PurePursuitController] "{name}" configured: '
            f'lookahead={self._lookahead_dist:.2f} m  '
            f'v=[{self._min_linear_vel:.2f}, {self._max_linear_vel:.2f}] m/s  '
            f'w_max={self._max_angular_vel:.2f} rad/s'
        )

    def cleanup(self) -> None:
        self._path = None

    def activate(self) -> None:
        self._node.get_logger().info(
            f'[PurePursuitController] "{self._name}" activated')
        self._cancelled = False

    def deactivate(self) -> None:
        self._node.get_logger().info(
            f'[PurePursuitController] "{self._name}" deactivated')

    def new_path_received(self, path: Path) -> None:
        """Store the new global path and reset the path index."""
        if not path.poses:
            raise InvalidPath('Received an empty path.')
        self._path = path
        self._path_index = 0
        self._node.get_logger().debug(
            f'[PurePursuitController] "{self._name}" received path '
            f'with {len(path.poses)} poses.'
        )

    def reset(self) -> None:
        self._path = None
        self._path_index = 0
        self._cancelled = False

    def cancel(self) -> bool:
        self._cancelled = True
        return True

    def set_speed_limit(self, speed_limit: float, percentage: bool) -> None:
        if percentage:
            self._speed_limit = speed_limit / 100.0
            self._speed_limit_absolute = False
        else:
            self._speed_limit = speed_limit
            self._speed_limit_absolute = True
        self._node.get_logger().info(
            f'[PurePursuitController] "{self._name}" speed limit set to '
            f'{speed_limit} ({"%" if percentage else "m/s"})'
        )

    def compute_velocity_commands(
        self,
        pose: PoseStamped,
        velocity: Twist,
        goal_checker,
        transformed_global_plan: Path,
        goal_pose: PoseStamped,
    ) -> TwistStamped:
        """
        Compute Pure Pursuit velocity commands.

        Parameters
        ----------
        pose : PoseStamped
            Current robot pose (in global frame).
        velocity : Twist
            Current (thresholded) robot velocity.
        goal_checker : GoalCheckerBase
            Active goal checker (not used here directly).
        transformed_global_plan : Path
            Global plan already transformed to local frame (or raw if no TF).
        goal_pose : PoseStamped
            Goal pose (last pose in path).

        Returns
        -------
        TwistStamped
            Velocity command.

        Raises
        ------
        NoValidControl
            If no lookahead point can be found.

        """
        if self._cancelled:
            cmd = TwistStamped()
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            return cmd

        plan = transformed_global_plan if transformed_global_plan.poses \
            else self._path

        if plan is None or not plan.poses:
            raise NoValidControl(
                f'[PurePursuitController] "{self._name}": no path available.'
            )

        # Robot position (use path frame; in pure Python we assume poses
        # are already expressed relative to robot if no TF is provided).
        rx = pose.pose.position.x
        ry = pose.pose.position.y
        robot_yaw = self._yaw_from_pose(pose.pose)

        # Find lookahead point
        lookahead_pose = self._find_lookahead_point(plan, rx, ry)
        if lookahead_pose is None:
            # All path points are behind us or path is exhausted → head to goal
            lookahead_pose = plan.poses[-1]

        lx = lookahead_pose.pose.position.x
        ly = lookahead_pose.pose.position.y

        # Transform lookahead point to robot frame
        dx = lx - rx
        dy = ly - ry
        dist_to_goal = math.sqrt(
            (plan.poses[-1].pose.position.x - rx) ** 2
            + (plan.poses[-1].pose.position.y - ry) ** 2
        )

        # Lookahead in robot frame
        lx_robot = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
        ly_robot = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy

        dist = math.sqrt(lx_robot ** 2 + ly_robot ** 2)
        if dist < 1e-6:
            raise NoValidControl(
                f'[PurePursuitController] "{self._name}": '
                f'lookahead point too close (dist={dist:.4f}).'
            )

        # Pure Pursuit curvature: κ = 2·ly / dist²
        curvature = 2.0 * ly_robot / (dist ** 2)

        # Effective max velocity (honour speed limit)
        v_max = self._max_linear_vel
        if self._speed_limit_absolute:
            v_max = min(v_max, self._speed_limit)
        else:
            v_max = v_max * self._speed_limit

        # Reduce speed near goal (proportional)
        v_linear = v_max * min(1.0, dist_to_goal /
                               (self._lookahead_dist * 3.0))
        v_linear = max(self._min_linear_vel, v_linear)

        # Angular velocity from curvature
        v_angular = curvature * v_linear
        v_angular = max(-self._max_angular_vel,
                        min(self._max_angular_vel, v_angular))

        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = v_linear
        cmd.twist.angular.z = v_angular

        self._node.get_logger().debug(
            f'[PurePursuitController] "{self._name}": '
            f'v={v_linear:.3f} m/s  w={v_angular:.3f} rad/s  '
            f'd_goal={dist_to_goal:.3f} m  κ={curvature:.4f}'
        )

        return cmd

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _find_lookahead_point(
        self,
        path: Path,
        rx: float,
        ry: float,
    ) -> Optional[PoseStamped]:
        """
        Return the first path pose at distance >= lookahead_dist from (rx, ry).

        Advances ``_path_index`` so we prune already-passed points.
        """
        poses = path.poses
        n = len(poses)
        if n == 0:
            return None

        # Advance index past poses that are closer than lookahead
        while self._path_index < n - 1:
            p = poses[self._path_index]
            d = math.sqrt(
                (p.pose.position.x - rx) ** 2
                + (p.pose.position.y - ry) ** 2
            )
            if d >= self._lookahead_dist:
                break
            self._path_index += 1

        return poses[self._path_index]

    @staticmethod
    def _yaw_from_pose(pose) -> float:
        """Extract yaw from a geometry_msgs.msg.Pose."""
        q = pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
