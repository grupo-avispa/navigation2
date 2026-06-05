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
Regulated Pure Pursuit controller plugin for nav2_controller_py.

Simplified but robust implementation based on nav2_regulated_pure_pursuit_controller.

Plugin type string (use in YAML):
    FollowPath:
      plugin: "nav2_controller_example_py.PurePursuitController"
"""

import math
from typing import Optional, Tuple

from geometry_msgs.msg import PointStamped, PoseStamped, Twist, TwistStamped
from nav2_core_py.controller import Controller
from nav2_core_py.controller_exceptions import InvalidPath, NoValidControl
from nav_msgs.msg import Path


class PurePursuitController(Controller):
    """
    Regulated Pure Pursuit local controller.

    Parameters (under ``<plugin_id>.`` namespace in YAML):
        lookahead_dist          (float, default 2.0 m)
        max_linear_vel          (float, default 0.5 m/s)
        min_linear_vel          (float, default 0.05 m/s)
        max_angular_vel         (float, default 1.0 rad/s)
        max_linear_accel        (float, default 2.5 m/s²)
        max_linear_decel        (float, default 2.5 m/s²)
        rotation_scaling_factor (float, default 1.0)
    """

    def __init__(self):
        self._node = None
        self._name = ''

        # Parameters
        self._lookahead_dist = 2.0
        self._max_linear_vel = 0.5
        self._min_linear_vel = 0.05
        self._max_angular_vel = 1.0
        self._max_linear_accel = 2.5
        self._max_linear_decel = 2.5
        self._rotation_scaling_factor = 1.0

        # Speed limit
        self._speed_limit = 1.0
        self._speed_limit_absolute = False

        # State
        self._path: Optional[Path] = None
        self._cancelled: bool = False
        self._last_linear_vel = 0.0

        # Publishers
        self._target_pub = None
        self._path_pub = None

    def configure(self, node, name: str, tf_buffer, costmap_ros) -> None:
        self._node = node
        self._name = name

        def _p(param, default):
            full = f'{name}.{param}'
            if not node.has_parameter(full):
                node.declare_parameter(full, default)
            return node.get_parameter(full).value

        self._lookahead_dist = _p('lookahead_dist', 2.0)
        self._max_linear_vel = _p('max_linear_vel', 0.5)
        self._min_linear_vel = _p('min_linear_vel', 0.05)
        self._max_angular_vel = _p('max_angular_vel', 1.0)
        self._max_linear_accel = _p('max_linear_accel', 2.5)
        self._max_linear_decel = _p('max_linear_decel', 2.5)
        self._rotation_scaling_factor = _p('rotation_scaling_factor', 1.0)

        # Publishers
        self._target_pub = node.create_publisher(PointStamped, f'{name}/target_point', 10)
        self._path_pub = node.create_publisher(Path, f'{name}/local_plan', 10)

        node.get_logger().info(
            f'[RegulatedPurePursuitController] "{name}" configured: '
            f'lookahead={self._lookahead_dist:.1f}m '
            f'v=[{self._min_linear_vel:.2f}, {self._max_linear_vel:.2f}]m/s'
        )

    def cleanup(self) -> None:
        self._path = None

    def activate(self) -> None:
        self._node.get_logger().info(
            f'[RegulatedPurePursuitController] "{self._name}" activated')
        self._cancelled = False
        self._last_linear_vel = 0.0

    def deactivate(self) -> None:
        self._node.get_logger().info(
            f'[RegulatedPurePursuitController] "{self._name}" deactivated')

    def new_path_received(self, path: Path) -> None:
        if not path.poses:
            raise InvalidPath('Received an empty path.')
        self._path = path

    def reset(self) -> None:
        self._path = None
        self._cancelled = False
        self._last_linear_vel = 0.0

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

    def compute_velocity_commands(
        self,
        pose: PoseStamped,
        velocity: Twist,
        goal_checker,
        transformed_global_plan: Path,
        goal_pose: PoseStamped,
    ) -> TwistStamped:
        """Compute regulated pure pursuit velocity commands."""
        if self._cancelled:
            cmd = TwistStamped()
            cmd.header.stamp = self._node.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            return cmd

        plan = transformed_global_plan if transformed_global_plan.poses else self._path

        if plan is None or not plan.poses:
            raise NoValidControl('[RegulatedPurePursuitController] No path available.')

        # Get lookahead point using path distance
        try:
            lookahead_pose = self._get_lookahead_point(
                plan,
                pose.pose.position.x,
                pose.pose.position.y
            )
        except Exception:
            raise NoValidControl(
                '[RegulatedPurePursuitController] Could not find lookahead point.'
            )

        # Publish target
        if self._target_pub:
            target_msg = PointStamped()
            target_msg.header = plan.header
            target_msg.point.x = lookahead_pose.pose.position.x
            target_msg.point.y = lookahead_pose.pose.position.y
            target_msg.point.z = 0.01
            self._target_pub.publish(target_msg)

        # Transform to robot frame
        lx, ly = self._transform_to_robot_frame(pose, lookahead_pose)
        dist = math.sqrt(lx ** 2 + ly ** 2)

        if dist < 0.01:
            raise NoValidControl(
                '[RegulatedPurePursuitController] Lookahead point too close.')

        # Calculate curvature
        curvature = self._calculate_curvature(lx, ly, dist)

        # Distance to goal
        goal_x = plan.poses[-1].pose.position.x
        goal_y = plan.poses[-1].pose.position.y
        dist_to_goal = math.sqrt(
            (goal_x - pose.pose.position.x) ** 2 +
            (goal_y - pose.pose.position.y) ** 2
        )

        # Regulation: reduce velocity based on path curvature and distance to goal
        linear_vel = self._regulate_velocity(curvature, dist_to_goal, velocity.linear.x)

        # Angular velocity from curvature
        angular_vel = curvature * linear_vel
        angular_vel = max(-self._max_angular_vel, min(self._max_angular_vel, angular_vel))

        # Build command
        cmd = TwistStamped()
        cmd.header.stamp = self._node.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.twist.linear.x = linear_vel
        cmd.twist.angular.z = angular_vel

        self._last_linear_vel = linear_vel

        # Publish trajectory from robot to target
        path_to_target = self._create_path_to_target(pose, lookahead_pose, plan.header.frame_id)
        if self._path_pub:
            self._path_pub.publish(path_to_target)

        return cmd

    def _get_lookahead_point(self, path: Path, rx: float, ry: float) -> PoseStamped:
        """Find lookahead point by walking path distance (not euclidean)."""
        poses = path.poses
        n = len(poses)

        if n == 0:
            raise ValueError('Empty path')

        # Find closest point
        closest_idx = 0
        min_dist = float('inf')
        for i in range(n):
            p = poses[i]
            d = math.sqrt((p.pose.position.x - rx) ** 2 + (p.pose.position.y - ry) ** 2)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # Walk forward accumulating distance
        path_dist = 0.0
        for i in range(closest_idx, n - 1):
            p1 = poses[i]
            p2 = poses[i + 1]

            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y
            seg_len = math.sqrt(dx ** 2 + dy ** 2)

            if seg_len == 0:
                continue

            if path_dist + seg_len >= self._lookahead_dist:
                # Interpolate within segment
                remaining = self._lookahead_dist - path_dist
                t = remaining / seg_len
                t = max(0.0, min(1.0, t))

                pose = PoseStamped()
                pose.header = p1.header
                pose.pose.position.x = p1.pose.position.x + t * dx
                pose.pose.position.y = p1.pose.position.y + t * dy
                pose.pose.orientation = p1.pose.orientation
                return pose

            path_dist += seg_len

        # Return last pose if lookahead distance exceeds path
        return poses[-1]

    @staticmethod
    def _transform_to_robot_frame(pose: PoseStamped, target: PoseStamped) -> Tuple[float, float]:
        """Transform point to robot frame."""
        dx = target.pose.position.x - pose.pose.position.x
        dy = target.pose.position.y - pose.pose.position.y

        # Get robot yaw
        q = pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Rotate to robot frame
        lx = math.cos(-robot_yaw) * dx - math.sin(-robot_yaw) * dy
        ly = math.sin(-robot_yaw) * dx + math.cos(-robot_yaw) * dy

        return lx, ly

    def _calculate_curvature(self, lx: float, ly: float, dist: float) -> float:
        """Calculate pure pursuit curvature: κ = 2*ly/dist²."""
        if dist < 0.001:
            return 0.0
        return 2.0 * ly / (dist ** 2)

    def _create_path_to_target(
        self,
        pose: PoseStamped,
        target: PoseStamped,
        frame_id: str
    ) -> Path:
        """Create a path from robot to target with interpolated points."""
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self._node.get_clock().now().to_msg()

        robot_x = pose.pose.position.x
        robot_y = pose.pose.position.y
        target_x = target.pose.position.x
        target_y = target.pose.position.y

        # Create 20 interpolated points from robot to target
        num_points = 20
        for i in range(num_points + 1):
            t = i / num_points

            point_pose = PoseStamped()
            point_pose.header = pose.header
            point_pose.pose.position.x = robot_x + t * (target_x - robot_x)
            point_pose.pose.position.y = robot_y + t * (target_y - robot_y)
            point_pose.pose.orientation = pose.pose.orientation

            path.poses.append(point_pose)

        return path

    def _regulate_velocity(
        self,
        curvature: float,
        dist_to_goal: float,
        current_vel: float
    ) -> float:
        """Regulate velocity based on curvature and distance to goal."""
        # Apply speed limit
        v_max = self._max_linear_vel
        if self._speed_limit_absolute:
            v_max = min(v_max, self._speed_limit)
        else:
            v_max = v_max * self._speed_limit

        # Reduce speed near high curvature
        curvature_factor = 1.0 / (1.0 + abs(curvature) * self._rotation_scaling_factor)
        v_max *= curvature_factor

        # Reduce speed near goal
        if dist_to_goal < self._lookahead_dist * 2.0:
            v_max *= dist_to_goal / (self._lookahead_dist * 2.0)

        v_linear = v_max
        v_linear = max(self._min_linear_vel, v_linear)

        # Apply acceleration constraints
        dt = 0.1  # Assume ~10Hz control rate
        max_accel = self._max_linear_accel * dt
        max_decel = self._max_linear_decel * dt

        if v_linear > current_vel + max_accel:
            v_linear = current_vel + max_accel
        elif v_linear < current_vel - max_decel:
            v_linear = current_vel - max_decel

        return v_linear
