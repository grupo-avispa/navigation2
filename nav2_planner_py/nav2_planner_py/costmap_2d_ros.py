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
This is a Python3 API for Costmap2D lifecycle node in the stack.

It mirrors the nav2_costmap_2d::Costmap2DROS from the C++ implementation.
Handles costmap updates, layer management, and ROS2 integration.
"""

import threading
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Point, Polygon, PolygonStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import Costmap
from rclpy.duration import Duration
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn, LifecycleState
import tf2_ros

from nav2_planner_py.costmap_2d import PyCostmap2D
from nav2_planner_py.occupancy_grid import PyOccupancyGrid


class Costmap2DROS(LifecycleNode):
    """
    A ROS wrapper for a 2D Costmap.

    Responsibilities:
        - Manage costmap layers and updates
        - Subscribe to sensor topics (PointCloud, LaserScan)
        - Track robot pose and transforms
        - Provide costmap access via PyCostmap2D
        - Handle lifecycle transitions
    """

    def __init__(
        self,
        name: str = 'costmap_2d_ros',
        parent_namespace: str = '/',
        use_sim_time: bool = False,
    ) -> None:
        """
        Initialize Costmap2DROS node.

        Parameters
        ----------
        name : str
            Name of the costmap ROS node.
        parent_namespace : str
            Absolute namespace of the parent node hosting this costmap.
        use_sim_time : bool
            Whether to use simulation time.
        """
        self._name = name
        self._parent_namespace = parent_namespace
        super().__init__(name)
        self._use_sim_time = use_sim_time

        # TF2 support
        self._tf_buffer: Optional[tf2_ros.Buffer] = None
        self._tf_listener: Optional[tf2_ros.TransformListener] = None

        # Frame IDs
        self._global_frame: str = 'map'
        self._robot_base_frame: str = 'base_link'

        # Costmap and layers
        self._costmap: Optional[PyCostmap2D] = None
        self._costmap_mutex = threading.Lock()
        self._layered_costmap: Optional[object] = None

        # Robot footprint
        self._padded_footprint: List[Point] = []
        self._unpadded_footprint: List[Point] = []
        self._footprint_padding: float = 0.01
        self._use_radius: bool = False
        self._robot_radius: float = 0.5

        # Parameters
        self._map_width_meters: int = 100
        self._map_height_meters: int = 100
        self._resolution: float = 0.05
        self._origin_x: float = 0.0
        self._origin_y: float = 0.0
        self._transform_tolerance: float = 0.3
        self._update_frequency: float = 10.0  # Hz
        self._publish_frequency: float = 0.0  # Hz

        # Threading
        self._map_update_thread: Optional[threading.Thread] = None
        self._stop_updates = False
        self._initialized = False
        self._stopped = True

        # Subscriptions
        self._costmap_sub: Optional[object] = None

        self.get_logger().info(f'Initialized {self._name}')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Configure the costmap node.

        This transition loads parameters, declares subscriptions,
        and initializes the TF2 buffer.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS or FAILURE.
        """
        self.get_logger().info('Configuring costmap')

        try:
            # Declare parameters
            self.declare_parameter('global_frame', self._global_frame)
            self.declare_parameter('robot_base_frame', self._robot_base_frame)
            self.declare_parameter('width_meters', self._map_width_meters)
            self.declare_parameter('height_meters', self._map_height_meters)
            self.declare_parameter('resolution', self._resolution)
            self.declare_parameter('origin_x', self._origin_x)
            self.declare_parameter('origin_y', self._origin_y)
            self.declare_parameter(
                'transform_tolerance', self._transform_tolerance)
            self.declare_parameter('update_frequency', self._update_frequency)
            self.declare_parameter(
                'publish_frequency', self._publish_frequency)
            self.declare_parameter(
                'footprint_padding', self._footprint_padding)
            self.declare_parameter('use_radius', self._use_radius)
            self.declare_parameter('robot_radius', self._robot_radius)

            # Get parameters
            self._global_frame = self.get_parameter('global_frame').value
            self._robot_base_frame = self.get_parameter(
                'robot_base_frame').value
            self._map_width_meters = self.get_parameter('width_meters').value
            self._map_height_meters = self.get_parameter('height_meters').value
            self._resolution = self.get_parameter('resolution').value
            self._origin_x = self.get_parameter('origin_x').value
            self._origin_y = self.get_parameter('origin_y').value
            self._transform_tolerance = self.get_parameter(
                'transform_tolerance').value
            self._update_frequency = self.get_parameter(
                'update_frequency').value
            self._publish_frequency = self.get_parameter(
                'publish_frequency').value
            self._footprint_padding = self.get_parameter(
                'footprint_padding').value
            self._use_radius = self.get_parameter('use_radius').value
            self._robot_radius = self.get_parameter('robot_radius').value

            # Initialize TF2
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(
                self._tf_buffer, self)

            self.get_logger().info('Costmap configured successfully')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to configure costmap: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Activate the costmap node.

        Starts costmap update thread and subscribes to topics.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS or FAILURE.
        """
        self.get_logger().info('Activating costmap')

        try:
            self._initialized = True
            self._stopped = False
            self.get_logger().info('Costmap activated successfully')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to activate costmap: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Deactivate the costmap node.

        Stops costmap updates and unsubscribes from topics.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS or FAILURE.
        """
        self.get_logger().info('Deactivating costmap')

        try:
            self._stopped = True
            self.get_logger().info('Costmap deactivated successfully')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to deactivate costmap: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Cleanup the costmap node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS or FAILURE.
        """
        self.get_logger().info('Cleaning up costmap')

        try:
            self._stop_updates = True

            if self._map_update_thread and self._map_update_thread.is_alive():
                self._map_update_thread.join(timeout=2.0)

            self._costmap = None
            self._tf_buffer = None
            self._tf_listener = None

            self.get_logger().info('Costmap cleaned up successfully')
            return TransitionCallbackReturn.SUCCESS

        except Exception as e:
            self.get_logger().error(f'Failed to cleanup costmap: {e}')
            return TransitionCallbackReturn.FAILURE

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """
        Shutdown the costmap node.

        Parameters
        ----------
        state : LifecycleState
            The current lifecycle state.

        Returns
        -------
        TransitionCallbackReturn
            SUCCESS or FAILURE.
        """
        self.get_logger().info('Shutting down costmap')
        return TransitionCallbackReturn.SUCCESS

    # --- Costmap access methods ---

    def get_costmap(self) -> Optional[PyCostmap2D]:
        """
        Get the costmap.

        Returns
        -------
        PyCostmap2D
            The current costmap, or None if not available.
        """
        with self._costmap_mutex:
            return self._costmap

    def set_costmap(self, costmap_msg: Costmap) -> None:
        """
        Set the costmap from a message.

        Parameters
        ----------
        costmap_msg : Costmap
            The costmap message.
        """
        with self._costmap_mutex:
            self._costmap = PyCostmap2D(costmap_msg)

    def update_costmap(self, occupancy_grid: OccupancyGrid) -> None:
        """
        Update the costmap from an occupancy grid message.

        Parameters
        ----------
        occupancy_grid : OccupancyGrid
            The occupancy grid message.
        """
        with self._costmap_mutex:
            py_occupancy = PyOccupancyGrid(occupancy_grid)
            # Store as costmap (convert if needed)
            self.get_logger().debug('Costmap updated from occupancy grid')

    # --- Frame and transform methods ---

    def get_global_frame_id(self) -> str:
        """
        Get the global frame ID of the costmap.

        Returns
        -------
        str
            The global frame ID.
        """
        return self._global_frame

    def get_base_frame_id(self) -> str:
        """
        Get the robot base frame ID.

        Returns
        -------
        str
            The robot base frame ID.
        """
        return self._robot_base_frame

    def get_robot_pose(self) -> Optional[PoseStamped]:
        """
        Get the pose of the robot in the global frame.

        Returns
        -------
        PoseStamped or None
            The robot's pose in the global frame, or None if unavailable.
        """
        if not self._tf_buffer:
            self.get_logger().warn('TF buffer not initialized')
            return None

        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._robot_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self._transform_tolerance),
            )

            pose = PoseStamped()
            pose.header.frame_id = self._global_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose

        except Exception as e:
            self.get_logger().warn(f'Failed to get robot pose: {e}')
            return None

    def transform_pose_to_global_frame(
        self, input_pose: PoseStamped
    ) -> Optional[PoseStamped]:
        """
        Transform a pose to the global frame of the costmap.

        Parameters
        ----------
        input_pose : PoseStamped
            The pose to transform.

        Returns
        -------
        PoseStamped or None
            The transformed pose, or None if transformation failed.
        """
        if not self._tf_buffer:
            self.get_logger().warn('TF buffer not initialized')
            return None

        try:
            transformed = self._tf_buffer.transform(
                input_pose,
                self._global_frame,
                timeout=Duration(seconds=self._transform_tolerance),
            )
            return transformed

        except Exception as e:
            self.get_logger().warn(f'Failed to transform pose: {e}')
            return None

    def get_tf_buffer(self) -> Optional[tf2_ros.Buffer]:
        """
        Get the TF2 buffer.

        Returns
        -------
        tf2_ros.Buffer or None
            The TF2 buffer, or None if not initialized.
        """
        return self._tf_buffer

    # --- Footprint methods ---

    def get_robot_footprint(self) -> List[Point]:
        """
        Get the padded robot footprint.

        Returns
        -------
        List[Point]
            The padded footprint as a list of points.
        """
        return self._padded_footprint

    def get_unpadded_robot_footprint(self) -> List[Point]:
        """
        Get the unpadded robot footprint.

        Returns
        -------
        List[Point]
            The unpadded footprint as a list of points.
        """
        return self._unpadded_footprint

    def get_robot_footprint_polygon(self) -> Polygon:
        """
        Get the robot footprint as a polygon.

        Returns
        -------
        Polygon
            The padded footprint as a geometry_msgs Polygon.
        """
        polygon = Polygon()
        polygon.points = self._padded_footprint
        return polygon

    def set_robot_footprint(self, points: List[Point]) -> None:
        """
        Set the robot footprint.

        Parameters
        ----------
        points : List[Point]
            The footprint points.
        """
        self._unpadded_footprint = list(points)
        # Apply padding
        self._padded_footprint = [
            Point(
                x=p.x + self._footprint_padding,
                y=p.y + self._footprint_padding
            )
            for p in points
        ]

    def set_robot_footprint_polygon(self, footprint: Polygon) -> None:
        """
        Set the robot footprint from a polygon.

        Parameters
        ----------
        footprint : Polygon
            The footprint polygon.
        """
        self.set_robot_footprint(list(footprint.points))

    def get_use_radius(self) -> bool:
        """
        Check if the robot footprint is a circle.

        Returns
        -------
        bool
            True if using radius, False if using polygon.
        """
        return self._use_radius

    def get_robot_radius(self) -> float:
        """
        Get the robot radius (when use_radius is True).

        Returns
        -------
        float
            The robot radius in meters.
        """
        return self._robot_radius

    # --- Information methods ---

    def get_name(self) -> str:
        """
        Get the costmap node name.

        Returns
        -------
        str
            The name of this costmap node.
        """
        return self._name

    def get_transform_tolerance(self) -> float:
        """
        Get the transform tolerance.

        Returns
        -------
        float
            The maximum acceptable delay in TF data (seconds).
        """
        return self._transform_tolerance

    def is_current(self) -> bool:
        """
        Check if the costmap is current.

        Returns
        -------
        bool
            True if the costmap is up to date, False otherwise.
        """
        # TODO: Implement based on layered_costmap update time
        return True

    def wait_until_current(self, timeout: Duration) -> None:
        """
        Wait for the costmap to become current.

        Parameters
        ----------
        timeout : Duration
            Maximum time to wait.

        Raises
        ------
        RuntimeError
            If timeout is exceeded.
        """
        # TODO: Implement timeout-based waiting for costmap updates
        pass

    def reset_layers(self) -> None:
        """Reset all costmap layers."""
        # TODO: Implement layer reset logic
        self.get_logger().debug('Resetting costmap layers')

    # --- Update methods ---

    def start(self) -> None:
        """Start costmap updates."""
        self._stopped = False
        self.get_logger().info('Costmap updates started')

    def stop(self) -> None:
        """Stop costmap updates."""
        self._stopped = True
        self.get_logger().info('Costmap updates stopped')

    def pause(self) -> None:
        """Pause costmap updates."""
        self._stopped = True
        self.get_logger().info('Costmap updates paused')

    def resume(self) -> None:
        """Resume costmap updates."""
        self._stopped = False
        self.get_logger().info('Costmap updates resumed')

    def update_map(self) -> None:
        """Update the costmap with new layer data."""
        if self._stopped or not self._initialized:
            return
        # TODO: Implement costmap layer update logic
        self.get_logger().debug('Costmap updated')
