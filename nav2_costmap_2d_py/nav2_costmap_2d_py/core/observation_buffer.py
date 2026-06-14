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
ObservationBuffer for nav2_costmap_2d_py.

Takes in point clouds from sensors, transforms them to the desired frame, and
stores them.
It mirrors the nav2_costmap_2d::ObservationBuffer from the C++ implementation.
"""

import threading
from typing import Any, List

from nav2_costmap_2d_py.core.observation import Observation
import rclpy.duration
import rclpy.time
from sensor_msgs.msg import PointCloud2


class ObservationBuffer:
    """Takes in point clouds, transforms them to the desired frame, and stores them."""

    def __init__(
        self,
        node: Any,
        topic_name: str,
        observation_keep_time: float,
        expected_update_rate: float,
        min_obstacle_height: float,
        max_obstacle_height: float,
        obstacle_max_range: float,
        obstacle_min_range: float,
        raytrace_max_range: float,
        raytrace_min_range: float,
        tf2_buffer: Any,
        global_frame: str,
        sensor_frame: str,
        tf_tolerance: float,
    ) -> None:
        """
        Construct an observation buffer.

        Parameters
        ----------
        node : Any
            The owning lifecycle node (provides the clock and logger).
        topic_name : str
            The topic of the observations, used as an identifier in messages.
        observation_keep_time : float
            Persistence of observations in seconds (0 means keep only the
            latest).
        expected_update_rate : float
            How often this buffer is expected to be updated, in seconds (0 means
            no limit).
        min_obstacle_height : float
            The minimum height of a hitpoint to be considered legal.
        max_obstacle_height : float
            The maximum height of a hitpoint to be considered legal.
        obstacle_max_range : float
            The range to which the sensor is trusted for inserting obstacles.
        obstacle_min_range : float
            The range from which the sensor is trusted for inserting obstacles.
        raytrace_max_range : float
            The range to which the sensor is trusted for raytracing clearing.
        raytrace_min_range : float
            The range from which the sensor is trusted for raytracing clearing.
        tf2_buffer : tf2_ros.Buffer
            A reference to a tf2 buffer.
        global_frame : str
            The frame to transform point clouds into.
        sensor_frame : str
            The frame of the origin of the sensor (may be empty to read it from
            the messages).
        tf_tolerance : float
            The amount of time to wait for a transform to be available, in
            seconds.

        """
        self._node = node
        self._clock = node.get_clock()
        self._logger = node.get_logger()
        self._tf2_buffer = tf2_buffer
        self._observation_keep_time = rclpy.duration.Duration(
            seconds=observation_keep_time)
        self._expected_update_rate = rclpy.duration.Duration(
            seconds=expected_update_rate)
        self._last_updated = self._clock.now()
        self._global_frame = global_frame
        self._sensor_frame = sensor_frame
        self._topic_name = topic_name
        self._min_obstacle_height = min_obstacle_height
        self._max_obstacle_height = max_obstacle_height
        self._obstacle_max_range = obstacle_max_range
        self._obstacle_min_range = obstacle_min_range
        self._raytrace_max_range = raytrace_max_range
        self._raytrace_min_range = raytrace_min_range
        self._tf_tolerance = tf_tolerance
        self._observation_list: List[Observation] = []
        self._lock = threading.RLock()

    def buffer_cloud(self, cloud: PointCloud2) -> None:
        """
        Transform a point cloud to the global frame and buffer it.

        The burden is on the caller to make sure the transform is available.

        Parameters
        ----------
        cloud : sensor_msgs.msg.PointCloud2
            The cloud to be buffered.

        """
        from geometry_msgs.msg import PointStamped
        from sensor_msgs_py import point_cloud2
        import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
        from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

        origin_frame = cloud.header.frame_id if self._sensor_frame == '' else self._sensor_frame

        try:
            observation = Observation()
            observation.raytrace_max_range = self._raytrace_max_range
            observation.raytrace_min_range = self._raytrace_min_range
            observation.obstacle_max_range = self._obstacle_max_range
            observation.obstacle_min_range = self._obstacle_min_range

            local_origin = PointStamped()
            local_origin.header.stamp = cloud.header.stamp
            local_origin.header.frame_id = origin_frame
            local_origin.point.x = 0.0
            local_origin.point.y = 0.0
            local_origin.point.z = 0.0
            transform = self._tf2_buffer.lookup_transform(
                self._global_frame, origin_frame,
                rclpy.time.Time.from_msg(cloud.header.stamp),
                timeout=rclpy.duration.Duration(seconds=self._tf_tolerance))
            global_origin = tf2_geometry_msgs.do_transform_point(
                local_origin, transform)
            observation.origin.x = global_origin.point.x
            observation.origin.y = global_origin.point.y
            observation.origin.z = global_origin.point.z

            cloud_transform = self._tf2_buffer.lookup_transform(
                self._global_frame, cloud.header.frame_id,
                rclpy.time.Time.from_msg(cloud.header.stamp),
                timeout=rclpy.duration.Duration(seconds=self._tf_tolerance))
            global_frame_cloud = do_transform_cloud(cloud, cloud_transform)

            # Filter the points by height threshold.
            kept = [
                (p[0], p[1], p[2])
                for p in point_cloud2.read_points(
                    global_frame_cloud, field_names=('x', 'y', 'z'), skip_nans=True)
                if self._min_obstacle_height <= p[2] <= self._max_obstacle_height
            ]
            observation.cloud = point_cloud2.create_cloud_xyz32(
                global_frame_cloud.header, kept)
            observation.cloud.header.stamp = cloud.header.stamp

            self._observation_list.insert(0, observation)
        except Exception as ex:  # noqa: BLE001
            self._logger.error(
                'TF Exception that should never happen for sensor frame: '
                f'{self._sensor_frame}, cloud frame: {cloud.header.frame_id}, {ex}')
            return

        self._last_updated = self._clock.now()
        self._purge_stale_observations()

    def get_observations(self) -> List[Observation]:
        """
        Return copies of all current observations after purging stale ones.

        Returns
        -------
        list of Observation
            The current observations.

        """
        self._purge_stale_observations()
        return list(self._observation_list)

    def is_current(self) -> bool:
        """
        Check if the observation buffer is being updated at its expected rate.

        Returns
        -------
        bool
            True if it is being updated at the expected rate, False otherwise.

        """
        if self._expected_update_rate == rclpy.duration.Duration(seconds=0.0):
            return True

        elapsed = self._clock.now() - self._last_updated
        current = elapsed <= self._expected_update_rate
        if not current:
            self._logger.warning(
                f'The {self._topic_name} observation buffer has not been updated '
                f'for {elapsed.nanoseconds / 1e9:.2f} seconds, and it should be '
                f'updated every {self._expected_update_rate.nanoseconds / 1e9:.2f} seconds.')
        return current

    def lock(self) -> None:
        """Lock the observation buffer."""
        self._lock.acquire()

    def unlock(self) -> None:
        """Unlock the observation buffer."""
        self._lock.release()

    def reset_last_updated(self) -> None:
        """Reset the last-updated timestamp to now."""
        self._last_updated = self._clock.now()

    def _purge_stale_observations(self) -> None:
        """Remove any stale observations from the buffer list."""
        if not self._observation_list:
            return
        if self._observation_keep_time == rclpy.duration.Duration(seconds=0.0):
            # Keep only the latest observation.
            del self._observation_list[1:]
        else:
            now = self._clock.now()
            self._observation_list = [
                obs for obs in self._observation_list
                if (now - rclpy.time.Time.from_msg(obs.cloud.header.stamp))
                <= self._observation_keep_time
            ]
