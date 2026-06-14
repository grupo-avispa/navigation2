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
FootprintSubscriber for nav2_costmap_2d_py.

Subscriber to the footprint topic to get the current robot footprint (if
changing) for use in collision avoidance.
It mirrors the nav2_costmap_2d::FootprintSubscriber from the C++ implementation.
"""

import math
from typing import Any, List, Optional, Tuple

from nav2_costmap_2d_py.core.footprint import to_point_vector, transform_footprint
import rclpy.duration
import rclpy.time
from std_msgs.msg import Header

Footprint = List[Tuple[float, float]]


class FootprintSubscriber:
    """Subscriber to the footprint topic for the current robot footprint."""

    def __init__(
        self,
        node: Any,
        topic_name: str,
        tf: Any,
        robot_base_frame: str = 'base_link',
        transform_tolerance: float = 0.1,
    ) -> None:
        """
        Construct the subscriber.

        Parameters
        ----------
        node : Any
            The owning node.
        topic_name : str
            The footprint (PolygonStamped) topic to subscribe to.
        tf : tf2_ros.Buffer
            The tf2 buffer used for transforms.
        robot_base_frame : str
            The robot base frame.
        transform_tolerance : float
            The transform tolerance, in seconds.

        """
        from geometry_msgs.msg import PolygonStamped
        self._node = node
        self._tf = tf
        self._robot_base_frame = robot_base_frame
        self._transform_tolerance = transform_tolerance
        self._footprint_received = False
        self._footprint: Optional[PolygonStamped] = None
        self._footprint_sub = node.create_subscription(
            PolygonStamped, topic_name, self.footprint_callback, 1)

    def get_footprint_raw(self) -> Tuple[bool, Footprint, Header]:
        """
        Return the latest footprint as received from the topic (oriented).

        Returns
        -------
        (ok, footprint, header)
            ``ok`` is False if no footprint has been received.

        """
        if not self._footprint_received or self._footprint is None:
            return False, [], Header()
        footprint = to_point_vector(self._footprint.polygon)
        return True, footprint, self._footprint.header

    def get_footprint_in_robot_frame(self) -> Tuple[bool, Footprint, Header]:
        """
        Return the latest footprint transformed into the robot base frame (unoriented).

        Returns
        -------
        (ok, footprint, header)
            ``ok`` is False if no footprint has been received or the transform
            failed.

        """
        ok, footprint, header = self.get_footprint_raw()
        if not ok:
            return False, [], Header()

        pose = self._get_current_pose(header.frame_id, header.stamp)
        if pose is None:
            return False, [], Header()
        x, y, theta = pose

        temp = transform_footprint(-x, -y, 0.0, footprint)
        footprint = transform_footprint(0.0, 0.0, -theta, temp)

        out_header = Header()
        out_header.frame_id = self._robot_base_frame
        out_header.stamp = header.stamp
        return True, footprint, out_header

    def footprint_callback(self, msg: Any) -> None:
        """Store a new footprint message."""
        self._footprint = msg
        if not self._footprint_received:
            self._footprint_received = True

    def _get_current_pose(
        self, frame_id: str, stamp: Any
    ) -> Optional[Tuple[float, float, float]]:
        """Return the robot ``(x, y, theta)`` in ``frame_id`` at ``stamp``."""
        if self._tf is None:
            return None
        try:
            tr = self._tf.lookup_transform(
                frame_id, self._robot_base_frame,
                rclpy.time.Time.from_msg(stamp),
                timeout=rclpy.duration.Duration(seconds=self._transform_tolerance))
        except Exception:  # noqa: BLE001
            return None
        x = tr.transform.translation.x
        y = tr.transform.translation.y
        q = tr.transform.rotation
        theta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, theta
