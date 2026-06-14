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
CostmapSubscriber for nav2_costmap_2d_py.

Subscribes to the costmap via a ROS topic and reconstructs a Costmap2D.
It mirrors the nav2_costmap_2d::CostmapSubscriber from the C++ implementation.
"""

import threading
from typing import Any, Optional

from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_msgs.msg import Costmap, CostmapUpdate
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def _latched_qos(depth: int = 1) -> QoSProfile:
    """Build a latched (transient-local, reliable) QoS profile."""
    return QoSProfile(
        depth=depth,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class CostmapSubscriber:
    """Subscribes to the costmap via a ROS topic."""

    def __init__(self, node: Any, topic_name: str) -> None:
        """
        Construct the subscriber.

        Parameters
        ----------
        node : Any
            The owning node.
        topic_name : str
            The base topic name (the update topic is ``<topic_name>_updates``).

        """
        self._node = node
        self._topic_name = topic_name
        self._frame_id = ''
        self._logger = node.get_logger()
        self._costmap: Optional[Costmap2D] = None
        self._costmap_msg: Optional[Costmap] = None
        self._costmap_msg_mutex = threading.Lock()

        self._costmap_sub = node.create_subscription(
            Costmap, topic_name, self.costmap_callback, _latched_qos(3))
        self._costmap_update_sub = node.create_subscription(
            CostmapUpdate, topic_name + '_updates',
            self.costmap_update_callback, _latched_qos())

    def get_costmap(self) -> Costmap2D:
        """
        Return the current costmap (raises if none received).

        Returns
        -------
        Costmap2D
            The reconstructed costmap.

        """
        if not self.is_costmap_received():
            raise RuntimeError('Costmap is not available')
        if self._costmap_msg is not None:
            self.process_current_costmap_msg()
        assert self._costmap is not None
        return self._costmap

    def costmap_callback(self, msg: Costmap) -> None:
        """Handle a full costmap message."""
        first = not self.is_costmap_received()
        with self._costmap_msg_mutex:
            self._costmap_msg = msg
            self._frame_id = msg.header.frame_id
        if first:
            self._costmap = Costmap2D(
                msg.metadata.size_x, msg.metadata.size_y,
                msg.metadata.resolution, msg.metadata.origin.position.x,
                msg.metadata.origin.position.y)
            self.process_current_costmap_msg()

    def costmap_update_callback(self, update_msg: CostmapUpdate) -> None:
        """Handle an incremental costmap update message."""
        if not self.is_costmap_received():
            self._logger.warning('No costmap received.')
            return
        if self._costmap_msg is not None:
            self.process_current_costmap_msg()

        assert self._costmap is not None
        with self._costmap.get_mutex():
            size_x = self._costmap.size_x
            size_y = self._costmap.size_y
            if (size_x < update_msg.x + update_msg.size_x
                    or size_y < update_msg.y + update_msg.size_y):
                self._logger.warning('Update area outside of original map area.')
                return
            master_array = self._costmap.get_char_map()
            for y in range(update_msg.size_y):
                dst = (y + update_msg.y) * size_x + update_msg.x
                src = y * update_msg.size_x
                master_array[dst:dst + update_msg.size_x] = bytes(
                    update_msg.data[src:src + update_msg.size_x])

    def process_current_costmap_msg(self) -> None:
        """Apply the buffered full costmap message to the Costmap2D."""
        assert self._costmap is not None
        with self._costmap.get_mutex(), self._costmap_msg_mutex:
            msg = self._costmap_msg
            if msg is None:
                return
            if self.have_costmap_parameters_changed():
                self._costmap.resize_map(
                    msg.metadata.size_x, msg.metadata.size_y,
                    msg.metadata.resolution, msg.metadata.origin.position.x,
                    msg.metadata.origin.position.y)
            master_array = self._costmap.get_char_map()
            master_array[:] = bytes(msg.data)
            self._costmap_msg = None

    def have_costmap_parameters_changed(self) -> bool:
        """Return whether the costmap size, resolution or origin changed."""
        return (self.has_costmap_size_changed()
                or self.has_costmap_resolution_changed()
                or self.has_costmap_origin_position_changed())

    def has_costmap_size_changed(self) -> bool:
        """Return whether the costmap size changed."""
        assert self._costmap is not None and self._costmap_msg is not None
        return (self._costmap.size_x != self._costmap_msg.metadata.size_x
                or self._costmap.size_y != self._costmap_msg.metadata.size_y)

    def has_costmap_resolution_changed(self) -> bool:
        """Return whether the costmap resolution changed."""
        assert self._costmap is not None and self._costmap_msg is not None
        return self._costmap.resolution != self._costmap_msg.metadata.resolution

    def has_costmap_origin_position_changed(self) -> bool:
        """Return whether the costmap origin changed."""
        assert self._costmap is not None and self._costmap_msg is not None
        return (self._costmap.origin_x != self._costmap_msg.metadata.origin.position.x
                or self._costmap.origin_y != self._costmap_msg.metadata.origin.position.y)

    def is_costmap_received(self) -> bool:
        """Return whether a costmap has been received yet."""
        return self._costmap is not None

    def get_frame_id(self) -> str:
        """Return the frame id of the last received costmap."""
        return self._frame_id
