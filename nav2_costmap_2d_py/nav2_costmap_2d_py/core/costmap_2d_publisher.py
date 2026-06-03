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
Costmap2DPublisher for nav2_costmap_2d_py.

Publishes a Costmap2D as a ``nav_msgs/msg/OccupancyGrid`` (and optionally
``map_msgs/msg/OccupancyGridUpdate`` for incremental updates).
It mirrors the nav2_costmap_2d::Costmap2DPublisher from the C++ implementation.
"""

from typing import Any, List, Optional

from map_msgs.msg import OccupancyGridUpdate
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav_msgs.msg import OccupancyGrid
import numpy as np
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class Costmap2DPublisher:
    """
    Publishes a Costmap2D as OccupancyGrid / OccupancyGridUpdate.

    Parameters
    ----------
    node :
        The ROS2 lifecycle node that owns the publishers.
    costmap : Costmap2D
        Reference to the combined costmap to publish.
    global_frame : str
        Frame ID for the published messages.
    topic_name : str
        Base topic name (e.g. ``"costmap"``).
    always_send_full_costmap : bool
        If True, always publish the full OccupancyGrid; otherwise send
        incremental OccupancyGridUpdate when possible.
    map_vis_z : float
        Z offset for the map origin in visualisation (metres).

    """

    def __init__(
        self,
        node: Any,
        costmap: Costmap2D,
        global_frame: str,
        topic_name: str,
        always_send_full_costmap: bool = False,
        map_vis_z: float = 0.0,
    ) -> None:
        """
        Initialize the publisher, creating the OccupancyGrid and OccupancyGridUpdate publishers.

        Parameters
        ----------
        node : Any
            The ROS2 lifecycle node that owns the publishers.
        costmap : Costmap2D
            Reference to the combined costmap to publish.
        global_frame : str
            Frame ID for the published messages.
        topic_name : str
            Base topic name (e.g. ``"costmap"``).
        always_send_full_costmap : bool
            If True, always publish the full OccupancyGrid; otherwise send
            incremental OccupancyGridUpdate when possible.
        map_vis_z : float
            Z offset for the map origin in visualization (metres).

        """
        self._node = node
        self._costmap = costmap
        self._global_frame = global_frame
        self._topic_name = topic_name
        self._always_send_full = always_send_full_costmap
        self._map_vis_z = map_vis_z
        self._active = False

        # Snapshot of the last full publish (for incremental updates)
        self._saved_origin_x: Optional[float] = None
        self._saved_origin_y: Optional[float] = None
        self._saved_size_x: Optional[int] = None
        self._saved_size_y: Optional[int] = None

        # Latched QoS: KEEP_LAST depth 1, RELIABLE, TRANSIENT_LOCAL
        latched_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publishers (inactive until on_activate)
        self._costmap_pub = node.create_publisher(
            OccupancyGrid, topic_name, latched_qos
        )
        self._costmap_update_pub = node.create_publisher(
            OccupancyGridUpdate, topic_name + '_updates', latched_qos
        )

    # ------------------------------------------------------------------
    # Cost translation (class-level LUT, built once at class definition)
    # ------------------------------------------------------------------

    @staticmethod
    def _build_cost_translation_table() -> 'np.ndarray':
        """
        Build a 256-element lookup table from costmap cost to OccupancyGrid value.

        Returns
        -------
        np.ndarray
            int8 array of length 256 indexed by raw cost value.

        """
        table = np.zeros(256, dtype=np.int8)
        idx = np.arange(1, 253)
        table[idx] = (1 + (97 * (idx - 1)) // 251).astype(np.int8)
        table[253] = 99    # INSCRIBED_INFLATED_OBSTACLE
        table[254] = 100   # LETHAL_OBSTACLE
        table[255] = -1    # NO_INFORMATION
        return table

    _COST_TRANSLATION_TABLE_NP: 'np.ndarray' = _build_cost_translation_table()

    def _translate_costmap(self, raw: bytearray) -> List[int]:
        """
        Translate a raw costmap buffer to OccupancyGrid values.

        Parameters
        ----------
        raw : bytearray
            The raw costmap buffer of cost values (0-255).

        Returns
        -------
        list of int
            The translated OccupancyGrid values (-1, 0-100).

        """
        return self._COST_TRANSLATION_TABLE_NP[
            np.frombuffer(raw, dtype=np.uint8)
        ].tolist()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_activate(self) -> None:
        """Activate the publisher so subsequent calls publish data."""
        self._active = True

    def on_deactivate(self) -> None:
        """Deactivate the publisher so it stops publishing data."""
        self._active = False

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def publish_costmap(self) -> None:
        """
        Publish the visualization data over ROS.

        Only publishes when the publisher is active.
        If ``always_send_full_costmap`` is True or no previous snapshot
        exists, publishes a full ``OccupancyGrid``; otherwise publishes an
        incremental ``OccupancyGridUpdate``.
        """
        if not self._active:
            return

        costmap = self._costmap
        with costmap.get_mutex():
            current_origin_x = costmap.origin_x
            current_origin_y = costmap.origin_y
            current_size_x = costmap.size_x
            current_size_y = costmap.size_y

            send_full = (
                self._always_send_full
                or self._saved_origin_x is None
                or self._saved_origin_x != current_origin_x
                or self._saved_origin_y != current_origin_y
                or self._saved_size_x != current_size_x
                or self._saved_size_y != current_size_y
            )

            if send_full:
                self._publish_full(costmap)
            else:
                self._publish_update(costmap)

            self._saved_origin_x = current_origin_x
            self._saved_origin_y = current_origin_y
            self._saved_size_x = current_size_x
            self._saved_size_y = current_size_y

    def _publish_full(self, costmap: Costmap2D) -> None:
        """
        Publish a full OccupancyGrid.

        Parameters
        ----------
        costmap : Costmap2D
            The costmap whose full contents are published.

        """
        msg = OccupancyGrid()
        now = self._node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self._global_frame

        msg.info.resolution = costmap.resolution
        msg.info.width = costmap.size_x
        msg.info.height = costmap.size_y
        msg.info.origin.position.x = costmap.origin_x
        msg.info.origin.position.y = costmap.origin_y
        msg.info.origin.position.z = self._map_vis_z
        msg.info.origin.orientation.w = 1.0

        raw = costmap.get_char_map()
        msg.data = self._translate_costmap(raw)
        self._costmap_pub.publish(msg)

    def _publish_update(self, costmap: Costmap2D) -> None:
        """
        Publish an OccupancyGridUpdate (incremental).

        Parameters
        ----------
        costmap : Costmap2D
            The costmap whose contents are published as an incremental update.

        """
        msg = OccupancyGridUpdate()
        now = self._node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self._global_frame
        msg.x = 0
        msg.y = 0
        msg.width = costmap.size_x
        msg.height = costmap.size_y
        raw = costmap.get_char_map()
        msg.data = self._translate_costmap(raw)
        self._costmap_update_pub.publish(msg)
