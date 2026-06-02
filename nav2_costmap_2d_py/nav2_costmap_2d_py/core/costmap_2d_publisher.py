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

from typing import Optional

import numpy as np
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from nav2_costmap_2d_py.core.costmap_2d import Costmap2D
from nav2_costmap_2d_py.core.cost_values import (
    FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION,
)


def _build_cost_translation_table() -> list:
    """Build a 256-element LUT from costmap cost → OccupancyGrid value."""
    table = [0] * 256
    table[FREE_SPACE] = 0
    table[NO_INFORMATION] = -1
    table[LETHAL_OBSTACLE] = 100
    table[INSCRIBED_INFLATED_OBSTACLE] = 99
    for i in range(1, 256):
        if i not in (
                FREE_SPACE, NO_INFORMATION, LETHAL_OBSTACLE, INSCRIBED_INFLATED_OBSTACLE):
            # Linear mapping 1..252 → 1..98
            table[i] = max(1, min(98, int(i * 100.0 / 255.0)))
    return table


_COST_TRANSLATION_TABLE = _build_cost_translation_table()
# NumPy lookup table (int8) for vectorised cost->OccupancyGrid translation.
_COST_TRANSLATION_TABLE_NP = np.asarray(_COST_TRANSLATION_TABLE, dtype=np.int8)


def _translate_costmap(raw: bytearray) -> list:
    """
    Vectorised replacement for ``[_COST_TRANSLATION_TABLE[c] for c in raw]``.

    The per-cell Python list comprehension ran over the whole costmap on every
    publish and held the GIL for hundreds of ms on large maps, freezing the
    process and making the planner miss the BT's goal-acknowledge timeout. The
    NumPy LUT index runs in C and releases the GIL.
    """
    arr = np.frombuffer(raw, dtype=np.uint8)
    return _COST_TRANSLATION_TABLE_NP[arr].tolist()


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
        node,
        costmap: Costmap2D,
        global_frame: str,
        topic_name: str,
        always_send_full_costmap: bool = False,
        map_vis_z: float = 0.0,
    ) -> None:
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
    # Lifecycle
    # ------------------------------------------------------------------

    def on_activate(self) -> None:
        self._active = True

    def on_deactivate(self) -> None:
        self._active = False

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

    def publish_costmap(self) -> None:
        """
        Publish the costmap.

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
        """Publish a full OccupancyGrid."""
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
        msg.data = _translate_costmap(raw)
        self._costmap_pub.publish(msg)

    def _publish_update(self, costmap: Costmap2D) -> None:
        """Publish an OccupancyGridUpdate (incremental)."""
        msg = OccupancyGridUpdate()
        now = self._node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self._global_frame
        msg.x = 0
        msg.y = 0
        msg.width = costmap.size_x
        msg.height = costmap.size_y
        raw = costmap.get_char_map()
        msg.data = _translate_costmap(raw)
        self._costmap_update_pub.publish(msg)
