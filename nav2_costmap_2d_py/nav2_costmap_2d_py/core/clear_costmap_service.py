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
ClearCostmapService for nav2_costmap_2d_py.

It mirrors the nav2_costmap_2d::ClearCostmapService from the C++ implementation.
Exposes services to clear costmap objects in inclusive/exclusive regions or completely.
"""

from __future__ import annotations

from typing import Callable, cast, List, Optional, Tuple, TYPE_CHECKING

from geometry_msgs.msg import PoseStamped
from nav2_costmap_2d_py.core.costmap_layer import CostmapLayer
from nav2_msgs.srv import (ClearCostmapAroundPose, ClearCostmapAroundRobot,
                           ClearCostmapExceptRegion, ClearEntireCostmap)

if TYPE_CHECKING:
    from nav2_costmap_2d_py.core.layer import Layer
    from nav2_costmap_2d_py.costmap_2d_ros import Costmap2DROS


class ClearCostmapService:
    """
    Expose services to clear costmap objects in inclusive/exclusive regions or completely.

    Parameters
    ----------
    node :
        The lifecycle node that owns the services (Costmap2DROS).
    costmap_ros :
        The Costmap2DROS instance to clear (provides ``get_costmap()``,
        ``get_layered_costmap()``, ``get_robot_pose()``, ``get_name()``,
        ``get_global_frame_id()``, ``get_tf_buffer()``, ``reset_layers()``).

    """

    def __init__(self, node: 'Costmap2DROS', costmap_ros: 'Costmap2DROS') -> None:
        """
        Initialize the service, creating the four clearing service servers.

        Parameters
        ----------
        node : Costmap2DROS
            The lifecycle node that owns the services.
        costmap_ros : Costmap2DROS
            The Costmap2DROS instance to clear.

        """
        self._node = node
        self._costmap = costmap_ros
        self._logger = node.get_logger()
        name = costmap_ros.get_name()

        self._clear_except_service = node.create_service(
            ClearCostmapExceptRegion, f'clear_except_{name}',
            self._clear_except_region_callback,
        )
        self._clear_around_service = node.create_service(
            ClearCostmapAroundRobot, f'clear_around_{name}',
            self._clear_around_robot_callback,
        )
        self._clear_entire_service = node.create_service(
            ClearEntireCostmap, f'clear_entirely_{name}',
            self._clear_entire_callback,
        )
        self._clear_around_pose_service = node.create_service(
            ClearCostmapAroundPose, f'clear_around_pose_{name}',
            self._clear_around_pose_callback,
        )

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------

    def _clear_except_region_callback(
        self,
        request: ClearCostmapExceptRegion.Request,
        response: ClearCostmapExceptRegion.Response,
    ) -> ClearCostmapExceptRegion.Response:
        """
        Clear the costmap everywhere except a region around the robot.

        Parameters
        ----------
        request : ClearCostmapExceptRegion.Request
            The service request holding the ``reset_distance``.
        response : ClearCostmapExceptRegion.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearCostmapExceptRegion.Response
            The service response.

        """
        self._logger.info(
            '[ClearCostmapService] Received request to clear except a region '
            f'the {self._costmap.get_name()} costmap')
        self.clear_region(request.reset_distance, True, list(request.plugins))
        return response

    def _clear_around_robot_callback(
        self,
        request: ClearCostmapAroundRobot.Request,
        response: ClearCostmapAroundRobot.Response,
    ) -> ClearCostmapAroundRobot.Response:
        """
        Clear the costmap in a region around the robot.

        Parameters
        ----------
        request : ClearCostmapAroundRobot.Request
            The service request holding the ``reset_distance``.
        response : ClearCostmapAroundRobot.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearCostmapAroundRobot.Response
            The service response.

        """
        self._logger.info(
            '[ClearCostmapService] Received request to clear around the robot '
            f'the {self._costmap.get_name()} costmap')
        self.clear_region(request.reset_distance, False, list(request.plugins))
        return response

    def _clear_entire_callback(
        self,
        request: ClearEntireCostmap.Request,
        response: ClearEntireCostmap.Response,
    ) -> ClearEntireCostmap.Response:
        """
        Clear the entire costmap and all (or the requested) clearable layers.

        Parameters
        ----------
        request : ClearEntireCostmap.Request
            The service request holding the ``plugins``.
        response : ClearEntireCostmap.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearEntireCostmap.Response
            The service response.

        """
        self._logger.info(
            '[ClearCostmapService] Received request to clear entirely '
            f'the {self._costmap.get_name()} costmap')
        self.clear_entirely(list(request.plugins))
        return response

    def _clear_around_pose_callback(
        self,
        request: ClearCostmapAroundPose.Request,
        response: ClearCostmapAroundPose.Response,
    ) -> ClearCostmapAroundPose.Response:
        """
        Clear the costmap in a region around the given pose.

        Parameters
        ----------
        request : ClearCostmapAroundPose.Request
            The service request holding the ``pose`` and ``reset_distance``.
        response : ClearCostmapAroundPose.Response
            The (empty) service response, returned unchanged.

        Returns
        -------
        ClearCostmapAroundPose.Response
            The service response.

        """
        self._logger.info(
            '[ClearCostmapService] Received request to clear around a pose '
            f'the {self._costmap.get_name()} costmap')
        self.clear_around_pose(
            request.pose, request.reset_distance, list(request.plugins))
        return response

    # ------------------------------------------------------------------
    # Clearing operations (faithful port of the C++ methods)
    # ------------------------------------------------------------------

    def clear_region(
        self,
        reset_distance: float,
        invert: bool,
        plugins: Optional[List[str]] = None,
    ) -> bool:
        """
        Clear the costmap layers within (or, if *invert*, outside) a region around the robot.

        Parameters
        ----------
        reset_distance : float
            Side length of the square region around the robot, in metres.
        invert : bool
            If False, clear cells inside the region; if True, clear cells outside.
        plugins : list of str, optional
            If given, only these (clearable) layers are cleared; otherwise all
            clearable layers are cleared.

        Returns
        -------
        bool
            ``True`` on success, ``False`` if the robot pose was unavailable or
            an invalid plugin was requested.

        """
        plugins = plugins or []
        pos = self.get_position()
        if pos is None:
            self._logger.error(
                'Cannot clear map because robot pose cannot be retrieved.')
            return False
        x, y = pos

        layers = self._get_plugins()

        if plugins:
            return self.validate_and_clear_plugins(
                plugins, layers,
                lambda layer: self.clear_layer_region(layer, x, y, reset_distance, invert),
                'clear costmap region')

        for layer in layers:
            if layer.is_clearable():
                self.clear_layer_region(
                    cast(CostmapLayer, layer), x, y, reset_distance, invert)
        return True

    def clear_around_pose(
        self,
        pose: PoseStamped,
        reset_distance: float,
        plugins: Optional[List[str]] = None,
    ) -> bool:
        """
        Clear the costmap layers within a region around the given pose.

        Parameters
        ----------
        pose : PoseStamped
            The pose to clear around (transformed to the costmap global frame).
        reset_distance : float
            Side length of the square region around the pose, in metres.
        plugins : list of str, optional
            If given, only these (clearable) layers are cleared; otherwise all
            clearable layers are cleared.

        Returns
        -------
        bool
            ``True`` on success, ``False`` if the pose could not be transformed
            or an invalid plugin was requested.

        """
        plugins = plugins or []
        global_frame = self._costmap.get_global_frame_id()

        if pose.header.frame_id == global_frame or not pose.header.frame_id:
            global_pose = pose
        else:
            tf_buffer = self._costmap.get_tf_buffer()
            if tf_buffer is None:
                self._logger.error(
                    'Cannot clear map around pose: no TF buffer available.')
                return False
            try:
                global_pose = tf_buffer.transform(pose, global_frame)
            except Exception as ex:  # noqa: BLE001
                self._logger.error(
                    'Cannot clear map around pose because pose cannot be '
                    f'transformed to costmap frame: {ex}')
                return False

        x = global_pose.pose.position.x
        y = global_pose.pose.position.y

        layers = self._get_plugins()

        if plugins:
            return self.validate_and_clear_plugins(
                plugins, layers,
                lambda layer: self.clear_layer_region(layer, x, y, reset_distance, False),
                'clear costmap around pose')

        for layer in layers:
            if layer.is_clearable():
                self.clear_layer_region(
                    cast(CostmapLayer, layer), x, y, reset_distance, False)
        return True

    def clear_layer_region(
        self,
        costmap: CostmapLayer,
        pose_x: float,
        pose_y: float,
        reset_distance: float,
        invert: bool,
    ) -> None:
        """
        Clear a square region of a single costmap layer.

        The square is ``reset_distance`` wide and centred on ``(pose_x, pose_y)``,
        and ``addExtraBounds`` is used so the whole layer is re-merged next cycle.

        Parameters
        ----------
        costmap : CostmapLayer
            The layer whose internal grid is cleared.
        pose_x, pose_y : float
            World coordinates of the centre of the region.
        reset_distance : float
            Side length of the square region, in metres.
        invert : bool
            If False, clear cells inside the square; if True, clear cells outside.

        """
        with costmap.get_mutex():
            start_point_x = pose_x - reset_distance / 2
            start_point_y = pose_y - reset_distance / 2
            end_point_x = start_point_x + reset_distance
            end_point_y = start_point_y + reset_distance

            start_x, start_y = costmap.world_to_map_no_bounds(start_point_x, start_point_y)
            end_x, end_y = costmap.world_to_map_no_bounds(end_point_x, end_point_y)

            costmap.clear_area(start_x, start_y, end_x, end_y, invert)

            ox = costmap.origin_x
            oy = costmap.origin_y
            width = costmap.size_x_meters
            height = costmap.size_y_meters
            costmap.add_extra_bounds(ox, oy, ox + width, oy + height)

    def clear_entirely(self, plugins: Optional[List[str]] = None) -> bool:
        """
        Clear the entire costmap and all (or the requested) clearable layers.

        Parameters
        ----------
        plugins : list of str, optional
            If given, only these (clearable) layers are reset and then the master
            grid is reset; otherwise every layer is reset via ``reset_layers``.

        Returns
        -------
        bool
            ``True`` on success, ``False`` if an invalid plugin was requested.

        """
        plugins = plugins or []
        master = self._costmap.get_costmap()
        if master is None:
            return False
        with master.get_mutex():
            if not plugins:
                self._costmap.reset_layers()
                return True

            layers = self._get_plugins()
            result = self.validate_and_clear_plugins(
                plugins, layers,
                lambda layer: layer.reset_map(
                    0, 0, layer.size_x, layer.size_y),
                'clear costmap entirely')
            if result:
                master.reset_map(0, 0, master.size_x, master.size_y)
            return result

    # ------------------------------------------------------------------
    # Plugin selection helpers
    # ------------------------------------------------------------------

    def validate_plugins(
        self,
        requested_plugins: List[str],
        layers: List['Layer'],
    ) -> List[str]:
        """
        Return the list of invalid (not found / not clearable) requested plugins.

        Parameters
        ----------
        requested_plugins : list of str
            The names of the plugins requested for clearing.
        layers : list of Layer
            The available costmap plugins.

        Returns
        -------
        list of str
            Human-readable descriptions of the invalid plugins (empty if all
            requested plugins are valid and clearable).

        """
        invalid_plugins: List[str] = []
        for requested in requested_plugins:
            found = False
            clearable = False
            for layer in layers:
                if layer.get_name() == requested:
                    found = True
                    clearable = layer.is_clearable()
                    break
            if not found:
                invalid_plugins.append(requested + ' (not found)')
            elif not clearable:
                invalid_plugins.append(requested + ' (not clearable)')
        return invalid_plugins

    def validate_and_clear_plugins(
        self,
        plugins: List[str],
        layers: List['Layer'],
        clear_callback: Callable[[CostmapLayer], None],
        operation_name: str,
    ) -> bool:
        """
        Validate the requested plugins and run ``clear_callback`` on each matching layer.

        If any requested plugin is invalid, no layers are cleared.

        Parameters
        ----------
        plugins : list of str
            The names of the plugins requested for clearing.
        layers : list of Layer
            The available costmap plugins.
        clear_callback : callable
            The clearing action to apply to each matching ``CostmapLayer``.
        operation_name : str
            Human-readable name of the operation, used in log messages.

        Returns
        -------
        bool
            ``True`` if all requested plugins were valid and cleared, ``False``
            otherwise.

        """
        invalid_plugins = self.validate_plugins(plugins, layers)
        if invalid_plugins:
            self._logger.error(
                'Invalid plugin(s) requested for clearing: '
                + ', '.join(invalid_plugins))
            self._logger.error(
                f'Failed to {operation_name}: {len(invalid_plugins)} invalid '
                f'plugin(s) out of {len(plugins)} requested. No layers were cleared.')
            return False

        for layer in layers:
            if layer.get_name() in plugins:
                clear_callback(layer)  # type: ignore[arg-type]
                self._logger.info(
                    f"Performed action '{operation_name}' on layer: {layer.get_name()}")
        return True

    # ------------------------------------------------------------------
    # Implementation helpers
    # ------------------------------------------------------------------

    def get_position(self) -> Optional[Tuple[float, float]]:
        """
        Get the robot's world position.

        Returns
        -------
        tuple of float or None
            The ``(x, y)`` world position of the robot, or ``None`` if the robot
            pose could not be obtained.

        """
        pose = self._costmap.get_robot_pose()
        if pose is None:
            return None
        return pose.pose.position.x, pose.pose.position.y

    def _get_plugins(self) -> List['Layer']:
        """Return the costmap plugins (empty list if no layered costmap)."""
        lc = self._costmap.get_layered_costmap()
        if lc is None:
            return []
        return lc.get_plugins()
