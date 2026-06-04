# Copyright (c) 2025 Nav2 Python Port
#
# Licensed under the Apache License, Version 2.0 (the "License")
"""
Example concrete planner plugin: a trivial straight-line planner.

Register it in your params YAML as:

    planner_server:
      ros__parameters:
        planner_plugins: ["GridBased"]
        GridBased:
          plugin: "nav2_planner_example_py.example_straight_line_planner.StraightLinePlanner"
"""

import math
from typing import List

from geometry_msgs.msg import PoseStamped
from nav2_core_py.global_planner import GlobalPlanner
from nav_msgs.msg import Path


class StraightLinePlanner(GlobalPlanner):
    """
    Minimal example planner that returns a straight-line path.

    Use this as a template for implementing your own Python planner plugin.
    """

    def configure(self, parent, name, tf_buffer, costmap_ros):
        self._node = parent
        self._name = name
        self._tf_buffer = tf_buffer
        self._costmap = costmap_ros

        # Declare / read plugin-specific parameters. declare_or_get_parameter is
        # idempotent, so a configure -> cleanup -> configure cycle (e.g. a
        # lifecycle reset after a server respawn) does not raise
        # ParameterAlreadyDeclaredException.
        self._resolution = self.declare_or_get_parameter(
            f'{name}.interpolation_resolution', 0.1
        )

        self._node.get_logger().info(
            f'[{name}] StraightLinePlanner configured '
            f'(resolution={self._resolution}m)'
        )

    def declare_or_get_parameter(self, name, default_value):
        """
        Declare a parameter if not already declared, or get its value if it exists.

        Mirrors ``nav2_planner_py.parameter_handler.declare_or_get_parameter``:
        returns the current value if the parameter already exists, otherwise
        declares it with *default_value* and returns the (possibly overridden)
        value applied from the parameter YAML.

        Parameters
        ----------
        name : str
            Fully-qualified parameter name.
        default_value
            Default value used if the parameter needs to be declared.

        Returns
        -------
        The current or default value of the parameter.

        """
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        try:
            return self._node.declare_parameter(name, default_value).value
        except Exception as ex:  # noqa: BLE001
            self._node.get_logger().warning(
                f"Could not declare parameter '{name}' with default value "
                f"'{default_value}': {ex}. Using default value."
            )
            return default_value

    def cleanup(self):
        self._node.get_logger().info(f'[{self._name}] Cleaning up')

    def activate(self):
        self._node.get_logger().info(f'[{self._name}] Activating')

    def deactivate(self):
        self._node.get_logger().info(f'[{self._name}] Deactivating')

    def create_plan(
        self,
        start: PoseStamped,
        goal: PoseStamped,
        viapoints: List[PoseStamped],
        cancel_checker,
    ) -> Path:
        path = Path()
        path.header.frame_id = goal.header.frame_id

        x0 = start.pose.position.x
        y0 = start.pose.position.y
        x1 = goal.pose.position.x
        y1 = goal.pose.position.y

        dist = math.hypot(x1 - x0, y1 - y0)
        if dist < 1e-6:
            path.poses.append(goal)
            return path

        steps = max(int(dist / self._resolution), 1)
        for i in range(steps + 1):
            if cancel_checker():
                break
            t = i / steps
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x0 + t * (x1 - x0)
            pose.pose.position.y = y0 + t * (y1 - y0)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        print(f'[{self._name}] Created straight-line path with {len(path.poses)} poses')
        return path
