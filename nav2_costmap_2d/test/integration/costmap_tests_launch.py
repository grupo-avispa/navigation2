#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
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

import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv: list[str] = sys.argv[1:]):  # type: ignore[no-untyped-def]
    launchFile = os.path.join(
        os.getenv('TEST_LAUNCH_DIR', ''), 'costmap_map_server.launch.py'
    )
    testExecutable = os.getenv('TEST_EXECUTABLE', '')

    map_to_odom = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'odom'
        ],
    )

    odom_to_base_link = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'odom',
            '--child-frame-id', 'base_link'
        ],
    )

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server']}, {'autostart': True}],
    )

    ld = LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource([launchFile])),
            map_to_odom,
            lifecycle_manager,
        ]
    )
    if os.getenv('STATIC_ODOM_TO_BASE_LINK') == 'true':
        ld.add_action(odom_to_base_link)

    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name='costmap_tests',
        output='screen'
    )

    lts = LaunchTestService()  # type: ignore[no-untyped-call]
    lts.add_test_action(ld, test1_action)  # type: ignore[no-untyped-call]
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)  # type: ignore[no-untyped-call]


if __name__ == '__main__':
    sys.exit(main())
