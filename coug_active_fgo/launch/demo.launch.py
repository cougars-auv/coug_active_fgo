# Copyright (c) 2026 BYU FROST Lab
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
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    LogInfo,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs) -> list:

    use_sim_time = LaunchConfiguration("use_sim_time")
    start_delay = LaunchConfiguration("start_delay")
    auv_ns = LaunchConfiguration("auv_ns")
    bag_path = LaunchConfiguration("bag_path")

    bag_path_str = context.perform_substitution(bag_path)
    auv_ns_str = context.perform_substitution(auv_ns)

    coug_active_fgo_dir = get_package_share_directory("coug_active_fgo")
    coug_active_fgo_launch_dir = os.path.join(coug_active_fgo_dir, "launch")

    actions = []

    if bag_path_str:
        play_process = ExecuteProcess(
            cmd=[
                "ros2",
                "bag",
                "play",
                bag_path_str,
                "--clock",
                "--start-offset",
                start_delay,
                "--topics",
                "/tf",
                "/tf_static",
                f"/{auv_ns_str}/robot_description",
                f"/{auv_ns_str}/odometry/truth",
                f"/{auv_ns_str}/odometry/global",
                f"/{auv_ns_str}/smoothed_path",
                f"/{auv_ns_str}/waypoints",
                f"/{auv_ns_str}/belief_state_monitor_node/norm_trace",
                f"/{auv_ns_str}/factor_graph_node/velocity",
                f"/{auv_ns_str}/factor_graph_node/imu_bias",
            ],
        )
        actions.append(play_process)

        actions.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=play_process,
                    on_exit=LogInfo(msg="Bag playback finished."),
                )
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    coug_active_fgo_launch_dir, "coug_active_fgo_viz.launch.py"
                )
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "auv_ns": auv_ns,
                "launch_rviz": "true",
                "launch_plotjuggler": "true",
            }.items(),
        )
    )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="0.0",
                description=(
                    "Time in seconds to skip from the beginning of the bag file (start offset)"
                ),
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            DeclareLaunchArgument(
                "bag_path",
                default_value="",
                description="Path to the rosbag to play (if empty, no bag is played)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
