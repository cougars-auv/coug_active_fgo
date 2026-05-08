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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)


def generate_launch_description() -> LaunchDescription:

    use_sim_time = LaunchConfiguration("use_sim_time")
    auv_ns = LaunchConfiguration("auv_ns")

    fleet_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_FOLDER"),
            "fleet",
            "coug_active_fgo_params.yaml",
        ]
    )
    auv_params = PathJoinSubstitution(
        [
            EnvironmentVariable("CONFIG_FOLDER"),
            PythonExpression(["'", auv_ns, "' + '_params.yaml'"]),
        ]
    )

    odom_frame = PythonExpression(
        ["'", auv_ns, "/odom' if '", auv_ns, "' != '' else 'odom'"]
    )

    base_link_frame = PythonExpression(
        [
            "'",
            auv_ns,
            "/base_link' if '",
            auv_ns,
            "' != '' else 'base_link'",
        ]
    )

    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/rosbag clock if true",
            ),
            DeclareLaunchArgument(
                "auv_ns",
                default_value="auv0",
                description="Namespace for the AUV (e.g. auv0)",
            ),
            Node(
                package="coug_active_fgo",
                executable="waypoint_nav2",
                name="waypoint_nav2_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="coug_active_fgo",
                executable="belief_state_monitor",
                name="belief_state_monitor_node",
                parameters=[
                    fleet_params,
                    auv_params,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                name="twist_mux",
                parameters=[fleet_params, auv_params, {"use_sim_time": use_sim_time}],
            ),
            # --- Navigation2 Pipeline ---
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[fleet_params, auv_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[fleet_params, auv_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[
                    fleet_params,
                    auv_params,
                    {
                        "use_sim_time": use_sim_time,
                        "global_frame": odom_frame,
                        "robot_base_frame": base_link_frame,
                    },
                ],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[
                    fleet_params,
                    auv_params,
                    {
                        "use_sim_time": use_sim_time,
                        "global_frame": "map",
                        "robot_base_frame": base_link_frame,
                    },
                ],
            ),
            Node(
                package="nav2_waypoint_follower",
                executable="waypoint_follower",
                name="waypoint_follower",
                parameters=[fleet_params, auv_params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": True},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
