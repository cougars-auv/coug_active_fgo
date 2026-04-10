// Copyright (c) 2026 BYU FROST Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file waypoint_nav2.hpp
 * @brief ROS 2 node for relaying waypoints to the Nav2 action server.
 * @author Nelson Durrant
 * @date Mar 2026
 */

#pragma once

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>
#include <vector>

#include "coug_active_fgo/waypoint_nav2_parameters.hpp"

namespace coug_active_fgo {

/**
 * @class WaypointNav2Node
 * @brief Nav2 mission bridge for AUV waypoint navigation.
 *
 * This node listens for a list of waypoints and forwards them to
 * the Nav2 FollowWaypoints action server.
 */
class WaypointNav2Node : public rclcpp::Node {
 public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  /**
   * @brief WaypointNav2Node constructor.
   * @param options The node options.
   */
  explicit WaypointNav2Node(const rclcpp::NodeOptions& options);

 protected:
  // --- Logic ---
  /**
   * @brief Callback for receiving a new list of waypoints.
   * @param msg The incoming PoseArray message containing waypoints.
   */
  void waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  /**
   * @brief Callback for handling the completion status from Nav2.
   * @param result The result from the action server.
   */
  void resultCallback(const GoalHandleFollowWaypoints::WrappedResult& result);

  // --- ROS Interfaces ---
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr nav2_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoint_sub_;

  // --- Parameters ---
  std::shared_ptr<waypoint_nav2_node::ParamListener> param_listener_;
  waypoint_nav2_node::Params params_;
};

}  // namespace coug_active_fgo
