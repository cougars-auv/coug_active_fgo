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
 * @file waypoint_nav2.cpp
 * @brief Implementation of the WaypointNav2Node.
 * @author Nelson Durrant
 * @date Mar 2026
 */

#include "coug_active_fgo/waypoint_nav2.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_active_fgo
{

WaypointNav2Node::WaypointNav2Node(const rclcpp::NodeOptions & options)
: Node("waypoint_nav2_node", options)
{
  RCLCPP_INFO(get_logger(), "Starting Waypoint Nav2 Node...");

  param_listener_ = std::make_shared<waypoint_nav2_node::ParamListener>(
    get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // --- ROS Interfaces ---
  nav2_client_ = rclcpp_action::create_client<FollowWaypoints>(
    this,
    "follow_waypoints");

  waypoint_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
    params_.waypoint_topic, 10,
    std::bind(&WaypointNav2Node::waypointCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for mission...");
}

void WaypointNav2Node::waypointCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty mission. Canceling mission...");
    nav2_client_->async_cancel_all_goals();
    return;
  }

  if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_logger(), "Nav2 FollowWaypoints action server not available.");
    return;
  }

  auto goal_msg = FollowWaypoints::Goal();

  std::string frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;

  for (const auto & pose : msg->poses) {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = get_clock()->now();
    pose_stamped.pose = pose;

    goal_msg.poses.push_back(pose_stamped);
  }

  RCLCPP_INFO(get_logger(), "Sending goal with %zu waypoints to Nav2...", goal_msg.poses.size());

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&WaypointNav2Node::resultCallback, this, std::placeholders::_1);

  nav2_client_->async_send_goal(goal_msg, send_goal_options);
}

void WaypointNav2Node::resultCallback(const GoalHandleFollowWaypoints::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Nav2 successfully reached all waypoints!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Nav2 aborted the mission.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "Nav2 mission was canceled.");
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code from Nav2.");
      break;
  }
}

}  // namespace coug_active_fgo

RCLCPP_COMPONENTS_REGISTER_NODE(coug_active_fgo::WaypointNav2Node)
