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
 * @file belief_state_monitor.cpp
 * @brief Implementation of the BeliefStateMonitorNode.
 * @author Nelson Durrant
 * @date May 2026
 */

#include "coug_active_fgo/belief_state_monitor.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace coug_active_fgo {

BeliefStateMonitorNode::BeliefStateMonitorNode(const rclcpp::NodeOptions& options)
    : Node("belief_state_monitor_node", options) {
  RCLCPP_INFO(get_logger(), "Starting Belief State Monitor Node...");

  param_listener_ =
      std::make_shared<belief_state_monitor_node::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      params_.fgo_odom_topic, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& c = msg->pose.covariance;
        Eigen::Matrix<double, 6, 6> pose_cov;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            pose_cov(i, j) = c[(i + 3) * 6 + (j + 3)];
            pose_cov(i + 3, j + 3) = c[i * 6 + j];
            pose_cov(i, j + 3) = c[(i + 3) * 6 + j];
            pose_cov(i + 3, j) = c[i * 6 + (j + 3)];
          }
        }
        std::lock_guard<std::mutex> lock(sigma_mutex_);
        Sigma_.block<6, 6>(0, 0) = pose_cov;
        received_odom_.store(true);
        publishTrace();
      });

  vel_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      params_.fgo_vel_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        const auto& c = msg->twist.covariance;
        Eigen::Matrix<double, 3, 3> vel_cov;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            vel_cov(i, j) = c[i * 6 + j];
          }
        }
        std::lock_guard<std::mutex> lock(sigma_mutex_);
        Sigma_.block<3, 3>(6, 6) = vel_cov;
        received_vel_.store(true);
        publishTrace();
      });

  bias_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      params_.fgo_bias_topic, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        const auto& c = msg->twist.covariance;
        Eigen::Matrix<double, 6, 6> bias_cov;
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) {
            bias_cov(i, j) = c[i * 6 + j];
          }
        }
        std::lock_guard<std::mutex> lock(sigma_mutex_);
        Sigma_.block<6, 6>(9, 9) = bias_cov;
        received_bias_.store(true);
        publishTrace();
      });

  trace_pub_ = create_publisher<std_msgs::msg::Float64>(params_.norm_trace_topic,
                                                        rclcpp::SystemDefaultsQoS());

  RCLCPP_INFO(get_logger(), "Startup complete! Waiting for FGO messages...");
}

void BeliefStateMonitorNode::publishTrace() {
  if (!received_odom_.load() || !received_vel_.load() || !received_bias_.load()) {
    return;
  }

  if (!sigma0_set_.load()) {
    Sigma0_bias_inv_ = Sigma_.block<6, 6>(9, 9).inverse();
    sigma0_set_.store(true);
  }

  double trace = (Sigma0_bias_inv_ * Sigma_.block<6, 6>(9, 9)).trace();
  if (trace == last_trace_) {
    return;
  }
  last_trace_ = trace;

  auto msg = std_msgs::msg::Float64();
  msg.data = trace;
  trace_pub_->publish(msg);
}

}  // namespace coug_active_fgo

RCLCPP_COMPONENTS_REGISTER_NODE(coug_active_fgo::BeliefStateMonitorNode)
