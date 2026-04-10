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
 * @file belief_state_monitor.hpp
 * @brief ROS 2 node that publishes the normalized covariance trace for visualization.
 * @author Nelson Durrant
 * @date Mar 2026
 */

#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "coug_active_fgo/belief_state_monitor_parameters.hpp"

namespace coug_active_fgo {

/**
 * @class BeliefStateMonitorNode
 * @brief Publishes the normalized trace for easy visualization.
 *
 * This node listens for and calculates the normalized trace of the covariance of
 * the belief state (FGO) estimate.
 */
class BeliefStateMonitorNode : public rclcpp::Node {
 public:
  /**
   * @brief BeliefStateMonitorNode constructor.
   * @param options The node options.
   */
  explicit BeliefStateMonitorNode(const rclcpp::NodeOptions& options);

 protected:
  // --- Logic ---
  /**
   * @brief Computes and publishes the normalized trace.
   */
  void publishTrace();

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr bias_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr trace_pub_;

  // --- Parameters ---
  std::shared_ptr<belief_state_monitor_node::ParamListener> param_listener_;
  belief_state_monitor_node::Params params_;

  // --- State ---
  mutable std::mutex sigma_mutex_;
  Eigen::Matrix<double, 15, 15> Sigma_{Eigen::Matrix<double, 15, 15>::Identity()};
  Eigen::Matrix<double, 6, 6> Sigma0_bias_inv_{Eigen::Matrix<double, 6, 6>::Identity()};
  std::atomic<bool> received_odom_{false};
  std::atomic<bool> received_vel_{false};
  std::atomic<bool> received_bias_{false};
  std::atomic<bool> sigma0_set_{false};
  double last_trace_{-1.0};
};

}  // namespace coug_active_fgo
