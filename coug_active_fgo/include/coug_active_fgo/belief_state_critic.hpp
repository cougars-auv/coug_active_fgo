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
 * @file belief_state_critic.hpp
 * @brief MPPI critic plugin that penalizes state estimation uncertainty.
 * @author Nelson Durrant
 * @date Mar 2026
 */

#pragma once

#include <Eigen/Dense>
#include <atomic>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace mppi::critics {

/**
 * @class mppi::critics::BeliefStateCritic
 * @brief Critic objective function for uncertainty-aware AUV navigation.
 *
 * This plugin propagates a 15-state EKF covariance along each trajectory rollout. It penalizes
 * trajectories based on the normalized trace of the final covariance.
 */
class BeliefStateCritic : public CriticFunction {
 public:
  /**
   * @brief Initialize critic
   */
  void initialize() override;

  /**
   * @brief Evaluate cost related to state estimation uncertainty
   *
   * @param costs [out] add covariance trace cost values to this tensor
   */
  void score(CriticData& data) override;

 protected:
  // --- Parameters ---
  unsigned int power_{0};
  float weight_{0};
  float dvl_update_rate_{0};
  float ahrs_update_rate_{0};
  Eigen::Matrix<double, 3, 3> R_dvl_;
  Eigen::Matrix<double, 1, 1> R_ahrs_;
  Eigen::Matrix<double, 3, 3> Q_gyro_cov_;
  Eigen::Matrix<double, 3, 3> Q_accel_cov_;
  Eigen::Matrix<double, 3, 3> Q_accel_bias_cov_;
  Eigen::Matrix<double, 3, 3> Q_gyro_bias_cov_;
  double integration_covariance_{0};

  // --- ROS Interfaces ---
  std::string fgo_odom_topic_;
  std::string fgo_vel_topic_;
  std::string fgo_bias_topic_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fgo_odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fgo_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fgo_bias_sub_;
  rclcpp::Clock::SharedPtr clock_;

  // --- State ---
  mutable std::mutex sigma0_mutex_;
  Eigen::Matrix<double, 15, 15> Sigma0_{Eigen::Matrix<double, 15, 15>::Identity()};
  std::atomic<bool> received_odom_{false};
  std::atomic<bool> received_vel_{false};
  std::atomic<bool> received_bias_{false};
};

}  // namespace mppi::critics
