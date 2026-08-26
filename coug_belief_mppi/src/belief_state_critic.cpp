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

#include "coug_belief_mppi/belief_state_critic.hpp"

#include <omp.h>

#include <cmath>

namespace mppi::critics {

void BeliefStateCritic::initialize() {
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);

  auto to_diag_cov = [](const std::vector<double>& sigmas) {
    return Eigen::Vector3d::Map(sigmas.data()).cwiseAbs2().asDiagonal();
  };

  std::vector<double> accel_noise_sigmas, gyro_noise_sigmas;
  std::vector<double> accel_bias_rw_sigmas, gyro_bias_rw_sigmas;

  getParam(accel_noise_sigmas, "accel_noise_sigmas", std::vector<double>{5.6e-4, 5.6e-4, 5.6e-4});
  getParam(gyro_noise_sigmas, "gyro_noise_sigmas", std::vector<double>{5.24e-5, 5.24e-5, 5.24e-5});
  getParam(accel_bias_rw_sigmas, "accel_bias_rw_sigmas",
           std::vector<double>{1.4e-5, 1.4e-5, 1.4e-5});
  getParam(gyro_bias_rw_sigmas, "gyro_bias_rw_sigmas", std::vector<double>{3.5e-6, 3.5e-6, 3.5e-6});
  getParam(integration_covariance_, "integration_covariance", 1.0e-8);

  gyro_noise_cov_ = to_diag_cov(gyro_noise_sigmas);
  accel_noise_cov_ = to_diag_cov(accel_noise_sigmas);
  accel_bias_rw_cov_ = to_diag_cov(accel_bias_rw_sigmas);
  gyro_bias_rw_cov_ = to_diag_cov(gyro_bias_rw_sigmas);

  std::vector<double> velocity_noise_sigmas;
  double yaw_noise_sigma;

  getParam(dvl_update_rate_hz_, "dvl_update_rate_hz", 10.0);
  getParam(ahrs_update_rate_hz_, "ahrs_update_rate_hz", 10.0);
  getParam(velocity_noise_sigmas, "velocity_noise_sigmas", std::vector<double>{0.02, 0.02, 0.02});
  getParam(yaw_noise_sigma, "yaw_noise_sigma", 0.01745);

  dvl_noise_cov_ = to_diag_cov(velocity_noise_sigmas);
  ahrs_noise_cov_ = Eigen::Matrix<double, 1, 1>::Constant(std::pow(yaw_noise_sigma, 2));

  getParam(fg_odom_topic_, "fg_odom_topic", std::string("odometry/global"));
  getParam(fg_vel_topic_, "fg_vel_topic", std::string("factor_graph_node/velocity"));
  getParam(fg_bias_topic_, "fg_bias_topic", std::string("factor_graph_node/imu/bias"));

  auto node = parent_.lock();
  clock_ = node->get_clock();
  fg_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      fg_odom_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Pose of the base frame in the map frame
        const auto& cov_msg = msg->pose.covariance;
        Eigen::Matrix<double, 6, 6> pose_cov;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            pose_cov(i, j) = cov_msg[(i + 3) * 6 + (j + 3)];
            pose_cov(i + 3, j + 3) = cov_msg[i * 6 + j];
            pose_cov(i, j + 3) = cov_msg[(i + 3) * 6 + j];
            pose_cov(i + 3, j) = cov_msg[i * 6 + (j + 3)];
          }
        }
        std::lock_guard<std::mutex> lock(state_cov_mutex_);
        init_state_cov_.block<6, 6>(0, 0) = pose_cov;
        received_odom_.store(true);
      });

  fg_vel_sub_ = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      fg_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        // Velocity of the target frame in the map frame
        // For simplicity, we assume it's at the base frame here
        const auto& cov_msg = msg->twist.covariance;
        Eigen::Matrix3d vel_cov;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            vel_cov(i, j) = cov_msg[i * 6 + j];
          }
        }
        std::lock_guard<std::mutex> lock(state_cov_mutex_);
        init_state_cov_.block<3, 3>(6, 6) = vel_cov;
        received_vel_.store(true);
      });

  fg_bias_sub_ = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      fg_bias_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        const auto& cov_msg = msg->twist.covariance;
        Eigen::Matrix<double, 6, 6> bias_cov;
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) {
            bias_cov(i, j) = cov_msg[i * 6 + j];
          }
        }
        std::lock_guard<std::mutex> lock(state_cov_mutex_);
        init_state_cov_.block<6, 6>(9, 9) = bias_cov;
        received_bias_.store(true);
      });

  RCLCPP_INFO(logger_, "Initialization complete.");
}

void BeliefStateCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }
  if (!received_odom_.load() || !received_vel_.load() || !received_bias_.load()) {
    return;
  }

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  const double dt = static_cast<double>(data.model_dt);

  Eigen::Matrix<double, 15, 15> init_cov;
  Eigen::Matrix<double, 6, 6> init_bias_cov_inv;
  {
    std::lock_guard<std::mutex> lock(state_cov_mutex_);
    init_cov = init_state_cov_;
    init_bias_cov_inv = init_state_cov_.block<6, 6>(9, 9).inverse();
  }

  const double dt_sq = dt * dt;
  Eigen::Matrix<double, 15, 15> process_noise_cov = Eigen::Matrix<double, 15, 15>::Zero();
  process_noise_cov.block<3, 3>(0, 0) = gyro_noise_cov_ * dt;
  process_noise_cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * integration_covariance_ * dt;
  process_noise_cov.block<3, 3>(6, 6) = accel_noise_cov_ * dt;
  process_noise_cov.block<3, 3>(9, 9) = accel_bias_rw_cov_ * dt;
  process_noise_cov.block<3, 3>(12, 12) = gyro_bias_rw_cov_ * dt;

  const Eigen::Matrix<double, 15, 15> I_15x15 = Eigen::Matrix<double, 15, 15>::Identity();
  const Eigen::Matrix3d I_3x3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 1, 15> J_ahrs_state = Eigen::Matrix<double, 1, 15>::Zero();
  J_ahrs_state(0, 2) = 1.0;

  const double dvl_period = 1.0 / dvl_update_rate_hz_;
  const double ahrs_period = 1.0 / ahrs_update_rate_hz_;

  // Iterate through each rollout in the batch
#pragma omp parallel for schedule(static)
  for (size_t batch_idx = 0; batch_idx < batch_size; ++batch_idx) {
    Eigen::Matrix<double, 15, 15> rollout_cov = init_cov;

    double last_dvl_time = 0.0;
    double last_ahrs_time = 0.0;
    double curr_sim_time = 0.0;

    for (size_t step = 0; step < time_steps; ++step) {
      const double yaw = static_cast<double>(data.trajectories.yaws(batch_idx, step));
      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      curr_sim_time += dt;

      // --- PREDICTION STEP ---
      Eigen::Matrix3d map_R_base;
      map_R_base << cos_yaw, -sin_yaw, 0, sin_yaw, cos_yaw, 0, 0, 0, 1;

      // Estimate map-frame acceleration
      Eigen::Vector3d map_a_base = Eigen::Vector3d::Zero();
      if (step + 1 < time_steps) {
        const double next_yaw = static_cast<double>(data.trajectories.yaws(batch_idx, step + 1));
        const double base_vx = static_cast<double>(data.state.vx(batch_idx, step));
        const double base_vy = static_cast<double>(data.state.vy(batch_idx, step));
        const double next_base_vx = static_cast<double>(data.state.vx(batch_idx, step + 1));
        const double next_base_vy = static_cast<double>(data.state.vy(batch_idx, step + 1));
        const Eigen::Vector3d map_v_base(cos_yaw * base_vx - sin_yaw * base_vy,
                                         sin_yaw * base_vx + cos_yaw * base_vy, 0.0);
        const Eigen::Vector3d next_map_v_base(
            std::cos(next_yaw) * next_base_vx - std::sin(next_yaw) * next_base_vy,
            std::sin(next_yaw) * next_base_vx + std::cos(next_yaw) * next_base_vy, 0.0);
        map_a_base = (next_map_v_base - map_v_base) / dt;
      }

      Eigen::Matrix<double, 15, 15> J_state_prev = I_15x15;
      J_state_prev.block<3, 3>(3, 6) = I_3x3 * dt;                 // d(pos)/d(vel)
      J_state_prev.block<3, 3>(6, 9) = -map_R_base * dt;           // d(vel)/d(accel_bias)
      J_state_prev.block<3, 3>(3, 9) = -map_R_base * 0.5 * dt_sq;  // d(pos)/d(accel_bias)
      J_state_prev.block<3, 3>(0, 12) = -I_3x3 * dt;               // d(orientation)/d(gyro_bias)
      J_state_prev.block<3, 1>(6, 2) =                             // d(vel)/d(yaw)
          (Eigen::Matrix3d() << -sin_yaw, -cos_yaw, 0, cos_yaw, -sin_yaw, 0, 0, 0, 0).finished() *
          (map_R_base.transpose() * map_a_base) * dt;

      // rollout_cov = J * rollout_cov * J^T + Q
      rollout_cov = J_state_prev * rollout_cov * J_state_prev.transpose() + process_noise_cov;

      // --- UPDATE STEPS ---
      const bool trigger_dvl = (curr_sim_time - last_dvl_time) >= dvl_period;
      const bool trigger_ahrs = (curr_sim_time - last_ahrs_time) >= ahrs_period;

      if (trigger_dvl) {
        // Maps map-frame velocity to base-frame DVL measurements
        Eigen::Matrix<double, 3, 15> J_dvl_state = Eigen::Matrix<double, 3, 15>::Zero();
        J_dvl_state.block<3, 3>(0, 6) << cos_yaw, sin_yaw, 0.0, -sin_yaw, cos_yaw, 0.0, 0.0, 0.0,
            1.0;

        // kalman_gain = rollout_cov * J^T * (J * rollout_cov * J^T + R)^-1
        const Eigen::Matrix<double, 15, 3> kalman_gain =
            rollout_cov * J_dvl_state.transpose() *
            (J_dvl_state * rollout_cov * J_dvl_state.transpose() + dvl_noise_cov_).inverse();

        // rollout_cov = (I - kalman_gain * J) * rollout_cov
        rollout_cov = (I_15x15 - kalman_gain * J_dvl_state) * rollout_cov;
        last_dvl_time = curr_sim_time;
      }

      if (trigger_ahrs) {
        // kalman_gain = rollout_cov * J^T * (J * rollout_cov * J^T + R)^-1
        const Eigen::Matrix<double, 15, 1> kalman_gain =
            rollout_cov * J_ahrs_state.transpose() *
            (J_ahrs_state * rollout_cov * J_ahrs_state.transpose() + ahrs_noise_cov_).inverse();

        // rollout_cov = (I - kalman_gain * J) * rollout_cov
        rollout_cov = (I_15x15 - kalman_gain * J_ahrs_state) * rollout_cov;
        last_ahrs_time = curr_sim_time;
      }
    }
    // Compute the normalized trace over the IMU bias covariance block
    const float norm_trace =
        static_cast<float>((init_bias_cov_inv * rollout_cov.block<6, 6>(9, 9)).trace());
    data.costs(batch_idx) += std::pow(weight_ * norm_trace, power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::BeliefStateCritic, mppi::critics::CriticFunction)
