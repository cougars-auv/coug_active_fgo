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
 * @file belief_state_critic.cpp
 * @brief Implementation of the BeliefStateCritic.
 * @author Nelson Durrant
 * @date Mar 2026
 */

#include "coug_active_fgo/belief_state_critic.hpp"

#include <omp.h>

#include <cmath>

namespace mppi::critics {

void BeliefStateCritic::initialize() {
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 5.0);

  auto toDiagCov = [](const std::vector<double>& sigmas) {
    return Eigen::Vector3d::Map(sigmas.data()).cwiseAbs2().asDiagonal();
  };

  std::vector<double> accel_noise_sigmas, gyro_noise_sigmas;
  std::vector<double> accel_bias_rw_sigmas, gyro_bias_rw_sigmas;

  getParam(accel_noise_sigmas, "accel_noise_sigmas", std::vector<double>{0.0078, 0.0078, 0.0078});
  getParam(gyro_noise_sigmas, "gyro_noise_sigmas", std::vector<double>{0.0012, 0.0012, 0.0012});
  getParam(accel_bias_rw_sigmas, "accel_bias_rw_sigmas",
           std::vector<double>{0.000105, 0.000105, 0.000105});
  getParam(gyro_bias_rw_sigmas, "gyro_bias_rw_sigmas",
           std::vector<double>{0.0000391, 0.0000391, 0.0000391});
  getParam(integration_covariance_, "integration_covariance", 0.00001);

  Q_gyro_cov_ = toDiagCov(gyro_noise_sigmas);
  Q_accel_cov_ = toDiagCov(accel_noise_sigmas);
  Q_accel_bias_cov_ = toDiagCov(accel_bias_rw_sigmas);
  Q_gyro_bias_cov_ = toDiagCov(gyro_bias_rw_sigmas);

  std::vector<double> velocity_noise_sigmas;
  double yaw_noise_sigma;

  getParam(dvl_update_rate_, "dvl_update_rate", 10.0);
  getParam(ahrs_update_rate_, "ahrs_update_rate", 50.0);
  getParam(velocity_noise_sigmas, "velocity_noise_sigmas", std::vector<double>{0.02, 0.02, 0.02});
  getParam(yaw_noise_sigma, "yaw_noise_sigma", 0.01745);

  R_dvl_ = toDiagCov(velocity_noise_sigmas);
  R_ahrs_ = Eigen::Matrix<double, 1, 1>::Constant(std::pow(yaw_noise_sigma, 2));

  getParam(fgo_odom_topic_, "fgo_odom_topic", std::string("odometry/global"));
  getParam(fgo_vel_topic_, "fgo_vel_topic", std::string("factor_graph_node/velocity"));
  getParam(fgo_bias_topic_, "fgo_bias_topic", std::string("factor_graph_node/imu_bias"));

  auto node = parent_.lock();
  clock_ = node->get_clock();
  fgo_odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      fgo_odom_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Pose of the base frame in the map frame
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
        std::lock_guard<std::mutex> lock(sigma0_mutex_);
        Sigma0_.block<6, 6>(0, 0) = pose_cov;
        received_odom_.store(true);
      });

  fgo_velocity_sub_ = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      fgo_vel_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        // Velocity of the target frame in the map frame
        // For simplicity, we assume it's at the base frame here
        const auto& c = msg->twist.covariance;
        Eigen::Matrix<double, 3, 3> vel_cov;
        for (int i = 0; i < 3; ++i) {
          for (int j = 0; j < 3; ++j) {
            vel_cov(i, j) = c[i * 6 + j];
          }
        }
        std::lock_guard<std::mutex> lock(sigma0_mutex_);
        Sigma0_.block<3, 3>(6, 6) = vel_cov;
        received_vel_.store(true);
      });

  fgo_bias_sub_ = node->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      fgo_bias_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
        const auto& c = msg->twist.covariance;
        Eigen::Matrix<double, 6, 6> bias_cov;
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) {
            bias_cov(i, j) = c[i * 6 + j];
          }
        }
        std::lock_guard<std::mutex> lock(sigma0_mutex_);
        Sigma0_.block<6, 6>(9, 9) = bias_cov;
        received_bias_.store(true);
      });

  RCLCPP_INFO(logger_, "BeliefStateCritic initialized.");
}

void BeliefStateCritic::score(CriticData& data) {
  if (!enabled_) {
    return;
  }
  if (!received_odom_.load() || !received_vel_.load() || !received_bias_.load()) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "BeliefStateCritic waiting for FGO messages...");
    return;
  }

  const size_t batch_size = data.trajectories.x.shape(0);
  const size_t time_steps = data.trajectories.x.shape(1);
  const double dt = static_cast<double>(data.model_dt);

  Eigen::Matrix<double, 15, 15> Sigma0;
  Eigen::Matrix<double, 6, 6> Sigma0_bias_inv;
  {
    std::lock_guard<std::mutex> lock(sigma0_mutex_);
    Sigma0 = Sigma0_;
    Sigma0_bias_inv = Sigma0_.block<6, 6>(9, 9).inverse();
  }

  const double dt2 = dt * dt;
  Eigen::Matrix<double, 15, 15> Q_step = Eigen::Matrix<double, 15, 15>::Zero();
  Q_step.block<3, 3>(0, 0) = Q_gyro_cov_ * dt;
  Q_step.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * integration_covariance_ * dt;
  Q_step.block<3, 3>(6, 6) = Q_accel_cov_ * dt;
  Q_step.block<3, 3>(9, 9) = Q_accel_bias_cov_ * dt;
  Q_step.block<3, 3>(12, 12) = Q_gyro_bias_cov_ * dt;

  const Eigen::Matrix<double, 15, 15> I_15 = Eigen::Matrix<double, 15, 15>::Identity();
  const Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix<double, 1, 15> H_ahrs = Eigen::Matrix<double, 1, 15>::Zero();
  H_ahrs(0, 2) = 1.0;

  const double dvl_period = 1.0 / dvl_update_rate_;
  const double ahrs_period = 1.0 / ahrs_update_rate_;

  // Iterate through each rollout in the batch
#pragma omp parallel for schedule(static)
  for (size_t i = 0; i < batch_size; ++i) {
    Eigen::Matrix<double, 15, 15> Sigma_i = Sigma0;

    double t_last_dvl = 0.0;
    double t_last_ahrs = 0.0;
    double curr_sim_time = 0.0;

    for (size_t t = 0; t < time_steps; ++t) {
      const double psi = static_cast<double>(data.trajectories.yaws(i, t));
      const double cpsi = std::cos(psi);
      const double spsi = std::sin(psi);
      curr_sim_time += dt;

      // --- PREDICTION STEP ---
      Eigen::Matrix3d Rz;
      Rz << cpsi, -spsi, 0, spsi, cpsi, 0, 0, 0, 1;

      // Estimate map frame acceleration
      Eigen::Vector3d a_map = Eigen::Vector3d::Zero();
      if (t + 1 < time_steps) {
        const double psi_n = static_cast<double>(data.trajectories.yaws(i, t + 1));
        const double vx_b = static_cast<double>(data.state.vx(i, t));
        const double vy_b = static_cast<double>(data.state.vy(i, t));
        const double vx_bn = static_cast<double>(data.state.vx(i, t + 1));
        const double vy_bn = static_cast<double>(data.state.vy(i, t + 1));
        const Eigen::Vector3d v_map(cpsi * vx_b - spsi * vy_b, spsi * vx_b + cpsi * vy_b, 0.0);
        const Eigen::Vector3d v_map_n(std::cos(psi_n) * vx_bn - std::sin(psi_n) * vy_bn,
                                      std::sin(psi_n) * vx_bn + std::cos(psi_n) * vy_bn, 0.0);
        a_map = (v_map_n - v_map) / dt;
      }

      Eigen::Matrix<double, 15, 15> F = I_15;
      F.block<3, 3>(3, 6) = I_3 * dt;         // d(pos)/d(vel)
      F.block<3, 3>(6, 9) = -Rz * dt;         // d(vel)/d(accel_bias)
      F.block<3, 3>(3, 9) = -Rz * 0.5 * dt2;  // d(pos)/d(accel_bias)
      F.block<3, 3>(0, 12) = -I_3 * dt;       // d(orientation)/d(gyro_bias)
      F.block<3, 1>(6, 2) =                   // d(vel)/d(yaw)
          (Eigen::Matrix3d() << -spsi, -cpsi, 0, cpsi, -spsi, 0, 0, 0, 0).finished() *
          (Rz.transpose() * a_map) * dt;

      // Σ_i = F * Σ_i * F^T + Q
      Sigma_i = F * Sigma_i * F.transpose() + Q_step;

      // --- UPDATE STEPS ---
      const bool trigger_dvl = (curr_sim_time - t_last_dvl) >= dvl_period;
      const bool trigger_ahrs = (curr_sim_time - t_last_ahrs) >= ahrs_period;

      if (trigger_dvl) {
        // Maps map frame velocity to base frame DVL measurements
        Eigen::Matrix<double, 3, 15> H_dvl = Eigen::Matrix<double, 3, 15>::Zero();
        H_dvl.block<3, 3>(0, 6) << cpsi, spsi, 0.0, -spsi, cpsi, 0.0, 0.0, 0.0, 1.0;

        // K = Σ_i * H^T * (H * Σ_i * H^T + R)^-1
        auto K =
            Sigma_i * H_dvl.transpose() * (H_dvl * Sigma_i * H_dvl.transpose() + R_dvl_).inverse();

        // Σ_i = (I - K*H) * Σ_i
        Sigma_i = (I_15 - K * H_dvl) * Sigma_i;
        t_last_dvl = curr_sim_time;
      }

      if (trigger_ahrs) {
        // K = Σ_i * H^T * (H * Σ_i * H^T + R)^-1
        auto K = Sigma_i * H_ahrs.transpose() *
                 (H_ahrs * Sigma_i * H_ahrs.transpose() + R_ahrs_).inverse();

        // Σ_i = (I - K*H) * Σ_i
        Sigma_i = (I_15 - K * H_ahrs) * Sigma_i;
        t_last_ahrs = curr_sim_time;
      }
    }
    // Compute the normalized trace over the IMU bias covariance block
    const float norm_trace =
        static_cast<float>((Sigma0_bias_inv * Sigma_i.block<6, 6>(9, 9)).trace());
    data.costs(i) += std::pow(weight_ * norm_trace, power_);
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::BeliefStateCritic, mppi::critics::CriticFunction)
