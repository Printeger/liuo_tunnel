#pragma once
#include <liuo_tunnel/common/imu_integration.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <stdexcept>

namespace liuo_tunnel {
// Conditional preprocessing, not an extra motion factor. Uses the same IMU
// preintegration convention as the joint graph. The graph's subsequent interval
// integration resets the scratch preintegrator before constructing ImuFactor.
class RollingMotion {
public:
  RollingMotion(IMUIntegration& imu, const gtsam::NavState& state, const gtsam::imuBias::ConstantBias& bias, double state_stamp, double scan_start, double scan_end)
  : start_(scan_start),
    end_(scan_end) {
    if (
      !std::isfinite(state_stamp) || !std::isfinite(start_) || !std::isfinite(end_) || start_ < state_stamp || end_ <= start_ || end_ - start_ > .10000001 ||
      end_ - state_stamp > .25000001 || !state.pose().matrix().allFinite() || !state.velocity().allFinite())
      throw std::invalid_argument("ROLLING_INVALID_INTERVAL_OR_STATE");
    const auto& samples = imu.imu_data_in_queue();
    if (samples.size() < 2 || samples.front()[0] > state_stamp + 1e-8 || samples.back()[0] < end_ - 1e-8) throw std::invalid_argument("ROLLING_IMU_COVERAGE_MISSING");
    for (size_t i = 0; i < samples.size(); ++i) {
      if (!samples[i].allFinite() || (i && samples[i][0] <= samples[i - 1][0])) throw std::invalid_argument("ROLLING_IMU_ORDER_OR_FINITE");
      if (i && samples[i][0] > state_stamp && samples[i - 1][0] < end_ && samples[i][0] - samples[i - 1][0] > .020000001) throw std::invalid_argument("ROLLING_IMU_GAP");
    }
    imu.integrate_imu(state_stamp, end_, state, bias, times_, poses_);
    reference_inverse_ = pose(start_).inverse();
  }

  Eigen::Vector3d point(const Eigen::Vector3d& raw_lidar, double relative_seconds, const Eigen::Isometry3d& T_BL) const {
    if (!std::isfinite(relative_seconds) || relative_seconds < 0 || relative_seconds >= end_ - start_ || !raw_lidar.allFinite())
      throw std::invalid_argument("ROLLING_POINT_TIME_OR_FINITE");
    return reference_inverse_ * pose(start_ + relative_seconds) * T_BL * raw_lidar;
  }

private:
  Eigen::Isometry3d pose(double stamp) const {
    auto right = std::lower_bound(times_.begin(), times_.end(), stamp);
    if (right == times_.end()) throw std::invalid_argument("ROLLING_POSE_OUT_OF_COVERAGE");
    const auto index = size_t(right - times_.begin());
    if (*right == stamp || index == 0) return poses_[index];
    const auto left = index - 1;
    const double beta = (stamp - times_[left]) / (times_[index] - times_[left]);
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = (1 - beta) * poses_[left].translation() + beta * poses_[index].translation();
    result.linear() = Eigen::Quaterniond(poses_[left].linear()).slerp(beta, Eigen::Quaterniond(poses_[index].linear())).toRotationMatrix();
    return result;
  }
  double start_, end_;
  std::vector<double> times_;
  std::vector<Eigen::Isometry3d> poses_;
  Eigen::Isometry3d reference_inverse_;
};
}  // namespace liuo_tunnel
