#pragma once
#include <cmath>
#include <Eigen/Eigenvalues>
#include <cstddef>
#include <limits>
#include <boost/math/distributions/chi_squared.hpp>
#include <string>
namespace liuo_tunnel {
struct EstimatorStatus {
  std::string state, reason;
};
// Post-fit measurement residual reference, NOT predictive NIS or a calibrated
// nonlinear false-alarm guarantee. It never changes a factor or its weight.
inline double postfit_range_limit(size_t samples) {
  return samples ? boost::math::quantile(boost::math::chi_squared(double(samples)), .999) : std::numeric_limits<double>::infinity();
}
// A small post-fit residual cannot certify recovery from a failed window.
// Keep the first failure until the owning estimator starts a new session.
inline double recovery_failure_stamp(double previous, double stamp, double recovery_until, size_t samples, double squared_sum) {
  if (previous >= 0) return previous;
  (void)recovery_until;  // Receipt/timer windows do not limit actual failure memory.
  if (samples && (!std::isfinite(squared_sum) || squared_sum < 0 || squared_sum > postfit_range_limit(samples))) return stamp;
  return previous;
}
// Fixed-map/fixed-attitude local range-only bound. This deliberately optimistic
// diagnostic is NOT the estimator covariance or a global branch certificate.
inline double optimistic_position_trace(const Eigen::Matrix3d& information) {
  if (!information.allFinite()) return std::numeric_limits<double>::infinity();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(information);
  if (eig.info() != Eigen::Success || eig.eigenvalues()[2] <= 0 || eig.eigenvalues()[0] <= 1e-10 * eig.eigenvalues()[2]) return std::numeric_limits<double>::infinity();
  return eig.eigenvalues().cwiseInverse().sum();
}
// Operational diagnostics only. Local posterior covariance is not a calibrated
// accuracy guarantee; never use simulation truth to select the online state.
inline EstimatorStatus fused_status(
  double stamp,
  double last_accepted,
  double last_rejected,
  const std::string& rejection,
  double position_trace,
  double pose_trace,
  double recovery_until = -1,
  size_t postfit_samples = 0,
  double postfit_squared_sum = 0,
  double failed_recovery_stamp = -1,
  double optimistic_geometry_trace = 0) {
  if (!std::isfinite(position_trace) || !std::isfinite(pose_trace)) return {"LOST", "POSTERIOR_NONFINITE"};
  if (pose_trace > 100) return {"LOST", "POSTERIOR_UNBOUNDED"};
  if (last_accepted < 0 || stamp - last_accepted > .3) {
    if (last_rejected >= 0 && std::abs(stamp - last_rejected) <= .3) return {"DEGRADED", "UWB_REJECTED_LIO_CONTINUES:" + rejection};
    return {"DEGRADED", "UWB_ABSENT_LIO_CONTINUES"};
  }
  if (postfit_samples && (!std::isfinite(postfit_squared_sum) || postfit_squared_sum < 0)) return {"LOST", "UWB_POSTFIT_NONFINITE_OR_INVALID"};
  if (postfit_samples && postfit_squared_sum > postfit_range_limit(postfit_samples)) return {"LOST", "UWB_POSTFIT_INCONSISTENT"};
  if (failed_recovery_stamp >= 0) return {"DEGRADED", "UWB_RECOVERY_UNVERIFIED"};
  if (position_trace > .25) return {"DEGRADED", "POSITION_POSTERIOR_WEAK"};
  if (stamp < recovery_until) return {"DEGRADED", "UWB_RECOVERING_WINDOW"};
  // Elapsed time and a small local posterior do not verify global recovery.
  // There is currently no independent recovery certificate; session reset
  // clears this history explicitly. Keep the limitation visible.
  if (recovery_until >= 0) return {"DEGRADED", "UWB_RECOVERY_UNVERIFIED"};
  if (!std::isfinite(optimistic_geometry_trace) || optimistic_geometry_trace > .25) return {"DEGRADED", "UWB_POSITION_GEOMETRY_WEAK"};
  return {"TRACKING", "JOINT_LIDAR_IMU_UWB"};
}
}  // namespace liuo_tunnel
