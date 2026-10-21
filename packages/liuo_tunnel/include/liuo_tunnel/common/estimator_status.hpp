#pragma once
#include <cmath>
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
  double postfit_squared_sum = 0) {
  if (!std::isfinite(position_trace) || !std::isfinite(pose_trace)) return {"LOST", "POSTERIOR_NONFINITE"};
  if (pose_trace > 100) return {"LOST", "POSTERIOR_UNBOUNDED"};
  if (last_accepted < 0 || stamp - last_accepted > .3) {
    if (last_rejected >= 0 && std::abs(stamp - last_rejected) <= .3) return {"DEGRADED", "UWB_REJECTED_LIO_CONTINUES:" + rejection};
    return {"DEGRADED", "UWB_ABSENT_LIO_CONTINUES"};
  }
  if (postfit_samples && (!std::isfinite(postfit_squared_sum) || postfit_squared_sum < 0)) return {"LOST", "UWB_POSTFIT_NONFINITE_OR_INVALID"};
  if (postfit_samples && postfit_squared_sum > postfit_range_limit(postfit_samples)) return {"LOST", "UWB_POSTFIT_INCONSISTENT"};
  if (position_trace > .25) return {"DEGRADED", "POSITION_POSTERIOR_WEAK"};
  if (stamp < recovery_until) return {"DEGRADED", "UWB_RECOVERING_WINDOW"};
  return {"TRACKING", "JOINT_LIDAR_IMU_UWB"};
}
}  // namespace liuo_tunnel
