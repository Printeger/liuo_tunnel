#pragma once
#include <cmath>
#include <string>
namespace liuo_tunnel {
struct EstimatorStatus {
  std::string state, reason;
};
// Operational diagnostics only. Local posterior covariance is not a calibrated
// accuracy guarantee; never use simulation truth to select the online state.
inline EstimatorStatus
fused_status(double stamp, double last_accepted, double last_rejected, const std::string& rejection, double position_trace, double pose_trace, double recovery_until = -1) {
  if (!std::isfinite(position_trace) || !std::isfinite(pose_trace)) return {"LOST", "POSTERIOR_NONFINITE"};
  if (pose_trace > 100) return {"LOST", "POSTERIOR_UNBOUNDED"};
  if (last_accepted < 0 || stamp - last_accepted > .3) {
    if (last_rejected >= 0 && std::abs(stamp - last_rejected) <= .3) return {"DEGRADED", "UWB_REJECTED_LIO_CONTINUES:" + rejection};
    return {"DEGRADED", "UWB_ABSENT_LIO_CONTINUES"};
  }
  if (position_trace > .25) return {"DEGRADED", "POSITION_POSTERIOR_WEAK"};
  if (stamp < recovery_until) return {"DEGRADED", "UWB_RECOVERING_WINDOW"};
  return {"TRACKING", "JOINT_LIDAR_IMU_UWB"};
}
}  // namespace liuo_tunnel
