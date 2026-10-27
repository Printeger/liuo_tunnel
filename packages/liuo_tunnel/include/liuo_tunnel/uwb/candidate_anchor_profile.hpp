#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>
#include <array>

namespace liuo_tunnel {
struct CandidateAnchorProfile {
  double gradient_norm = 0;
  std::array<double, 2> minimum_cost_change{0, 0};
  std::array<size_t, 2> feasible_trials{0, 0};
};
// Diagnostic only: hold every joint variable fixed except the new anchor.
// Factors not incident on that anchor cancel exactly; excluding them avoids
// subtracting the large VGICP/background cost. No values or factors are changed.
// A detected descent is evidence against stationarity. Absence is not proof:
// this finite probe omits coupled robot/anchor directions and other step sizes.
inline CandidateAnchorProfile candidate_anchor_profile(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
                                                       gtsam::Key anchor, const gtsam::Rot3& basis) {
  CandidateAnchorProfile result;
  std::vector<gtsam::NonlinearFactor::shared_ptr> incident;
  std::vector<double> initial_errors;
  gtsam::Vector3 gradient = gtsam::Vector3::Zero();
  for (const auto& factor : graph) {
    if (!factor || std::find(factor->keys().begin(), factor->keys().end(), anchor) == factor->keys().end()) continue;
    incident.push_back(factor);
    initial_errors.push_back(factor->error(values));
    if (!std::isfinite(initial_errors.back())) throw std::domain_error("NONFINITE_ANCHOR_PROFILE_BASE");
    gradient += factor->linearize(values)->gradientAtZero().at(anchor);
  }
  if (incident.empty() || !gradient.allFinite()) throw std::domain_error("INVALID_ANCHOR_PROFILE");
  result.gradient_norm = gradient.norm();
  std::vector<gtsam::Vector3> directions;
  for (int axis = 0; axis < 3; ++axis)
    for (double sign : {-1., 1.}) directions.push_back(sign * basis.matrix().col(axis));
  if (result.gradient_norm > 0) directions.push_back(-gradient / result.gradient_norm);
  const std::array<double, 2> steps{1e-3, 1e-4};
  for (size_t level = 0; level < steps.size(); ++level) {
    for (const auto& direction : directions) {
      auto trial = values;
      trial.update(anchor, gtsam::Point3(values.at<gtsam::Point3>(anchor) + steps[level] * direction));
      long double change = 0;
      bool finite = true;
      for (size_t i = 0; i < incident.size(); ++i) {
        const double error = incident[i]->error(trial);
        if (!std::isfinite(error)) { finite = false; break; }
        change += static_cast<long double>(error) - initial_errors[i];
      }
      if (!finite) continue;
      ++result.feasible_trials[level];
      result.minimum_cost_change[level] = std::min(result.minimum_cost_change[level], static_cast<double>(change));
    }
  }
  return result;
}
}  // namespace liuo_tunnel
