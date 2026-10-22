#pragma once
#include <liuo_tunnel/uwb/initialization_candidates.hpp>
#include <liuo_tunnel/uwb/range_factors.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <Eigen/Cholesky>

namespace liuo_tunnel {
// This four-DOF solve changes numerical starting values only. The full raw
// graph remains the estimator; no fitted position/trajectory becomes a factor.
inline gtsam::Values move_bootstrap_gauge(const gtsam::Values& values, const Eigen::Vector4d& step) {
  auto result = values;
  const gtsam::Rot3 rotation = gtsam::Rot3::Rz(step[3]);
  for (auto key : values.keys()) {
    if (gtsam::Symbol(key).chr() == 'x') {
      const auto& pose = values.at<gtsam::Pose3>(key);
      result.update(key, gtsam::Pose3(rotation * pose.rotation(), rotation * pose.translation() + step.head<3>()));
    }
    if (gtsam::Symbol(key).chr() == 'v') result.update(key, gtsam::Vector3(rotation * values.at<gtsam::Vector3>(key)));
  }
  return result;
}
inline Eigen::RowVector4d range_gauge_jacobian(const TagAnchorRangeFactor& factor, const gtsam::Values& values) {
  std::vector<gtsam::Matrix> H;
  factor.unwhitenedError(values, &H);
  Eigen::RowVector4d J = Eigen::RowVector4d::Zero();
  for (size_t i = 0; i < factor.keys().size(); ++i) {
    const auto key = factor.keys()[i];
    if (gtsam::Symbol(key).chr() != 'x') continue;
    const auto& pose = values.at<gtsam::Pose3>(key);
    const Eigen::RowVector3d world = H[i].rightCols(3) * pose.rotation().matrix().transpose();
    J.head<3>() += world;
    J[3] += (H[i].leftCols(3) * pose.rotation().unrotate(gtsam::Vector3::UnitZ()))[0] + world.dot(gtsam::Vector3::UnitZ().cross(pose.translation()));
  }
  return J;
}
struct BootstrapPreconditionResult {
  gtsam::Values values;
  double cost = 0;
  size_t iterations = 0;
};
inline BootstrapPreconditionResult precondition_bootstrap(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial) {
  std::vector<const TagAnchorRangeFactor*> ranges;
  for (const auto& factor : graph)
    if (const auto* range = dynamic_cast<const TagAnchorRangeFactor*>(factor.get())) ranges.push_back(range);
  if (ranges.empty()) throw std::invalid_argument("NO_RANGE_PRECONDITION_DATA");
  auto cost = [&](const gtsam::Values& values) {
    double result = 0;
    for (const auto* range : ranges) result += range->error(values);
    return result;
  };
  BootstrapPreconditionResult result{initial, cost(initial), 0};
  double lambda = 1e-3;
  for (; result.iterations < 30; ++result.iterations) {
    Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
    Eigen::Vector4d g = Eigen::Vector4d::Zero();
    for (const auto* range : ranges) {
      const gtsam::Matrix J = range->noiseModel()->Whiten(gtsam::Matrix(range_gauge_jacobian(*range, result.values)));
      const auto e = range->noiseModel()->whiten(range->unwhitenedError(result.values));
      H += J.transpose() * J;
      g += J.transpose() * e;
    }
    if (!H.allFinite() || !g.allFinite() || !std::isfinite(result.cost)) throw std::runtime_error("INVALID_RANGE_PRECONDITION");
    if (g.lpNorm<Eigen::Infinity>() < 1e-6) break;
    Eigen::Matrix4d damped = H;
    damped.diagonal().array() += lambda * H.diagonal().array().max(1.);
    const Eigen::Vector4d step = -damped.ldlt().solve(g);
    if (!step.allFinite()) throw std::runtime_error("SINGULAR_RANGE_PRECONDITION");
    const auto trial = move_bootstrap_gauge(result.values, step);
    const double next = cost(trial);
    if (std::isfinite(next) && next < result.cost) {
      result.values = trial;
      result.cost = next;
      lambda = std::max(1e-12, lambda / 3);
    } else {
      lambda *= 10;
    }
  }
  return result;
}
}  // namespace liuo_tunnel
