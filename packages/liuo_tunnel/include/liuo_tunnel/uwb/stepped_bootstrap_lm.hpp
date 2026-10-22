#pragma once
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <cmath>
namespace liuo_tunnel {
// Same convergence policy as gtsam_points 1.2.1 optimize(), exposed one outer
// iteration at a time. Derived from the MIT/BSD implementation credited in
// THIRD_PARTY_NOTICES.md. Graph, damping and GPU caches survive between steps.
class SteppedBootstrapLM : public gtsam_points::LevenbergMarquardtOptimizerExt {
public:
  SteppedBootstrapLM(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial, const gtsam_points::LevenbergMarquardtExtParams& params)
  : LevenbergMarquardtOptimizerExt(graph, initial, params) {
    optimization_start_time_ = std::chrono::high_resolution_clock::now();
    initTime();
  }
  bool advance() {
    if (complete_) return true;
    const double before = error();
    linearized_ = iterate();
    const double after = error(), change = std::abs(before - after);
    complete_ = iterations() > params_.maxIterations || !std::isfinite(after) || change < params_.absoluteErrorTol || change / std::abs(before) < params_.relativeErrorTol ||
                after < params_.errorTol;
    if (params_.termination_criteria) complete_ = params_.termination_criteria(values()) || complete_;
    return complete_;
  }

private:
  bool complete_ = false;
};
}  // namespace liuo_tunnel
