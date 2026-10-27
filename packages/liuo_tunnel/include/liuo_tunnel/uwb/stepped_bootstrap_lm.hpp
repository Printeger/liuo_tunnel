#pragma once
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <cmath>
#include <fstream>
#include <gtsam/nonlinear/internal/NonlinearOptimizerState.h>
#include <gtsam_points/optimizers/linearization_hook.hpp>
namespace liuo_tunnel {
// Same convergence policy as gtsam_points 1.2.1 optimize(), exposed one outer
// iteration at a time. Derived from the MIT/BSD implementation credited in
// THIRD_PARTY_NOTICES.md. Graph, damping and GPU caches survive between steps.
class SteppedBootstrapLM : public gtsam_points::LevenbergMarquardtOptimizerExt {
public:
  SteppedBootstrapLM(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& initial, const gtsam_points::LevenbergMarquardtExtParams& params, bool single_trial = false)
  : LevenbergMarquardtOptimizerExt(graph, initial, params), single_trial_(single_trial) {
    optimization_start_time_ = std::chrono::high_resolution_clock::now();
    initTime();
  }
  bool finished() const { return complete_; }
  // Diagnostic bits can coexist: reaching the iteration budget does not
  // establish stationarity, even when another stopping condition also fires.
  unsigned termination_flags() const { return termination_flags_; }
  double last_absolute_error_change() const { return last_change_; }
  bool advance() {
    if (complete_) return true;
    double before = error();
    if (single_trial_) {
      if (!trial_linear_) {
        outer_before_ = before;
        const auto start = std::chrono::high_resolution_clock::now();
        linearization_hook_->linearize(values());
        trial_linear_ = graph_.linearize(values());
        linearization_time_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - start);
        linearization_hook_->error(values());
        const_cast<double&>(state_->error) = graph_.error(values());
        if (getInnerIterations() == 0) write_trial_log(error());
        diagonal_ = gtsam::VectorValues();
        if (params_.diagonalDamping) {
          diagonal_ = trial_linear_->hessianDiagonal();
          for (auto& entry : diagonal_) entry.second = entry.second.cwiseMax(params_.minDiagonal).cwiseMin(params_.maxDiagonal).cwiseSqrt();
        }
      }
      if (!tryLambda(*trial_linear_, diagonal_)) {
        write_trial_log(error());
        return false;
      }
      before = outer_before_;
      linearized_ = trial_linear_;
      trial_linear_.reset();
    } else linearized_ = iterate();
    const double after = error(), change = std::abs(before - after);
    last_change_ = change;
    termination_flags_ = (iterations() > params_.maxIterations ? 1u : 0u) | (!std::isfinite(after) ? 2u : 0u) |
                         (change < params_.absoluteErrorTol ? 4u : 0u) | (change / std::abs(before) < params_.relativeErrorTol ? 8u : 0u) |
                         (after < params_.errorTol ? 16u : 0u);
    if (params_.termination_criteria && params_.termination_criteria(values())) termination_flags_ |= 32u;
    complete_ = termination_flags_ != 0;
    return complete_;
  }

private:
  // Upstream writeLogFile is inline in its .cpp and is not exported by the
  // installed shared library. Preserve its optional CSV behavior locally.
  void write_trial_log(double current_error) const {
    if (params_.logFile.empty()) return;
    std::ofstream out(params_.logFile, std::ios::app);
    const double seconds = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startTime_).count() / 1e6;
    out << getInnerIterations() << ',' << seconds << ',' << current_error << ',' << lambda() << ',' << iterations() << std::endl;
  }
  bool complete_ = false, single_trial_ = false;
  unsigned termination_flags_ = 0;
  double last_change_ = NAN;
  double outer_before_ = 0;
  gtsam::GaussianFactorGraph::shared_ptr trial_linear_;
  gtsam::VectorValues diagonal_;
};
}  // namespace liuo_tunnel
