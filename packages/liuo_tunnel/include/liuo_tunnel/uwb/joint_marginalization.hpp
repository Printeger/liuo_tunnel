#pragma once
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam_points/optimizers/linearization_hook.hpp>
#include <set>

namespace liuo_tunnel {
struct JointMarginalization {
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  size_t retired_factors = 0;
};
// A branch-local, fixed-linearization approximation, not a new measurement.
// Replace the input graph with the result; adding the result to it double counts.
// Untouched nonlinear factors retain their original likelihood and ownership.
inline JointMarginalization marginalize_joint(
    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values, const gtsam::Ordering& eliminate) {
  JointMarginalization result;
  const std::set<gtsam::Key> removed(eliminate.begin(), eliminate.end());
  if (removed.size() != eliminate.size()) throw std::invalid_argument("MARGINAL_DUPLICATE_KEY");
  const auto graph_keys = graph.keys();
  for (auto key : eliminate)
    if (!values.exists(key) || !graph_keys.count(key)) throw std::invalid_argument("MARGINAL_MISSING_KEY");
  gtsam::NonlinearFactorGraph absorbed;
  for (const auto& factor : graph) {
    if (!factor) continue;
    bool touches_old = false;
    for (auto key : factor->keys()) touches_old = touches_old || removed.count(key);
    if (touches_old) absorbed.push_back(factor->clone());
    else result.graph.push_back(factor);
  }
  result.retired_factors = absorbed.size();
  for (auto key : values.keys())
    if (!removed.count(key)) result.values.insert(key, values.at(key));
  if (!absorbed.empty()) {
    // GPU factors require the same batched linearization hook as the optimizer.
    gtsam_points::LinearizationHook hook(absorbed);
    hook.linearize(values);
    const auto linear = absorbed.linearize(values);
    const auto remaining = linear->eliminatePartialSequential(gtsam::KeyVector(eliminate.begin(), eliminate.end()), gtsam::EliminateCholesky).second;
    result.graph.add(gtsam::LinearContainerFactor::ConvertLinearGraph(*remaining, result.values));
  }
  for (auto key : result.graph.keys())
    if (removed.count(key)) throw std::logic_error("MARGINAL_RETIRED_KEY_SURVIVED");
  return result;
}
}  // namespace liuo_tunnel
