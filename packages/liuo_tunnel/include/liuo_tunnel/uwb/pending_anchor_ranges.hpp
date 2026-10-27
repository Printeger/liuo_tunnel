#pragma once
#include <liuo_tunnel/uwb/measurement_queue.hpp>
#include <liuo_tunnel/uwb/range_factors.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace liuo_tunnel {
struct PendingAnchorRange {
  RangeObservation observation;
  RangeBracket bracket;
};
struct KnownAnchorRangeTransfer {
  gtsam::NonlinearFactorGraph factors;
  std::vector<PendingAnchorRange> accepted, expired, remaining;
};
// Prepare without consuming input. The graph owner swaps remaining only after
// the graph update succeeds, so a failed update never silently consumes raw TA.
inline KnownAnchorRangeTransfer prepare_known_anchor_ranges(const std::vector<PendingAnchorRange>& pending, const gtsam::Values& ready, double cutoff, const gtsam::Point3& lever) {
  KnownAnchorRangeTransfer result;
  for (const auto& item : pending) {
    const auto& o = item.observation;
    const auto& b = item.bracket;
    if (o.a_type != 0 || !ready.exists(gtsam::symbol_shorthand::A(o.b))) {
      result.remaining.push_back(item);
      continue;
    }
    if (o.stamp < cutoff || !ready.exists(b.left) || (b.right && !ready.exists(*b.right))) {
      result.expired.push_back(item);
      continue;
    }
    result.factors.emplace_shared<TagAnchorRangeFactor>(b.left, b.right, gtsam::symbol_shorthand::A(o.b), gtsam::Point3::Zero(), lever, b.beta, o.range, std::sqrt(o.variance));
    result.accepted.push_back(item);
  }
  return result;
}
}  // namespace liuo_tunnel
