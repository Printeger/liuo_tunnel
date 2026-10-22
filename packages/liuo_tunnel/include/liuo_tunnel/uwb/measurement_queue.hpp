#pragma once
#include <gtsam/inference/Key.h>
#include <map>
#include <vector>
#include <set>
#include <deque>
#include <tuple>
#include <cmath>
#include <optional>
#include <cstdint>
#include <string>
namespace liuo_tunnel {
struct RangeObservation {
  uint32_t producer = 0;
  uint64_t session = 0, sequence = 0;
  double stamp = 0, received = 0, range = 0, variance = 0;
  uint8_t a_type = 0, b_type = 1;
  uint32_t a = 0, b = 0;
};
struct RangeBracket {
  std::string reason;
  gtsam::Key left = 0;
  std::optional<gtsam::Key> right;
  double beta = 0;
};
inline RangeBracket range_bracket(const std::map<double, gtsam::Key>& states, double stamp) {
  if (states.empty()) return {"WAIT_RIGHT"};
  auto right = states.lower_bound(stamp - 1e-8);
  if (right == states.end()) return {"WAIT_RIGHT"};
  if (std::abs(right->first - stamp) < 1e-8) return {"SYNC", right->second, std::nullopt, 0};
  if (right == states.begin()) return {"WINDOW_EXPIRED"};
  auto left = std::prev(right);
  double dt = right->first - left->first;
  if (dt > .150000001) return {"TIME_BRACKET_GAP"};
  return {"INTERPOLATED", left->second, right->second, (stamp - left->first) / dt};
}
// The user-declared stationary alignment interval can share its single endpoint
// pose. Never extrapolate a dynamic pose or extend the interval to a late cloud.
inline RangeBracket static_alignment_bracket(const std::map<double, gtsam::Key>& states, double stamp, double start, double end) {
  if (
    states.size() == 1 && std::isfinite(start) && std::isfinite(end) && start >= 0 && end > start && std::abs(states.begin()->first - end) < 1e-8 && stamp >= start && stamp < end)
    return {"STATIC_ALIGNMENT", states.begin()->second, std::nullopt, 0};
  return range_bracket(states, stamp);
}
// Bounded sequence watermark prevents old duplicates being resurrected after
// their factor leaves the smoother. Sequence identity is independent of time order.
class RangeQueue {
public:
  explicit RangeQueue(size_t capacity = 4096) : capacity_(capacity) {}
  std::string push(const RangeObservation& o) {
    if (!std::isfinite(o.stamp) || o.stamp <= 0 || !std::isfinite(o.range) || o.range <= 0 || !std::isfinite(o.variance) || o.variance <= 0) return "INVALID_FIELDS";
    if (o.a_type > 1 || o.b_type > 1 || (o.a_type == 0 && o.b_type == 0)) return "UNSUPPORTED_LINK";
    if (o.a_type == o.b_type && o.a == o.b) return "INVALID_ENDPOINTS";
    auto identity = std::make_pair(o.producer, o.session);
    if (retired_.count(identity)) return "RETIRED_SESSION";
    auto active = active_.find(o.producer);
    if (active != active_.end() && active->second != o.session) {
      retire({o.producer, active->second});
      // Already queued old-session data remains a valid physical observation on
      // this time axis; clock reset explicitly drains/retires it in the owner.
    }
    active_[o.producer] = o.session;
    auto& history = history_[identity];
    if (history.maximum > 65536 && o.sequence < history.maximum - 65536) return "IDENTITY_TOO_OLD";
    if (!history.sequences.insert(o.sequence).second) return "DUPLICATE";
    history.maximum = std::max(history.maximum, o.sequence);
    while (!history.sequences.empty() && history.maximum > 65536 && *history.sequences.begin() < history.maximum - 65536) history.sequences.erase(history.sequences.begin());
    if (pending_.size() >= capacity_) return "QUEUE_LIMIT";
    pending_.emplace(o.stamp, o);
    return "QUEUED";
  }
  std::vector<RangeObservation> take_until(double t) {
    std::vector<RangeObservation> result;
    auto stop = pending_.upper_bound(t + 1e-8);
    for (auto it = pending_.begin(); it != stop; ++it) result.push_back(it->second);
    pending_.erase(pending_.begin(), stop);
    return result;
  }
  std::vector<RangeObservation> reset() {
    auto pending = take_until(INFINITY);
    for (const auto& item : active_) retire(item);
    active_.clear();
    history_.clear();
    return pending;
  }
  size_t size() const { return pending_.size(); }

private:
  struct History {
    uint64_t maximum = 0;
    std::set<uint64_t> sequences;
  };
  void retire(std::pair<uint32_t, uint64_t> id) {
    if (retired_.insert(id).second) retirement_.push_back(id);
    history_.erase(id);
    while (retirement_.size() > 64) {
      retired_.erase(retirement_.front());
      retirement_.pop_front();
    }
  }
  size_t capacity_;
  std::multimap<double, RangeObservation> pending_;
  std::map<uint32_t, uint64_t> active_;
  std::map<std::pair<uint32_t, uint64_t>, History> history_;
  std::set<std::pair<uint32_t, uint64_t>> retired_;
  std::deque<std::pair<uint32_t, uint64_t>> retirement_;
};
}  // namespace liuo_tunnel
