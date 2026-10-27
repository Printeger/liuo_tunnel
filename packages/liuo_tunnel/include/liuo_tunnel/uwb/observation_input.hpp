#pragma once
#include <liuo_uwb_msgs/msg/uwb_observation_batch.hpp>
#include <liuo_tunnel/uwb/measurement_queue.hpp>
#include <nlohmann/json.hpp>
#include <limits>

namespace liuo_tunnel {
// One configured network per subscription. No geometry is inferred from IDs.
class ObservationInput {
public:
  using Batch = liuo_uwb_msgs::msg::UwbObservationBatch;
  using Observation = liuo_uwb_msgs::msg::UwbObservation;
  explicit ObservationInput(const nlohmann::json& config) : config_(config) {
    network_ = config.at("network_instance").get<std::string>();
    if (network_.empty()) throw std::invalid_argument("EMPTY_UWB_NETWORK");
    std::set<std::pair<uint8_t, uint32_t>> targets;
    for (const auto& node : config.at("nodes")) {
      const int logical = node.at("logical_id");
      const int type = node.at("type");
      const uint32_t internal = node.at("internal_id");
      if (logical < 0 || logical > 255 || type < 0 || type > 1 || !targets.insert({type, internal}).second ||
          !nodes_.emplace(logical, std::make_pair(type, internal)).second) throw std::invalid_argument("INVALID_UWB_REGISTRY");
    }
    receiver_ = config.at("receiver_id").get<int>();
    if (!nodes_.count(receiver_) || nodes_.at(receiver_).first != 0) throw std::invalid_argument("INVALID_UWB_RECEIVER");
    if (config.contains("model")) {
      const auto& model = config.at("model");
      for (const char* key : {"id", "source", "clock_domain", "time_evidence", "noise_evidence", "independence_evidence"})
        if (model.at(key).get<std::string>().empty()) throw std::invalid_argument("MISSING_UWB_MODEL_EVIDENCE");
      ta_sigma_ = model.at("ta_sigma_m"); aa_sigma_ = model.at("aa_sigma_m"); max_uncertainty_ = model.at("max_time_uncertainty_s");
      if (!std::isfinite(ta_sigma_) || ta_sigma_ <= 0 || !std::isfinite(aa_sigma_) || aa_sigma_ <= 0 ||
          !std::isfinite(max_uncertainty_) || max_uncertainty_ < 0) throw std::invalid_argument("INVALID_UWB_MODEL");
      modeled_ = true;
    }
  }
  std::string batch_reason(const Batch& b) const {
    if (!b.header.frame_id.empty() || b.receiver_id != receiver_ || b.trigger_type > 1 || b.driver_session_id == 0) return "INVALID_BATCH";
    if (b.observations.size() > 1024) return "BATCH_LIMIT";
    return "";
  }
  size_t identity_size() const {
    size_t size = 0;
    for (const auto& source : sources_)
      for (const auto& frame : source.second.frames) size += frame.second.size();
    return size;
  }
  size_t retired_size() const {
    size_t size = retired_drivers_.size();
    for (const auto& source : sources_) size += source.second.retired.size();
    return size;
  }
  // Clock resets retire the entire dataset, including records not yet fused.
  void reset() {
    if (driver_) retired_drivers_.insert(*driver_);
    driver_.reset(); sources_.clear(); aa_sources_.clear();
    if (retired_drivers_.size() >= 64) exhausted_ = true;
  }
  std::string adapt(const Batch& b, const Observation& m, double callback_stamp, RangeObservation& out) {
    out.producer = m.reporter_id; out.session = m.source_session_id; out.sequence = m.source_frame_seq;
    out.received = callback_stamp; out.range = m.distance_m;
    out.driver_session = b.driver_session_id; out.network = network_;
    out.stamp = double(m.source_frame_stamp.sec) + 1e-9 * m.source_frame_stamp.nanosec;
    auto reason = batch_reason(b); if (!reason.empty()) return reason;
    if (m.kind > 1 || m.time_basis > 3 || m.shard_count == 0 || m.shard_index >= m.shard_count) return "INVALID_ENUM_OR_SHARD";
    if (!nodes_.count(m.node_a) || !nodes_.count(m.node_b) || m.node_a == m.node_b || m.reporter_id != m.node_a) return "INVALID_ENDPOINTS";
    const auto a = nodes_.at(m.node_a), c = nodes_.at(m.node_b);
    if (a.first != m.kind || c.first != 1 || (m.kind == 0 && m.node_a != receiver_) || b.trigger_type != m.kind) return "INVALID_REPORTER_KIND";
    out.a_type = a.first; out.a = a.second; out.b_type = c.first; out.b = c.second;
    if (!std::isfinite(m.distance_m) || m.distance_m <= 0) return "INVALID_DISTANCE";
    if (m.rssi_valid && (!std::isfinite(m.fp_rssi) || !std::isfinite(m.rx_rssi))) return "INVALID_RSSI";
    if (m.source_hold_age_valid && m.source_hold_age_ms == 65535) return "INVALID_HOLD_AGE";
    if (!m.source_frame_seq_valid) return "SOURCE_SEQUENCE_UNKNOWN";
    if (m.source_session_id == 0) return "INVALID_SOURCE_SESSION";
    reason = identity(b, m); if (!reason.empty()) return reason;
    if (m.time_basis == 0 || m.time_basis == 3) return "MEASUREMENT_TIME_UNKNOWN";
    if (m.source_frame_stamp.sec < 0 || m.source_frame_stamp.nanosec >= 1000000000 || out.stamp <= 0) return "INVALID_SOURCE_TIME";
    if (!modeled_) return "MODEL_UNAVAILABLE";
    if (!std::isfinite(m.time_uncertainty_s) || m.time_uncertainty_s < 0 || m.time_uncertainty_s > max_uncertainty_) return "TIME_UNCERTAINTY_UNSUPPORTED";
    if (!m.pair_measurement_freshness_verified) return "INDEPENDENCE_UNVERIFIED";
    // Lock to the first available directed source for each static edge. Never
    // privilege the smaller ID; changing source would require correlation evidence.
    if (m.kind == 1) {
      const auto edge = std::minmax(m.node_a, m.node_b);
      auto [it, inserted] = aa_sources_.emplace(std::make_pair(edge.first, edge.second), m.reporter_id);
      if (!inserted && it->second != m.reporter_id) return "AA_OTHER_SOURCE";
    }
    out.variance = std::pow(m.kind ? aa_sigma_ : ta_sigma_, 2);
    out.identity_validated = true;
    return "ACCEPT_INPUT";
  }
private:
  struct Source {
    uint64_t session = 0, maximum = 0;
    bool initialized = false;
    std::set<uint64_t> retired;
    std::map<uint64_t, std::set<std::tuple<uint8_t,uint8_t,uint8_t>>> frames;
  };
  std::string identity(const Batch& b, const Observation& m) {
    if (exhausted_) return "IDENTITY_CAPACITY_EXHAUSTED";
    if (retired_drivers_.count(b.driver_session_id)) return "RETIRED_DRIVER_SESSION";
    if (driver_ && *driver_ != b.driver_session_id) reset();
    if (exhausted_) return "IDENTITY_CAPACITY_EXHAUSTED";
    driver_ = b.driver_session_id;
    auto& s = sources_[m.reporter_id];
    if (s.retired.count(m.source_session_id)) return "RETIRED_SOURCE_SESSION";
    if (s.session && s.session != m.source_session_id) {
      if (s.retired.size() >= 64) return "SOURCE_SESSION_CAPACITY";
      s.retired.insert(s.session); s.frames.clear(); s.initialized = false;
    }
    s.session = m.source_session_id;
    uint64_t seq = m.source_frame_seq;
    if (s.initialized) {
      const uint32_t delta = m.source_frame_seq - uint32_t(s.maximum);
      if (delta == 0x80000000u) return "AMBIGUOUS_SEQUENCE";
      if (delta < 0x80000000u) {
        if (delta > 1000000) return "SEQUENCE_ADVANCE_LIMIT";
        seq = s.maximum + delta;
      } else {
        const uint64_t back = uint64_t(0x100000000ull) - delta;
        if (back > 127 || back > s.maximum) return "IDENTITY_TOO_OLD";
        seq = s.maximum - back;
      }
    }
    s.initialized = true; s.maximum = std::max(s.maximum, seq);
    while (!s.frames.empty() && s.maximum - s.frames.begin()->first > 127) s.frames.erase(s.frames.begin());
    if (!s.frames[seq].insert({m.kind,m.node_a,m.node_b}).second) return "DUPLICATE";
    return "";
  }
  nlohmann::json config_;
  std::string network_;
  int receiver_ = 0;
  std::map<int, std::pair<uint8_t,uint32_t>> nodes_;
  bool modeled_ = false, exhausted_ = false;
  double ta_sigma_ = NAN, aa_sigma_ = NAN, max_uncertainty_ = NAN;
  std::optional<uint64_t> driver_;
  std::set<uint64_t> retired_drivers_;
  std::map<uint8_t, Source> sources_;
  std::map<std::pair<uint8_t,uint8_t>,uint8_t> aa_sources_;
};
} // namespace liuo_tunnel
