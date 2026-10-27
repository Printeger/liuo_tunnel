#pragma once
#include <liuo_tunnel_msgs/msg/static_anchor_record.hpp>
#include <liuo_tunnel/uwb/static_anchor_edges.hpp>
#include <set>

namespace liuo_tunnel {
// Stateless callback validation. This does not claim fusion, retain a revision
// watermark, initialize A, or change a physical timestamp to the vehicle time.
class StaticRecordInput {
public:
  explicit StaticRecordInput(const nlohmann::json& config) : config_(config) {
    for (const auto key : {"network", "deployment_epoch", "source_identity", "noise_evidence", "independence_evidence", "time_basis"})
      if (!identity(config.at(key).get<std::string>())) throw std::invalid_argument("STATIC_RECORD_SOURCE_EVIDENCE_MISSING");
    variance_ = config.at("variance_m2");
    if (!std::isfinite(variance_) || variance_ <= 0 || config.at("time_basis") != "shared_simulation_clock") throw std::invalid_argument("STATIC_RECORD_MODEL_UNSUPPORTED");
  }

  std::vector<StaticRangeBlock> validate(const liuo_tunnel_msgs::msg::StaticAnchorRecord& record) const {
    if (record.network != config_.at("network") || record.deployment_epoch != config_.at("deployment_epoch") || record.source_identity != config_.at("source_identity"))
      throw std::invalid_argument("STATIC_RECORD_SOURCE_MISMATCH");
    if (
      !record.valid || !identity(record.record_id) || record.revision == 0 || record.first == record.second || !record.header.frame_id.empty() ||
      record.error_model != "independent-gaussian-v1" || !valid_time(record.header.stamp))
      throw std::invalid_argument("STATIC_RECORD_INVALID");
    if (record.leaves.empty() || record.leaves.size() > 128 || record.covered_blocks.size() != record.leaves.size()) throw std::invalid_argument("STATIC_RECORD_COVERAGE");
    std::set<std::string> covered(record.covered_blocks.begin(), record.covered_blocks.end());
    if (covered.size() != record.leaves.size()) throw std::invalid_argument("STATIC_RECORD_COVERAGE");
    std::vector<StaticRangeBlock> output;
    output.reserve(record.leaves.size());
    for (const auto& leaf : record.leaves) {
      if (!identity(leaf.physical_id) || !covered.erase(leaf.physical_id) || (leaf.physical_source != record.first && leaf.physical_source != record.second))
        throw std::invalid_argument("STATIC_RECORD_LEAF_IDENTITY");
      if (
        !leaf.acquisition_stamp_valid || !valid_time(leaf.acquisition_stamp) || leaf.time_basis != config_.at("time_basis") ||
        leaf.acquisition_stamp.sec > record.header.stamp.sec ||
        (leaf.acquisition_stamp.sec == record.header.stamp.sec && leaf.acquisition_stamp.nanosec > record.header.stamp.nanosec))
        throw std::invalid_argument("STATIC_RECORD_SOURCE_TIME");
      if (!std::isfinite(leaf.range_m) || leaf.range_m <= 0 || !std::isfinite(leaf.variance_m2) || leaf.variance_m2 != variance_)
        throw std::invalid_argument("STATIC_RECORD_RANGE_MODEL");
      output.push_back({record.network, record.deployment_epoch, record.first, record.second, leaf.physical_id, record.error_model, leaf.range_m, leaf.variance_m2});
    }
    return output;
  }

private:
  static bool identity(const std::string& value) { return !value.empty() && value.size() <= 256 && value.find('\0') == std::string::npos; }
  static bool valid_time(const builtin_interfaces::msg::Time& time) { return time.sec >= 0 && time.nanosec < 1000000000; }
  nlohmann::json config_;
  double variance_;
};
}  // namespace liuo_tunnel
