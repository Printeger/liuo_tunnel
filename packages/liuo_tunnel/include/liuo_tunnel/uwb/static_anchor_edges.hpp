#pragma once
#include <liuo_tunnel/uwb/range_factors.hpp>
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace liuo_tunnel {

// A physical exchange, not a packet. The producer must establish the ID across
// delivery retries and driver restarts. Delivery clocks are deliberately absent.
struct StaticRangeBlock {
  std::string network, deployment;
  uint32_t first, second;
  std::string physical_id, error_model;
  double range, variance;
};

// Estimator-thread ledger. prepare() is side-effect free; commit() is called
// only after the graph update succeeds. No anchor initialization is done here.
class StaticAnchorEdges {
  using Pair = std::pair<uint32_t, uint32_t>;
  using Blocks = std::map<std::string, StaticRangeBlock>;
  using Summaries = std::map<Pair, GaussianRangeSummary>;

public:
  class Transaction {
  public:
    // Only changed edges: duplicates must not cause even a factor replacement.
    const Summaries& updates() const { return updates_; }

  private:
    friend class StaticAnchorEdges;
    const StaticAnchorEdges* owner_;
    uint64_t generation_;
    Blocks blocks_;
    Summaries summaries_;
    Summaries updates_;
    Transaction(const StaticAnchorEdges* owner, uint64_t generation, Blocks blocks, Summaries summaries)
    : owner_(owner),
      generation_(generation),
      blocks_(std::move(blocks)),
      summaries_(std::move(summaries)) {}
  };

  StaticAnchorEdges(std::string network, std::string deployment, size_t max_blocks) : network_(std::move(network)), deployment_(std::move(deployment)), max_blocks_(max_blocks) {
    if (!valid_identity(network_) || !valid_identity(deployment_) || max_blocks_ == 0) throw std::invalid_argument("STATIC_EDGE_INVALID_DOMAIN");
  }
  StaticAnchorEdges(const StaticAnchorEdges&) = delete;
  StaticAnchorEdges& operator=(const StaticAnchorEdges&) = delete;

  // Only for a ledger stored atomically with the posterior that absorbed it.
  // Old maps without this exact schema cannot claim an empty absorbed set.
  StaticAnchorEdges(std::string network, std::string deployment, size_t max_blocks, const nlohmann::json& absorbed)
  : StaticAnchorEdges(std::move(network), std::move(deployment), max_blocks) {
    if (absorbed.at("schema") != "liuo_absorbed_static_leaves_v1" || absorbed.at("network") != network_ || absorbed.at("deployment_epoch") != deployment_)
      throw std::invalid_argument("STATIC_EDGE_CHECKPOINT_DOMAIN");
    const auto& records = absorbed.at("leaves");
    if (!records.is_array() || records.size() > max_blocks_) throw std::invalid_argument("STATIC_EDGE_CHECKPOINT_CAPACITY");
    std::vector<StaticRangeBlock> leaves;
    for (const auto& record : records) {
      leaves.push_back(
        {network_,
         deployment_,
         endpoint(record.at("first")),
         endpoint(record.at("second")),
         record.at("physical_id").get<std::string>(),
         record.at("error_model").get<std::string>(),
         record.at("range_m").get<double>(),
         record.at("variance_m2").get<double>()});
    }
    auto restored = prepare(leaves);
    if (restored.blocks_.size() != leaves.size()) throw std::invalid_argument("STATIC_EDGE_CHECKPOINT_DUPLICATE");
    commit(std::move(restored));
    summaries_.clear();  // All restored leaves live only in the joint prior.
  }

  nlohmann::json absorbed_checkpoint() const {
    nlohmann::json leaves = nlohmann::json::array();
    for (const auto& [id, block] : blocks_)
      leaves.push_back(
        {{"physical_id", id}, {"first", block.first}, {"second", block.second}, {"error_model", block.error_model}, {"range_m", block.range}, {"variance_m2", block.variance}});
    return {{"schema", "liuo_absorbed_static_leaves_v1"}, {"network", network_}, {"deployment_epoch", deployment_}, {"leaves", leaves}};
  }

  Transaction prepare(const std::vector<StaticRangeBlock>& input) const {
    Transaction next(this, generation_, blocks_, summaries_);
    for (auto block : input) {
      if (block.network != network_ || block.deployment != deployment_) throw std::invalid_argument("STATIC_EDGE_DEPLOYMENT_MISMATCH");
      if (block.first == block.second || !valid_identity(block.physical_id) || block.error_model != "independent-gaussian-v1")
        throw std::invalid_argument("STATIC_EDGE_INVALID_IDENTITY_OR_MODEL");
      if (block.first > block.second) std::swap(block.first, block.second);
      GaussianRangeSummary sample;
      sample.add(block.range, block.variance);
      auto found = next.blocks_.find(block.physical_id);
      if (found != next.blocks_.end()) {
        const auto& old = found->second;
        if (old.first != block.first || old.second != block.second || old.range != block.range || old.variance != block.variance || old.error_model != block.error_model)
          throw std::invalid_argument("STATIC_EDGE_IDENTITY_CONFLICT");
        continue;
      }
      if (next.blocks_.size() >= max_blocks_) throw std::length_error("STATIC_EDGE_BLOCK_CAPACITY");
      next.summaries_[{block.first, block.second}].add(block.range, block.variance);
      next.updates_[{block.first, block.second}] = next.summaries_.at({block.first, block.second});
      next.blocks_.emplace(block.physical_id, std::move(block));
    }
    return next;
  }

  void commit(Transaction transaction) {
    if (transaction.owner_ != this || transaction.generation_ != generation_) throw std::logic_error("STATIC_EDGE_STALE_TRANSACTION");
    blocks_.swap(transaction.blocks_);
    summaries_.swap(transaction.summaries_);
    ++generation_;
  }

  GaussianRangeSummary summary(uint32_t first, uint32_t second) const {
    const auto pair = std::minmax(first, second);
    auto found = summaries_.find({pair.first, pair.second});
    return found == summaries_.end() ? GaussianRangeSummary{} : found->second;
  }

private:
  static uint32_t endpoint(const nlohmann::json& value) {
    if (!value.is_number_integer() || (!value.is_number_unsigned() && value.get<int64_t>() < 0) || value.get<uint64_t>() > UINT32_MAX)
      throw std::invalid_argument("STATIC_EDGE_CHECKPOINT_ENDPOINT");
    return value.get<uint32_t>();
  }
  static bool valid_identity(const std::string& value) { return !value.empty() && value.size() <= 256 && value.find('\0') == std::string::npos; }
  std::string network_, deployment_;
  size_t max_blocks_;
  uint64_t generation_ = 0;
  Blocks blocks_;
  Summaries summaries_;
};
}  // namespace liuo_tunnel
