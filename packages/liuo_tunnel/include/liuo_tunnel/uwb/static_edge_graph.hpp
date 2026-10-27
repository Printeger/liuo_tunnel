#pragma once
#include <liuo_tunnel/uwb/static_anchor_edges.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <memory>

namespace liuo_tunnel {
// Estimator-thread bridge from validated leaves to one explicit factor per edge.
// Vehicle time is deliberately not an input. A keys must already exist legally.
class StaticEdgeGraph {
  using Pair = std::pair<uint32_t, uint32_t>;

public:
  StaticEdgeGraph(std::string network, std::string deployment, size_t capacity, nlohmann::json absorbed = nullptr)
  : network_(std::move(network)),
    deployment_(std::move(deployment)),
    capacity_(capacity),
    absorbed_(std::move(absorbed)) {
    reset_graph();
  }

  void offer(const std::vector<StaticRangeBlock>& blocks) {
    if (transaction_) throw std::logic_error("STATIC_GRAPH_OFFER_DURING_UPDATE");
    std::vector<StaticRangeBlock> combined;
    for (const auto& [id, block] : pending_) combined.push_back(block);
    combined.insert(combined.end(), blocks.begin(), blocks.end());
    ledger_->prepare(combined);  // Validate conflicts and total capacity atomically.
    for (const auto& block : blocks) {
      if (ledger_->contains(block.physical_id)) continue;
      pending_.emplace(block.physical_id, block);
      history_.emplace(block.physical_id, block);
    }
  }

  // Read-only likelihood snapshot for one unknown candidate joined to known A.
  // Reading never absorbs a leaf or changes explicit factor ownership.
  std::map<Pair, GaussianRangeSummary> candidate_summaries(uint32_t unknown, const gtsam::Values& known) const {
    if (transaction_) throw std::logic_error("STATIC_CANDIDATE_DURING_UPDATE");
    if (known.exists(gtsam::symbol_shorthand::A(unknown))) throw std::invalid_argument("STATIC_CANDIDATE_ALREADY_KNOWN");
    std::map<Pair, GaussianRangeSummary> summaries;
    for (const auto& [id, block] : pending_) {
      if (block.first != unknown && block.second != unknown) continue;
      const auto other = block.first == unknown ? block.second : block.first;
      if (!known.exists(gtsam::symbol_shorthand::A(other))) continue;
      summaries[{std::min(block.first, block.second), std::max(block.first, block.second)}].add(block.range, block.variance);
    }
    return summaries;
  }

  gtsam::FactorIndices stage(gtsam::NonlinearFactorGraph& graph, const gtsam::Values& known) {
    if (transaction_) throw std::logic_error("STATIC_GRAPH_UPDATE_ALREADY_STAGED");
    std::vector<StaticRangeBlock> ready;
    for (const auto& [id, block] : pending_)
      if (known.exists(gtsam::symbol_shorthand::A(block.first)) && known.exists(gtsam::symbol_shorthand::A(block.second))) ready.push_back(block);
    transaction_ = std::make_unique<StaticAnchorEdges::Transaction>(ledger_->prepare(ready));
    ready_.clear();
    for (const auto& block : ready) ready_.push_back(block.physical_id);
    gtsam::FactorIndices remove;
    added_.clear();
    for (const auto& [pair, summary] : transaction_->updates()) {
      if (indices_.count(pair)) remove.push_back(indices_.at(pair));
      added_.push_back({pair, graph.size()});
      graph
        .emplace_shared<AnchorAnchorRangeFactor>(gtsam::symbol_shorthand::A(pair.first), gtsam::symbol_shorthand::A(pair.second), summary.mean, summary.sigma(), summary.scatter);
    }
    return remove;
  }

  void commit(const gtsam::FactorIndices& new_indices) {
    if (!transaction_) throw std::logic_error("STATIC_GRAPH_NO_STAGED_UPDATE");
    auto next_indices = indices_;
    for (const auto& [pair, local] : added_) next_indices[pair] = new_indices.at(local);
    ledger_->commit(std::move(*transaction_));
    indices_.swap(next_indices);
    for (const auto& id : ready_) pending_.erase(id);
    abort();
  }
  // Discard staging after failure. This does NOT repair an ISAM2 instance whose
  // update threw. Its owner must reconstruct the graph from a known checkpoint.
  void abort() {
    transaction_.reset();
    added_.clear();
    ready_.clear();
  }

  // The vehicle graph was discarded: retain physical records, but only the
  // original loaded map's leaves remain absorbed. Re-add other leaves once A
  // is initialized again; never claim they are already in the replacement graph.
  void reset_graph() {
    abort();
    ledger_ = absorbed_.is_null() ? std::make_unique<StaticAnchorEdges>(network_, deployment_, capacity_)
                                  : std::make_unique<StaticAnchorEdges>(network_, deployment_, capacity_, absorbed_);
    indices_.clear();
    pending_ = history_;
  }
  size_t pending() const { return pending_.size(); }
  size_t factor_count() const { return indices_.size(); }
  size_t fused() const { return ledger_->size() - (absorbed_.is_null() ? 0 : absorbed_.at("leaves").size()); }
  nlohmann::json checkpoint() const {
    if (transaction_) throw std::logic_error("STATIC_GRAPH_CHECKPOINT_DURING_UPDATE");
    return ledger_->absorbed_checkpoint();
  }

private:
  std::string network_, deployment_;
  size_t capacity_;
  nlohmann::json absorbed_;
  std::unique_ptr<StaticAnchorEdges> ledger_;
  std::map<std::string, StaticRangeBlock> pending_, history_;
  std::map<Pair, gtsam::FactorIndex> indices_;
  std::unique_ptr<StaticAnchorEdges::Transaction> transaction_;
  std::vector<std::pair<Pair, size_t>> added_;
  std::vector<std::string> ready_;
};
}  // namespace liuo_tunnel
