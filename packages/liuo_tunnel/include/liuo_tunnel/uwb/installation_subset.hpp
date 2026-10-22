#pragma once
#include <liuo_tunnel/uwb/installation_input.hpp>
#include <liuo_tunnel/uwb/installation_factors.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
namespace liuo_tunnel {
// Replace the old subset factor with this whole correlated factor. Adding both
// would count the old installation observations twice. IDs are identities only.
inline gtsam::NonlinearFactor::shared_ptr joint_installation_factor(const InstallationInput& input, const std::vector<uint32_t>& ids) {
  gtsam::KeyVector keys;
  std::vector<size_t> rows;
  std::set<uint32_t> seen;
  for (auto id : ids) {
    auto found = std::find(input.ids.begin(), input.ids.end(), id);
    if (found == input.ids.end() || !seen.insert(id).second) throw std::invalid_argument("INSTALLATION_SUBSET_ID");
    rows.push_back(std::distance(input.ids.begin(), found));
    keys.push_back(gtsam::Symbol('a', id));
  }
  if (ids.empty()) throw std::invalid_argument("EMPTY_INSTALLATION_SUBSET");
  gtsam::Vector offsets(ids.size());
  gtsam::Matrix covariance(ids.size(), ids.size());
  for (size_t i = 0; i < ids.size(); ++i) {
    offsets[i] = input.offsets[rows[i]];
    for (size_t j = 0; j < ids.size(); ++j) covariance(i,j) = input.covariance(rows[i],rows[j]);
  }
  return gtsam::NonlinearFactor::shared_ptr(new InstallationOffsetFactor(gtsam::Symbol('l',0),keys,offsets,covariance,gtsam::Point3::Zero(),1000.));
}
inline size_t installation_subset_index(const gtsam::NonlinearFactorGraph& graph, const std::vector<uint32_t>& ids) {
  std::set<gtsam::Key> expected{gtsam::Symbol('l',0)};
  for (auto id : ids) expected.insert(gtsam::Symbol('a',id));
  size_t count = 0, index = 0;
  for (size_t i = 0; i < graph.size(); ++i) {
    const auto* factor = dynamic_cast<const InstallationOffsetFactor*>(graph[i].get());
    if (!factor) continue;
    if (std::set<gtsam::Key>(graph[i]->keys().begin(),graph[i]->keys().end()) != expected)
      throw std::logic_error("INSTALLATION_SUBSET_KEYS_CONFLICT");
    ++count; index = i;
  }
  if (count != 1) throw std::logic_error("INSTALLATION_SUBSET_NOT_EXACTLY_ONCE");
  return index;
}
}  // namespace liuo_tunnel
