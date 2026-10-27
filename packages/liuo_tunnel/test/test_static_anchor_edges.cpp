#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/static_anchor_edges.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>

using namespace liuo_tunnel;

TEST(StaticAnchorEdges, PhysicalIdentitySurvivesRepeatedAndReversedDelivery) {
  StaticAnchorEdges edges("network", "deployment", 16);
  StaticRangeBlock block{"network", "deployment", 9, 4, "exchange-1", "independent-gaussian-v1", 10.0, 0.01};
  auto transaction = edges.prepare({block});
  // Preparing work does not claim that the graph accepted it.
  EXPECT_EQ(edges.summary(4, 9).count, 0u);
  edges.commit(transaction);
  std::swap(block.first, block.second);
  for (int i = 0; i < 1000; ++i) edges.commit(edges.prepare({block}));
  EXPECT_EQ(edges.summary(4, 9).count, 1u);
  EXPECT_DOUBLE_EQ(edges.summary(4, 9).weight, 100.0);
  EXPECT_DOUBLE_EQ(edges.summary(4, 9).mean, 10.0);
  EXPECT_DOUBLE_EQ(edges.summary(4, 9).scatter, 0.0);
}

TEST(StaticAnchorEdges, PreparedReplacementIsCommittedOnlyAfterGraphSuccess) {
  StaticAnchorEdges edges("network", "deployment", 16);
  StaticRangeBlock block{"network", "deployment", 4, 9, "exchange-1", "independent-gaussian-v1", 10.0, 0.01};
  auto pending = edges.prepare({block});
  ASSERT_EQ(pending.updates().size(), 1u);
  const auto summary = pending.updates().begin()->second;
  gtsam::NonlinearFactorGraph factors;
  factors.emplace_shared<AnchorAnchorRangeFactor>(4, 9, summary.mean, summary.sigma(), summary.scatter);
  gtsam::ISAM2 graph;
  // The actual solver rejects missing anchor values. Its failure must not make
  // a later retry look like an already-fused physical exchange.
  EXPECT_ANY_THROW(graph.update(factors, gtsam::Values{}));
  EXPECT_EQ(edges.summary(4, 9).count, 0u);
  auto retry = edges.prepare({block});
  EXPECT_EQ(retry.updates().size(), 1u);
  gtsam::ISAM2 recovered;
  gtsam::Values initial;
  initial.insert(4, gtsam::Point3(0, 0, 0));
  initial.insert(9, gtsam::Point3(10, 0, 0));
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(4, initial.at<gtsam::Point3>(4), noise);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(9, initial.at<gtsam::Point3>(9), noise);
  ASSERT_NO_THROW(recovered.update(factors, initial));
  edges.commit(retry);
  EXPECT_EQ(edges.summary(4, 9).count, 1u);
  EXPECT_TRUE(edges.prepare({block}).updates().empty());
  EXPECT_THROW(edges.commit(pending), std::logic_error);
}

TEST(StaticAnchorEdges, ConflictingIdentityAndFullLedgerNeverEvictPhysicalHistory) {
  StaticAnchorEdges edges("network", "deployment", 1);
  StaticRangeBlock block{"network", "deployment", 4, 9, "exchange-1", "independent-gaussian-v1", 10.0, 0.01};
  edges.commit(edges.prepare({block}));
  auto changed = block;
  changed.range = 11;
  EXPECT_THROW(edges.prepare({changed}), std::invalid_argument);
  changed = block;
  changed.deployment = "moved-anchor";
  EXPECT_THROW(edges.prepare({changed}), std::invalid_argument);
  changed = block;
  changed.physical_id = "exchange-2";
  EXPECT_THROW(edges.prepare({changed}), std::length_error);
  EXPECT_TRUE(edges.prepare({block}).updates().empty());
  EXPECT_EQ(edges.summary(4, 9).count, 1u);
}

TEST(StaticAnchorEdges, PhysicalIdentifiersHaveBoundedStorage) {
  StaticAnchorEdges edges("network", "deployment", 1);
  StaticRangeBlock block{"network", "deployment", 4, 9, std::string(257, 'x'), "independent-gaussian-v1", 10.0, 0.01};
  EXPECT_THROW(edges.prepare({block}), std::invalid_argument);
  EXPECT_EQ(edges.summary(4, 9).count, 0u);
}

TEST(StaticAnchorEdges, RestoredAbsorbedPrefixCannotBeCountedByCumulativeDelivery) {
  StaticAnchorEdges first_run("network", "deployment", 16);
  StaticRangeBlock old{"network", "deployment", 4, 9, "exchange-1", "independent-gaussian-v1", 10.0, 0.01};
  first_run.commit(first_run.prepare({old}));
  // The map exporter embeds this exact ledger alongside the joint posterior.
  // It is not a second coordinate prior or a list of runtime factor indices.
  auto checkpoint = first_run.absorbed_checkpoint();
  StaticAnchorEdges reused("network", "deployment", 16, checkpoint);
  EXPECT_TRUE(reused.prepare({old}).updates().empty());
  auto increment = old;
  increment.physical_id = "exchange-2";
  increment.range = 12;
  auto cumulative = reused.prepare({old, increment});
  ASSERT_EQ(cumulative.updates().size(), 1u);
  EXPECT_EQ(cumulative.updates().begin()->second.count, 1u);
  EXPECT_DOUBLE_EQ(cumulative.updates().begin()->second.mean, 12.0);
  EXPECT_DOUBLE_EQ(cumulative.updates().begin()->second.weight, 100.0);
  reused.commit(cumulative);
  StaticAnchorEdges third("network", "deployment", 16, reused.absorbed_checkpoint());
  EXPECT_TRUE(third.prepare({increment, old}).updates().empty());
  EXPECT_THROW((StaticAnchorEdges("network", "new-deployment", 16, checkpoint)), std::invalid_argument);
}

TEST(StaticAnchorEdges, CheckpointRejectsUnrepresentableIdsAndDuplicateLeaves) {
  StaticAnchorEdges edges("network", "deployment", 16);
  StaticRangeBlock sample{"network", "deployment", 4, 9, "exchange-1", "independent-gaussian-v1", 10.0, 0.01};
  edges.commit(edges.prepare({sample}));
  const auto valid = nlohmann::json::parse(edges.absorbed_checkpoint().dump());
  for (const auto& bad_id : {nlohmann::json(-1), nlohmann::json(4294967296ULL), nlohmann::json(4.5)}) {
    auto invalid = valid;
    invalid["leaves"][0]["first"] = bad_id;
    EXPECT_THROW((StaticAnchorEdges("network", "deployment", 16, invalid)), std::invalid_argument);
  }
  auto duplicate = valid;
  duplicate["leaves"].push_back(valid["leaves"][0]);
  EXPECT_THROW((StaticAnchorEdges("network", "deployment", 16, duplicate)), std::invalid_argument);
}
