#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/static_edge_graph.hpp>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>

using namespace liuo_tunnel;
using gtsam::symbol_shorthand::A;

TEST(StaticEdgeGraph, UnknownEndpointsWaitThenOneFactorIsReplacedWithoutRepeatedWeight) {
  StaticEdgeGraph edges("network", "deployment", 32);
  StaticRangeBlock block{"network", "deployment", 4, 9, "physical-1", "independent-gaussian-v1", 10, .01};
  edges.offer({block});
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  EXPECT_TRUE(edges.stage(factors, values).empty());
  EXPECT_TRUE(factors.empty());
  edges.commit({});
  EXPECT_EQ(edges.pending(), 1u);
  values.insert(A(4), gtsam::Point3(0, 0, 0));
  values.insert(A(9), gtsam::Point3(10, 0, 0));
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(A(4), values.at<gtsam::Point3>(A(4)), noise);
  factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(A(9), values.at<gtsam::Point3>(A(9)), noise);
  auto remove = edges.stage(factors, values);
  gtsam::ISAM2 graph;
  auto result = graph.update(factors, values, remove);
  edges.commit(result.newFactorsIndices);
  EXPECT_EQ(edges.pending(), 0u);
  EXPECT_EQ(edges.factor_count(), 1u);
  for (int i = 0; i < 1000; ++i) edges.offer({block});
  factors.resize(0);
  EXPECT_TRUE(edges.stage(factors, values).empty());
  EXPECT_TRUE(factors.empty());
  edges.commit({});
  block.physical_id = "physical-2";
  block.range = 10.1;
  edges.offer({block});
  remove = edges.stage(factors, values);
  ASSERT_EQ(remove.size(), 1u);
  ASSERT_EQ(factors.size(), 1u);
  result = graph.update(factors, {}, remove);
  edges.commit(result.newFactorsIndices);
  EXPECT_EQ(graph.getFactorsUnsafe().nrFactors(), 3u);
  EXPECT_EQ(edges.factor_count(), 1u);
  EXPECT_EQ(edges.checkpoint().at("leaves").size(), 2u);
  edges.reset_graph();
  EXPECT_EQ(edges.fused(), 0u);
  EXPECT_EQ(edges.pending(), 2u);
  EXPECT_EQ(edges.factor_count(), 0u);
  EXPECT_TRUE(edges.checkpoint().at("leaves").empty());
  factors.resize(0);
  EXPECT_TRUE(edges.stage(factors, values).empty());
  ASSERT_EQ(factors.size(), 1u);
  edges.abort();
  EXPECT_EQ(edges.fused(), 0u);
  EXPECT_EQ(edges.pending(), 2u);
}
