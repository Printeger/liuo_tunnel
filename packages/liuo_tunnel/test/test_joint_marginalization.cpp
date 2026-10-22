#include <gtest/gtest.h>
#include <gtsam/geometry/Point3.h>
#include <liuo_tunnel/uwb/joint_marginalization.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
using namespace liuo_tunnel;
TEST(JointMarginalization, RetainsJointSchurInformationAndLinearizedCost) {
  const auto x = gtsam::Symbol('x', 0), a = gtsam::Symbol('a', 7), l = gtsam::Symbol('l', 0);
  gtsam::NonlinearFactorGraph graph;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, .3);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(x, gtsam::Point3(1, 2, 3), noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(x, a, gtsam::Point3(4, 2, -1), noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(x, l, gtsam::Point3(0, 1, 2), noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(a, l, gtsam::Point3(-3, -1, 3), noise);
  gtsam::Values values;
  values.insert(x, gtsam::Point3(.9, 2.1, 3.1));
  values.insert(a, gtsam::Point3(5.2, 3.8, 2));
  values.insert(l, gtsam::Point3(1, 3, 5.1));
  const gtsam::Ordering ordering{x, a, l}, retained{a, l};
  auto full = graph.linearize(values)->augmentedHessian(ordering);
  auto result = marginalize_joint(graph, values, gtsam::Ordering{x});
  EXPECT_EQ(result.retired_factors, 3);
  EXPECT_FALSE(result.values.exists(x));
  EXPECT_EQ(graph.size(), 4); // Transaction does not mutate source evidence.
  const gtsam::Matrix expected = full.bottomRightCorner(7, 7) - full.bottomLeftCorner(7, 3) * full.topLeftCorner(3, 3).inverse() * full.topRightCorner(3, 7);
  const auto actual = result.graph.linearize(result.values)->augmentedHessian(retained);
  EXPECT_LT((actual - expected).norm(), 1e-9);
  EXPECT_GT(actual.block(0, 3, 3, 3).norm(), 1); // Correlations must not be discarded.
  for (double shift : {-.1, 0., .15}) {
    gtsam::Vector delta = gtsam::Vector::Constant(6, shift);
    const gtsam::Vector old = full.topLeftCorner(3, 3).inverse() * (full.block(0, 9, 3, 1) - full.block(0, 3, 3, 6) * delta);
    auto trial = values;
    trial.update(x, gtsam::Point3(values.at<gtsam::Point3>(x) + old));
    trial.update(a, gtsam::Point3(values.at<gtsam::Point3>(a) + delta.head(3)));
    trial.update(l, gtsam::Point3(values.at<gtsam::Point3>(l) + delta.tail(3)));
    EXPECT_NEAR(graph.error(trial), result.graph.error(trial), 1e-9);
  }
  EXPECT_THROW(marginalize_joint(graph, values, gtsam::Ordering{x, x}), std::invalid_argument);
  EXPECT_THROW(marginalize_joint(graph, values, gtsam::Ordering{gtsam::Symbol('x', 99)}), std::invalid_argument);
}
