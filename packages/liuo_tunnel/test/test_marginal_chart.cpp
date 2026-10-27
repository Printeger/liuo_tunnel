#include <gtest/gtest.h>
#include <liuo_tunnel/optimizers/isam2_ext.hpp>
#include <liuo_tunnel/uwb/local_installation.hpp>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <liuo_tunnel/optimizers/manifold_marginal_factor.hpp>

namespace {
// Test-only Cartesian observation gives the pair a known relative displacement.
// It is not installed in the estimator or used as a production pose prior.
class RelativePoint : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
public:
  RelativePoint(gtsam::Key x, gtsam::Key a, const gtsam::Point3& target = gtsam::Point3(0, 2.8, 1))
  : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(3, .1), x, a),
    target_(target) {}
  gtsam::Vector evaluateError(const gtsam::Pose3& x, const gtsam::Point3& a, gtsam::OptionalMatrixType Hx = nullptr, gtsam::OptionalMatrixType Ha = nullptr) const override {
    gtsam::Matrix36 X;
    gtsam::Matrix33 A;
    const auto p = x.transformTo(a, X, A);
    if (Hx) *Hx = X;
    if (Ha) *Ha = A;
    return p - target_;
  }

private:
  gtsam::Point3 target_;
};
}  // namespace

TEST(MarginalChart, NewFiniteFactorUsesConsistentExistingPoseAndNewAnchorChart) {
  using namespace gtsam;
  const Key x = Symbol('x', 0), a = Symbol('a', 1);
  ISAM2Params params;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = .05;
  params.setOptimizationParams(ISAM2DoglegParams());
  liuo_tunnel::solver::ISAM2Ext isam(params);
  NonlinearFactorGraph initial;
  initial.emplace_shared<PriorFactor<Pose3>>(x, Pose3(), noiseModel::Isotropic::Sigma(6, 100));
  Values values;
  values.insert(x, Pose3());
  isam.update(initial, values);
  NonlinearFactorGraph move;
  move.emplace_shared<PriorFactor<Pose3>>(x, Pose3(Rot3(), Point3(.02, 0, 0)), noiseModel::Isotropic::Sigma(6, .01));
  isam.update(move);
  const auto current = isam.calculateEstimate().at<Pose3>(x);
  ASSERT_NEAR(current.x(), .02, 1e-7);
  ASSERT_NEAR(isam.getLinearizationPoint().at<Pose3>(x).x(), 0, 1e-10);
  const Point3 local(9.99, 2.8, 1);
  Values added;
  added.insert(a, current.transformFrom(local));
  liuo_tunnel::LocalInstallationObservation patch{};
  patch.normal = Vector3::UnitY();
  patch.tangent = Vector3::UnitX();
  patch.up = Vector3::UnitZ();
  patch.width = 6;
  patch.sigma = .02;
  for (auto& floor : patch.floors) {
    floor.longitudinal_axis = patch.tangent;
    floor.minimum_longitudinal = -10;
    floor.maximum_longitudinal = 10;
  }
  NonlinearFactorGraph factors;
  factors.emplace_shared<RelativePoint>(x, a, local);
  factors.emplace_shared<liuo_tunnel::LocalInstallationFactor>(x, a, patch, 2.8);
  ASSERT_NO_THROW(isam.update(factors, added));
  const auto estimate = isam.calculateEstimate();
  EXPECT_LT((estimate.at<Pose3>(x).transformTo(estimate.at<Point3>(a)) - local).norm(), 1e-6);
  EXPECT_TRUE(std::isfinite(factors.error(estimate)));
}

TEST(MarginalChart, FiniteFactorEndpointsRelinearizeTogetherAtSupportBoundary) {
  using namespace gtsam;
  const Key x = Symbol('x', 0), a = Symbol('a', 1);
  ISAM2Params params;
  params.relinearizeSkip = 1;
  FastMap<char, Vector> thresholds;
  thresholds['x'] = Vector6::Constant(.01);
  thresholds['a'] = Vector3::Constant(.05);
  params.relinearizeThreshold = thresholds;
  params.setOptimizationParams(ISAM2DoglegParams());
  liuo_tunnel::solver::ISAM2Ext isam(params);
  const Point3 local(9.99, 2.8, 1);
  liuo_tunnel::LocalInstallationObservation patch{};
  patch.normal = Vector3::UnitY();
  patch.tangent = Vector3::UnitX();
  patch.up = Vector3::UnitZ();
  patch.width = 6;
  patch.sigma = .02;
  for (auto& floor : patch.floors) {
    floor.longitudinal_axis = patch.tangent;
    floor.minimum_longitudinal = -10;
    floor.maximum_longitudinal = 10;
  }
  NonlinearFactorGraph graph;
  graph.emplace_shared<PriorFactor<Pose3>>(x, Pose3(), noiseModel::Isotropic::Sigma(6, 100));
  graph.emplace_shared<RelativePoint>(x, a, local);
  graph.emplace_shared<liuo_tunnel::LocalInstallationFactor>(x, a, patch, 2.8);
  Values values;
  values.insert(x, Pose3());
  values.insert(a, local);
  isam.update(graph, values);
  NonlinearFactorGraph move;
  move.emplace_shared<PriorFactor<Pose3>>(x, Pose3(Rot3(), Point3(-.02, 0, 0)), noiseModel::Isotropic::Sigma(6, .01));
  isam.update(move);
  const auto before = isam.calculateEstimate();
  ASSERT_NEAR(before.at<Pose3>(x).x(), -.02, 1e-7);
  ASSERT_TRUE(std::isfinite(graph.error(before)));
  ASSERT_NO_THROW(isam.update());
  const auto estimate = isam.calculateEstimate();
  EXPECT_LT((estimate.at<Pose3>(x).transformTo(estimate.at<Point3>(a)) - local).norm(), 1e-6);
  EXPECT_TRUE(std::isfinite(graph.error(estimate)));
}

TEST(MarginalChart, FinitePatchSurvivesCommonMotionAfterAnchorMarginalization) {
  using namespace gtsam;
  const Key x0 = Symbol('x', 0), x1 = Symbol('x', 1), x2 = Symbol('x', 2), a = Symbol('a', 1);
  ISAM2Params params;
  params.relinearizeSkip = 1;
  params.relinearizeThreshold = .01;
  ISAM2DoglegParams dogleg;
  dogleg.initialDelta = 100;
  params.setOptimizationParams(dogleg);
  liuo_tunnel::solver::ISAM2Ext isam(params);
  NonlinearFactorGraph initial;
  initial.emplace_shared<PriorFactor<Pose3>>(x0, Pose3(), noiseModel::Isotropic::Sigma(6, 100));
  initial.emplace_shared<BetweenFactor<Pose3>>(x0, x1, Pose3(), noiseModel::Isotropic::Sigma(6, .1));
  initial.emplace_shared<RelativePoint>(x0, a);
  Values values;
  values.insert(x0, Pose3());
  values.insert(x1, Pose3());
  values.insert(a, Point3(0, 2.8, 1));
  FastMap<Key, int> ordering;
  ordering[x0] = 0;
  ordering[x1] = 1;
  ordering[a] = 1;
  isam.update(initial, values, {}, ordering);
  isam.marginalizeLeaves(FastList<Key>{x0});
  NonlinearFactorGraph next;
  next.emplace_shared<BetweenFactor<Pose3>>(x1, x2, Pose3(), noiseModel::Isotropic::Sigma(6, .1));
  next.emplace_shared<RelativePoint>(x2, a);
  liuo_tunnel::LocalInstallationObservation patch{};
  patch.normal = Vector3::UnitY();
  patch.tangent = Vector3::UnitX();
  patch.up = Vector3::UnitZ();
  patch.width = 6;
  patch.sigma = .02;
  for (auto& floor : patch.floors) {
    floor.longitudinal_axis = patch.tangent;
    floor.minimum_longitudinal = -10;
    floor.maximum_longitudinal = 10;
  }
  next.emplace_shared<liuo_tunnel::LocalInstallationFactor>(x2, a, patch, 2.8);
  Values new_values;
  new_values.insert(x2, Pose3());
  isam.update(next, new_values);
  NonlinearFactorGraph motion;
  motion.emplace_shared<PriorFactor<Pose3>>(x2, Pose3(Rot3(), Point3(30, 0, 0)), noiseModel::Isotropic::Sigma(6, .1));
  ASSERT_NO_THROW({
    isam.update(motion);
    for (int i = 0; i < 8; ++i) isam.update();
  });
  const auto estimate = isam.calculateEstimate();
  EXPECT_NEAR(estimate.at<Pose3>(x2).x(), 30, .01);
  EXPECT_LT((estimate.at<Pose3>(x2).transformTo(estimate.at<Point3>(a)) - Point3(0, 2.8, 1)).norm(), .01);
  EXPECT_TRUE(std::isfinite(next.error(estimate)));
}

TEST(MarginalChart, ImmutableQuadraticHasCorrectManifoldGradientAndCrossInformation) {
  using namespace gtsam;
  const Key x = Symbol('x', 0), a = Symbol('a', 1);
  Matrix D(12, 9);
  for (int i = 0; i < 12; ++i)
    for (int j = 0; j < 9; ++j) D(i, j) = std::sin(1.7 * i + .3 * j) + (i == j ? 3. : 0.);
  const Matrix H = D.transpose() * D;
  const Vector g = Vector::LinSpaced(9, -.2, .3);
  auto quadratic = make_shared<HessianFactor>(x, a, H.topLeftCorner(6, 6), H.topRightCorner(6, 3), g.head(6), H.bottomRightCorner(3, 3), g.tail(3), 7.);
  Values reference;
  reference.insert(x, Pose3(Rot3::RzRyRx(.2, -.1, .3), Point3(5, 6, 7)));
  reference.insert(a, Point3(1, 2, 3));
  liuo_tunnel::solver::ManifoldMarginalFactor prior(quadratic, reference);
  VectorValues shift;
  shift.insert(x, (Vector6() << .4, -.3, .2, 30, -5, 8).finished());
  shift.insert(a, Vector3(29, -4, 7));
  const auto current = reference.retract(shift);
  const Ordering order{x, a};
  const auto dense = [&](const Values& v) { return reference.localCoordinates(v).vector(order); };
  Matrix numeric_J(9, 9);
  Vector numeric_gradient(9);
  constexpr double epsilon = 1e-5;
  for (int i = 0; i < 9; ++i) {
    auto d = current.zeroVectors();
    d.at(i < 6 ? x : a)[i < 6 ? i : i - 6] = epsilon;
    const auto plus = current.retract(d), minus = current.retract(-1. * d);
    numeric_J.col(i) = (dense(plus) - dense(minus)) / (2 * epsilon);
    numeric_gradient[i] = (prior.error(plus) - prior.error(minus)) / (2 * epsilon);
  }
  NonlinearFactorGraph graph;
  graph.push_back(prior.clone());
  const Matrix actual = graph.linearize(current)->augmentedHessian(order);
  const Matrix expected_H = numeric_J.transpose() * H * numeric_J;
  EXPECT_LT((actual.topLeftCorner(9, 9) - expected_H).norm() / expected_H.norm(), 1e-8);
  EXPECT_LT((actual.topRightCorner(9, 1) + numeric_gradient).norm() / numeric_gradient.norm(), 1e-8);
  EXPECT_GT((actual.col(9).head(6) - (g - H * dense(current)).head(6)).norm(), 1.);
  EXPECT_GT(actual.block(0, 6, 6, 3).norm(), 1.);
  EXPECT_NEAR(.5 * actual(9, 9), prior.error(current), 1e-8);
  EXPECT_LT((quadratic->augmentedInformation() - HessianFactor(*prior.factor()).augmentedInformation()).norm(), 1e-12);
  EXPECT_NEAR(prior.clone()->error(current), prior.error(current), 1e-10);
  EXPECT_LT((prior.linearizationPoint()->localCoordinates(reference).vector()).norm(), 1e-12);
}
