#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/range_factors.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PoseRotationPrior.h>
#include <random>
using namespace liuo_tunnel;
namespace {
void check_jacobian(const gtsam::NoiseModelFactor& f, const gtsam::Values& values, const std::set<gtsam::Key>& poses) {
  std::vector<gtsam::Matrix> actual;
  f.unwhitenedError(values, &actual);
  for (size_t k = 0; k < f.keys().size(); k++) {
    const auto key = f.keys()[k];
    const int n = poses.count(key) ? 6 : 3;
    gtsam::Matrix numerical(actual[k].rows(), n);
    for (int j = 0; j < n; j++) {
      gtsam::Values plus = values, minus = values;
      constexpr double h = 1e-5;
      if (n == 6) {
        gtsam::Vector6 d = gtsam::Vector6::Zero();
        d[j] = h;
        plus.update(key, values.at<gtsam::Pose3>(key).retract(d));
        minus.update(key, values.at<gtsam::Pose3>(key).retract(-d));
      } else {
        gtsam::Point3 d = gtsam::Point3::Zero();
        d[j] = h;
        plus.update(key, gtsam::Point3(values.at<gtsam::Point3>(key) + d));
        minus.update(key, gtsam::Point3(values.at<gtsam::Point3>(key) - d));
      }
      numerical.col(j) = (f.unwhitenedError(plus) - f.unwhitenedError(minus)) / (2 * h);
    }
    for (int i = 0; i < numerical.rows(); i++)
      for (int j = 0; j < n; j++) {
        double delta = std::abs(actual[k](i, j) - numerical(i, j));
        if (std::abs(numerical(i, j)) < 1e-3)
          EXPECT_LT(delta, 1e-6);
        else
          EXPECT_LT(delta / std::abs(numerical(i, j)), 1e-5);
      }
  }
}
}  // namespace
TEST(UwbFactors, RandomRightRetractDerivatives) {
  std::mt19937 rng(821);
  std::normal_distribution<double> normal;
  auto vec = [&] { return gtsam::Vector3(normal(rng), normal(rng), normal(rng)); };
  for (int trial = 0; trial < 150; trial++) {
    gtsam::Values v;
    v.insert(0, gtsam::Pose3(gtsam::Rot3::Expmap(vec() * .7), gtsam::Point3(vec() * 3)));
    v.insert(1, gtsam::Pose3(gtsam::Rot3::Expmap(vec() * .7), gtsam::Point3(vec() * 3)));
    v.insert(2, gtsam::Point3(vec() * 20));
    v.insert(3, gtsam::Point3(vec() * 20));
    auto lever = vec();
    auto a = v.at<gtsam::Point3>(2);
    for (bool variable : {false, true}) {
      auto ak = variable ? std::optional<gtsam::Key>(2) : std::nullopt;
      TagAnchorRangeFactor sync(0, std::nullopt, ak, a, lever, 0, 20, .1);
      check_jacobian(sync, v, {0, 1});
      for (double beta : {.25, .5, .93}) {
        TagAnchorRangeFactor async(0, 1, ak, a, lever, beta, 20, .1);
        check_jacobian(async, v, {0, 1});
      }
    }
    AnchorAnchorRangeFactor aa(2, 3, 20, .1);
    check_jacobian(aa, v, {});
  }
}
TEST(UwbFactors, ObservableNoiselessGraphRecoversPose) {
  const gtsam::Point3 lever(.13, -.2, .4);
  const gtsam::Pose3 truth(gtsam::Rot3::RzRyRx(.4, -.2, .7), {2, -3, 5});
  const std::vector<gtsam::Point3> anchors{{-10, -10, 0}, {10, -10, 2}, {-10, 10, 8}, {10, 10, 12}};
  gtsam::NonlinearFactorGraph graph;
  // Orientation has its OWN observation; a scalar range is not an attitude sensor.
  graph.emplace_shared<gtsam::PoseRotationPrior<gtsam::Pose3>>(0, truth.rotation(), gtsam::noiseModel::Isotropic::Sigma(3, .01));
  for (const auto& a : anchors) graph.emplace_shared<TagAnchorRangeFactor>(0, std::nullopt, std::nullopt, a, lever, 0, (truth.transformFrom(lever) - a).norm(), .1);
  gtsam::Values v;
  gtsam::Vector6 delta;
  delta << .04, -.05, .03, .4, -.3, .2;
  v.insert(0, truth.retract(delta));
  gtsam::LevenbergMarquardtParams params;
  params.setAbsoluteErrorTol(1e-12);
  params.setRelativeErrorTol(1e-12);
  auto result = gtsam::LevenbergMarquardtOptimizer(graph, v, params).optimize().at<gtsam::Pose3>(0);
  EXPECT_LT((result.translation() - truth.translation()).norm(), 1e-4);
  EXPECT_LT(gtsam::Rot3::Logmap(truth.rotation().between(result.rotation())).norm(), 1e-4);
}
TEST(UwbFactors, InterpolationEndpointsAndAcceleratedMotionEnvelope) {
  gtsam::Pose3 l(gtsam::Rot3::Rz(.1), {1, 2, 3}), r(gtsam::Rot3::Rz(.12), {1.5, 2, 3});
  gtsam::Point3 lever(.3, .2, .1), anchor(100, 2, 3);
  EXPECT_LT((interpolated_tag(l, r, 0, lever) - l.transformFrom(lever)).norm(), 1e-12);
  EXPECT_LT((interpolated_tag(l, r, 1, lever) - r.transformFrom(lever)).norm(), 1e-12);
  for (double t : {.025, .05}) {
    gtsam::Pose3 start(gtsam::Rot3(), {0, 0, 0}), end(gtsam::Rot3::Rz(.02), {.505, 0, 0});
    gtsam::Pose3 actual(gtsam::Rot3::Rz(.2 * t), {5 * t + .5 * t * t, 0, 0});
    auto q = interpolated_tag(start, end, t / .1, lever);
    EXPECT_LT(std::abs((q - anchor).norm() - (actual.transformFrom(lever) - anchor).norm()), .01);
  }
}
TEST(UwbFactors, CorrelatedMapPriorRetainsCrossTerms) {
  gtsam::Matrix covariance = gtsam::Matrix::Identity(6, 6) * .04;
  covariance(0, 3) = covariance(3, 0) = .02;
  std::vector<gtsam::Point3> means{{0, 0, 0}, {1, 2, 3}};
  SurveyedMapPrior f({2, 3}, means, covariance);
  gtsam::Values v;
  v.insert(2, gtsam::Point3(.1, .2, .3));
  v.insert(3, gtsam::Point3(1.3, 2.2, 3.1));
  check_jacobian(f, v, {});
  auto residual = f.unwhitenedError(v);
  EXPECT_NEAR(2 * f.error(v), residual.dot(covariance.inverse() * residual), 1e-10);
}
TEST(UwbFactors, RejectSingularGeometry) {
  gtsam::Values v;
  v.insert(0, gtsam::Pose3());
  TagAnchorRangeFactor f(0, std::nullopt, std::nullopt, {.1, .2, .3}, {.1, .2, .3}, 0, 1, .1);
  EXPECT_THROW(f.unwhitenedError(v), std::domain_error);
}
