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

#include <liuo_tunnel/uwb/known_map.hpp>
TEST(KnownMap, RejectInvalidFramesCovarianceAndExtrinsics) {
  nlohmann::json j = {
    {"schema_version", "liuo_known_map_v1"},
    {"units", "m"},
    {"axis_convention", "ENU"},
    {"valid", true},
    {"map_id", "survey-test"},
    {"mode", "surveyed_map"},
    {"frame", "map"},
    {"provenance", "synthetic_survey"},
    {"tag_lever_m", {.1, .2, .3}}};
  for (int i = 0; i < 4; i++) j["anchors"].push_back({{"id", i}, {"mean_m", {double(i), double(i % 2), 2.}}});
  std::vector<std::vector<double>> covariance(12, std::vector<double>(12, 0));
  for (int i = 0; i < 12; i++) covariance[i][i] = .04;
  covariance[0][3] = covariance[3][0] = .01;
  j["joint_covariance_m2"] = covariance;
  auto map = KnownMap::parse(j, {.1, .2, .3});
  EXPECT_DOUBLE_EQ(map.covariance(0, 3), .01);
  EXPECT_THROW(KnownMap::parse(j, {0, 0, 0}), std::invalid_argument);
  auto invalid = j;
  invalid["frame"] = "odom";
  EXPECT_THROW(KnownMap::parse(invalid, {.1, .2, .3}), std::invalid_argument);
  invalid = j;
  invalid["joint_covariance_m2"][0][0] = -.1;
  EXPECT_THROW(KnownMap::parse(invalid, {.1, .2, .3}), std::invalid_argument);
  invalid = j;
  invalid["anchors"][1]["id"] = 0;
  EXPECT_THROW(KnownMap::parse(invalid, {.1, .2, .3}), std::invalid_argument);
  invalid = j;
  invalid["units"] = "mm";
  EXPECT_THROW(KnownMap::parse(invalid, {.1, .2, .3}), std::invalid_argument);
}

#include <liuo_tunnel/uwb/measurement_queue.hpp>
TEST(RangeQueue, AsyncDelayDisorderDuplicateExpiryAndReset) {
  RangeQueue queue(3);
  RangeObservation o;
  o.producer = 1;
  o.session = 7;
  o.range = 800.08;
  o.variance = .01;
  o.stamp = 1.05;
  EXPECT_EQ(queue.push(o), "QUEUED");  // do not truncate valid noisy range.
  EXPECT_EQ(queue.push(o), "DUPLICATE");
  o.sequence = 2;
  o.stamp = 1.025;
  EXPECT_EQ(queue.push(o), "QUEUED");
  EXPECT_TRUE(queue.take_until(1).empty());
  std::map<double, gtsam::Key> states{{1, 0}, {1.1, 1}};
  auto pending = queue.take_until(1.1);
  ASSERT_EQ(pending.size(), 2);
  EXPECT_LT(pending[0].stamp, pending[1].stamp);
  EXPECT_NEAR(range_bracket(states, pending[0].stamp).beta, .25, 1e-10);
  EXPECT_NEAR(range_bracket(states, pending[1].stamp).beta, .5, 1e-10);
  EXPECT_EQ(range_bracket(states, 1.1).reason, "SYNC");
  EXPECT_EQ(range_bracket(states, 1.2).reason, "WAIT_RIGHT");
  EXPECT_EQ(range_bracket(states, .9).reason, "WINDOW_EXPIRED");
  EXPECT_EQ(range_bracket({{1, 0}, {1.2, 1}}, 1.05).reason, "TIME_BRACKET_GAP");
  EXPECT_EQ(queue.push(o), "DUPLICATE");  // after consumption still duplicate.
  queue.reset();
  o.sequence++;
  EXPECT_EQ(queue.push(o), "RETIRED_SESSION");
  o.session++;
  EXPECT_EQ(queue.push(o), "QUEUED");
}
TEST(RangeQueue, BoundAndLongSequenceHistory) {
  RangeQueue queue(2);
  RangeObservation o;
  o.range = 10;
  o.variance = .01;
  o.stamp = 1;
  for (uint64_t i = 0; i < 70000; i++) {
    o.sequence = i;
    ASSERT_EQ(queue.push(o), "QUEUED");
    queue.take_until(1);
  }
  o.sequence = 0;
  EXPECT_EQ(queue.push(o), "IDENTITY_TOO_OLD");
  o.sequence = 70000;
  EXPECT_EQ(queue.push(o), "QUEUED");
  o.sequence++;
  EXPECT_EQ(queue.push(o), "QUEUED");
  o.sequence++;
  EXPECT_EQ(queue.push(o), "QUEUE_LIMIT");
  EXPECT_EQ(queue.size(), 2);
}

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
TEST(KnownMap, StaticAnchorsSurviveLongWindowWithoutRepeatedPriors) {
  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::X;
  gtsam::ISAM2Params params;
  params.findUnusedFactorSlots = true;
  gtsam_points::IncrementalFixedLagSmootherExt smoother(10, params);
  gtsam::KeyVector keys;
  std::vector<gtsam::Point3> means;
  for (int j = 0; j < 13; j++) {
    keys.push_back(A(j));
    means.emplace_back(j * 200, (j % 2 ? 2.8 : -2.8), 2);
  }
  for (int i = 0; i < 501; i++) {
    double t = i * .1;
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    gtsam::FixedLagSmoother::KeyTimestampMap stamps;
    for (auto key : keys) stamps[key] = t;  // Every update, including >20s with no UWB.
    values.insert(X(i), gtsam::Pose3(gtsam::Rot3(), {double(i), 0, 0}));
    stamps[X(i)] = t;
    if (i == 0) {
      for (int j = 0; j < 13; j++) values.insert(keys[j], means[j]);
      gtsam::Matrix cov = gtsam::Matrix::Identity(39, 39) * .01;
      cov(0, 3) = cov(3, 0) = .002;
      factors.emplace_shared<SurveyedMapPrior>(keys, means, cov);
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(i), values.at<gtsam::Pose3>(X(i)), gtsam::noiseModel::Isotropic::Sigma(6, .01));
    } else
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), gtsam::Pose3(gtsam::Rot3(), {1, 0, 0}), gtsam::noiseModel::Isotropic::Sigma(6, .1));
    smoother.update(factors, values, stamps);
    size_t priors = 0;
    for (const auto& f : smoother.getFactors())
      if (dynamic_cast<SurveyedMapPrior*>(f.get())) priors++;
    EXPECT_EQ(priors, 1);
    EXPECT_LE(smoother.timestamps().size(), 115);
    for (auto key : keys) EXPECT_TRUE(smoother.calculateEstimate().exists(key));
  }
}

TEST(UwbFactors, GravityAlignmentDoesNotObservePositionOrYaw) {
  gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.2, -.3, .9), {4, 5, 6});
  auto measured = pose.rotation().unrotate(gtsam::Vector3::UnitZ());
  GravityAlignmentFactor f(0, measured, .001);
  gtsam::Values v;
  v.insert(0, pose);
  check_jacobian(f, v, {0});
  std::vector<gtsam::Matrix> H;
  f.unwhitenedError(v, &H);
  EXPECT_NEAR(H[0].rightCols<3>().norm(), 0, 1e-14);
  EXPECT_NEAR((H[0].leftCols<3>() * measured).norm(), 0, 1e-12);
  auto rotated = gtsam::Pose3(gtsam::Rot3::Rz(1.2).compose(pose.rotation()), {100, 200, 300});
  v.update(0, rotated);
  EXPECT_LT(f.unwhitenedError(v).norm(), 1e-12);
}

TEST(Smoother, DoglegAcceptsNewVariableAfterBatchBootstrap) {
  using gtsam::symbol_shorthand::X;
  gtsam::ISAM2Params params;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  params.findUnusedFactorSlots = true;
  gtsam_points::IncrementalFixedLagSmootherExt smoother(10, params);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::FixedLagSmoother::KeyTimestampMap stamps;
  for (int i = 0; i < 60; i++) {
    initial.insert(X(i), gtsam::Pose3(gtsam::Rot3(), {i * .1, 0, 0}));
    stamps[X(i)] = i * .1;
    if (i) graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(i - 1), X(i), gtsam::Pose3(gtsam::Rot3(), {.1, 0, 0}), gtsam::noiseModel::Isotropic::Sigma(6, .01));
  }
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1));
  smoother.update(graph, initial, stamps);
  smoother.calculateEstimate();
  graph.resize(0);
  initial.clear();
  stamps.clear();
  initial.insert(X(60), gtsam::Pose3(gtsam::Rot3(), {30, 5, 3}));
  stamps[X(60)] = 6;
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(X(59), X(60), gtsam::Pose3(gtsam::Rot3(), {.1, 0, 0}), gtsam::noiseModel::Isotropic::Sigma(6, .01));
  smoother.update(graph, initial, stamps);
  EXPECT_TRUE(smoother.calculateEstimate<gtsam::Pose3>(X(60)).matrix().allFinite());
}
