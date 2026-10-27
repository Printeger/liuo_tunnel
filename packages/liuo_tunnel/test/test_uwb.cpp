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
// Independent circular-path oracle; the chord/SLERP implementation is not used
// to construct the intermediate reference. ||p''|| dt^2 / 8 bounds chord error.
TEST(UwbFactors, HighwayCurvatureAndBrakingInterpolationBound) {
  constexpr double duration = .1, radius = 2998.125, max_speed = 80. / 3.6;
  const gtsam::Point3 lever(.5, -.2, .1);
  const gtsam::Pose3 gauge(gtsam::Rot3::RzRyRx(.02, -.005, .7), {21, -13, 4});
  const std::vector<gtsam::Point3> anchors{{800, 5, 3}, {-700, -5, 2}, {0, 5, 3}};
  for (double acceleration : {-1.5, 0., 1.}) {
    const double v0 = max_speed - std::max(0., acceleration) * duration;
    auto pose = [&](double t) {
      const double angle = (v0 * t + .5 * acceleration * t * t) / radius;
      return gauge.compose(gtsam::Pose3(gtsam::Rot3::Rz(angle), {radius * std::sin(angle), radius * (1 - std::cos(angle)), 0}));
    };
    const double translation_acceleration = std::hypot(acceleration, max_speed * max_speed / radius);
    const double lever_acceleration = lever.norm() * (std::abs(acceleration) / radius + std::pow(max_speed / radius, 2));
    const double bound = (translation_acceleration + lever_acceleration) * duration * duration / 8;
    ASSERT_LT(bound, .00190);
    for (int i = 0; i <= 100; ++i) {
      const double t = duration * i / 100.;
      const auto estimated = interpolated_tag(pose(0), pose(duration), t / duration, lever);
      const auto actual = pose(t).transformFrom(lever);
      EXPECT_LE((estimated - actual).norm(), bound + 1e-9);
      for (const auto& anchor : anchors) {
        EXPECT_LE(std::abs((estimated - anchor).norm() - (actual - anchor).norm()), bound + 1e-9);
      }
    }
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

#include <liuo_tunnel/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>
TEST(KnownMap, StaticAnchorsSurviveLongWindowWithoutRepeatedPriors) {
  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::X;
  gtsam::ISAM2Params params;
  params.findUnusedFactorSlots = true;
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(10, params);
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
  gtsam::ISAM2DoglegParams dogleg;
  dogleg.initialDelta = 0.01;
  dogleg.adaptationMode = gtsam::DoglegOptimizerImpl::ONE_STEP_PER_ITERATION;
  params.setOptimizationParams(dogleg);
  params.relinearizeSkip = 1;
  params.findUnusedFactorSlots = true;
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(10, params);
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  gtsam::FixedLagSmoother::KeyTimestampMap stamps;
  for (int i = 0; i < 60; i++) {
    initial.insert(X(i), gtsam::Pose3(gtsam::Rot3::Rz(.1), {i * .1 + 1, .2, .3}));
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

#include <liuo_tunnel/common/state_covariance.hpp>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/JacobianFactor.h>
TEST(StateCovariance, JointMarginalPreservesCorrelatedPoseVelocity) {
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(10);
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3());
  initial.insert(V(0), gtsam::Vector3::Zero().eval());
  gtsam::Matrix root = gtsam::Matrix::Identity(9, 9);
  root.block<3, 3>(0, 6) = .4 * gtsam::Matrix33::Identity();
  root.block<3, 3>(3, 6) = -.2 * gtsam::Matrix33::Identity();
  gtsam::JacobianFactor linear(X(0), root.leftCols(6), V(0), root.rightCols(3), gtsam::Vector::Zero(9), gtsam::noiseModel::Unit::Create(9));
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::LinearContainerFactor>(linear, initial);
  smoother.update(graph, initial, {{X(0), 1}, {V(0), 1}});
  auto covariance = liuo_tunnel::joint_covariance(smoother.getISAM2(), X(0), V(0));
  EXPECT_LT((covariance - (root.transpose() * root).inverse()).norm(), 1e-10);
  EXPECT_GT((covariance.block<6, 3>(0, 6)).norm(), .1);
  EXPECT_LT((covariance.topLeftCorner<6, 6>() - smoother.marginalCovariance(X(0))).norm(), 1e-10);
}

TEST(KnownMap, BootstrapBatchOlderThanLagMarginalizesNewKeys) {
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(10);
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  gtsam::FixedLagSmoother::KeyTimestampMap times;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, .1);
  for (int i = 0; i <= 20; ++i) {
    auto key = gtsam::Symbol('p', i);
    gtsam::Point3 point(i, 0, 0);
    values.insert(key, point);
    times[key] = i;
    if (i == 0)
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(key, point, noise);
    else
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(gtsam::Symbol('p', i - 1), key, gtsam::Point3(1, 0, 0), noise);
  }
  ASSERT_NO_THROW(smoother.update(factors, values, times));
  EXPECT_EQ(smoother.timestamps().size(), 11);
  EXPECT_FALSE(smoother.getLinearizationPoint().exists(gtsam::Symbol('p', 0)));
  EXPECT_LT((smoother.calculateEstimate<gtsam::Point3>(gtsam::Symbol('p', 20)) - gtsam::Point3(20, 0, 0)).norm(), 1e-10);
}

TEST(StateCovariance, SelectedAncestorsMatchDenseCorrelatedMarginal) {
  gtsam::GaussianFactorGraph graph;
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, .2);
  auto I = gtsam::Matrix33::Identity().eval();
  auto zero = gtsam::Vector3::Zero().eval();
  graph.emplace_shared<gtsam::JacobianFactor>(0, I, zero, noise);
  for (gtsam::Key k = 1; k < 9; ++k) {
    gtsam::Matrix33 coupling = I;
    coupling(0, 1) = .13 * k;
    graph.emplace_shared<gtsam::JacobianFactor>((k - 1) / 2, -coupling, k, I, zero, noise);
  }
  auto tree = graph.eliminateMultifrontal(gtsam::Ordering{8, 7, 6, 5, 4, 3, 2, 1, 0});
  auto information = graph.hessian(gtsam::Ordering{0, 1, 2, 3, 4, 5, 6, 7, 8}).first;
  gtsam::Matrix reference = information.inverse();
  const gtsam::KeyVector selected{7, 2, 8};  // siblings, shared ancestors, non-key order
  auto actual = selected_covariance(*tree, selected);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_LT((actual.block<3, 3>(3 * i, 3 * j) - reference.block<3, 3>(3 * selected[i], 3 * selected[j])).norm(), 1e-10);
  EXPECT_THROW(selected_covariance(*tree, {7, 7}), std::invalid_argument);
}

#include <gtsam/linear/HessianFactor.h>
TEST(UwbFactors, StaticPairSummaryPreservesRawCostGradientAndInformation) {
  gtsam::Values values;
  values.insert(1, gtsam::Point3(1, -.2, .3));
  values.insert(2, gtsam::Point3(6, 1, 2));
  gtsam::NonlinearFactorGraph raw;
  GaussianRangeSummary summary;
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0, .1);
  for (int i = 0; i < 1000; ++i) {
    double measurement = 5.5 + noise(rng), variance = .01 + .001 * (i % 7);
    raw.emplace_shared<AnchorAnchorRangeFactor>(1, 2, measurement, std::sqrt(variance));
    summary.add(measurement, variance);
  }
  AnchorAnchorRangeFactor aggregated(1, 2, summary.mean, summary.sigma(), summary.scatter);
  EXPECT_EQ(summary.count, 1000);
  EXPECT_NEAR(raw.error(values), aggregated.error(values), 1e-8);
  gtsam::HessianFactor raw_hessian(*raw.linearize(values));
  gtsam::HessianFactor aggregate_hessian(*aggregated.linearize(values));
  EXPECT_LT((raw_hessian.augmentedInformation() - aggregate_hessian.augmentedInformation()).norm() / raw_hessian.augmentedInformation().norm(), 1e-12);
  check_jacobian(aggregated, values, {});
  EXPECT_NEAR(aggregated.clone()->error(values), raw.error(values), 1e-8);
  EXPECT_THROW(summary.add(5, 0), std::invalid_argument);
}

TEST(KnownMap, ReplacingStaticPairSummaryBoundsHistoryWithoutRepeatingMapPrior) {
  using gtsam::symbol_shorthand::A;
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(10);
  GaussianRangeSummary summary;
  std::optional<gtsam::FactorIndex> previous;
  const std::vector<gtsam::Point3> means{{0, 0, 0}, {5, 1, 2}};
  for (int i = 0; i < 501; ++i) {
    summary.add((means[0] - means[1]).norm() + .1 * std::sin(i), .01);
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    if (i == 0) {
      initial.insert(A(0), means[0]);
      initial.insert(A(1), means[1]);
      factors.emplace_shared<SurveyedMapPrior>(gtsam::KeyVector{A(0), A(1)}, means, gtsam::Matrix::Identity(6, 6) * .01);
    }
    auto local_index = factors.size();
    factors.emplace_shared<AnchorAnchorRangeFactor>(A(0), A(1), summary.mean, summary.sigma(), summary.scatter);
    gtsam::FactorIndices remove;
    if (previous) remove.push_back(*previous);
    smoother.update(factors, initial, {{A(0), i * .1}, {A(1), i * .1}}, remove);
    previous = smoother.getISAM2Result().newFactorsIndices.at(local_index);
    EXPECT_EQ(smoother.getFactors().nrFactors(), 2);
    EXPECT_LE(smoother.getFactors().size(), 3);
    EXPECT_EQ(smoother.timestamps().size(), 2);
    EXPECT_TRUE(smoother.calculateEstimate<gtsam::Point3>(A(1)).allFinite());
  }
  EXPECT_EQ(summary.count, 501);
}

#include <gtsam/nonlinear/LinearContainerFactor.h>
TEST(Smoother, DoglegMarginalPriorMustContributeToNonlinearAcceptanceCost) {
  gtsam::ISAM2Params params;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  params.relinearizeSkip = 1;
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(1.0, params);
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, .1);
  for (size_t i = 0; i < 3; ++i) {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    initial.insert(i, gtsam::Point3(double(i), 0, 0));
    if (i == 0)
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(i, gtsam::Point3::Zero(), noise);
    else
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(i - 1, i, gtsam::Point3(1, 0, 0), noise);
    smoother.update(factors, initial, {{i, double(i)}});
    smoother.calculateEstimate();
  }
  gtsam::Values displaced = smoother.getISAM2().getLinearizationPoint();
  displaced.update(1, gtsam::Point3(1.2, 0, 0));
  int marginal_count = 0;
  for (const auto& factor : smoother.getFactors()) {
    auto marginal = dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get());
    if (!marginal) continue;
    ++marginal_count;
    EXPECT_TRUE(marginal->hasLinearizationPoint());
    // P1 prior variance is .01 + .01 after eliminating P0; displacement .2
    // must cost .5 * .2^2 / .02 = 1 in the nonlinear Dogleg acceptance test.
    EXPECT_NEAR(marginal->error(displaced), 1.0, 1e-10);
  }
  EXPECT_EQ(marginal_count, 1);
}

#include <gtsam/nonlinear/Marginals.h>
TEST(Smoother, RepeatedBranchedMarginalizationMatchesFullHistoryInformation) {
  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::X;
  gtsam::ISAM2Params params;
  params.setOptimizationParams(gtsam::ISAM2DoglegParams());
  params.relinearizeSkip = 1;
  liuo_tunnel::solver::IncrementalFixedLagSmootherExt smoother(2.5, params);
  gtsam::NonlinearFactorGraph history;
  gtsam::Values all;
  const auto motion = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(.1, .2, .3));
  const auto observation = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(.3, .1, .2));
  const auto prior = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  const gtsam::Point3 anchors[] = {{0, 2, 3}, {10, -2, 4}};
  for (size_t i = 0; i < 40; ++i) {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values initial;
    const gtsam::Point3 position(double(i), .2 * i, -.1 * i);
    initial.insert(X(i), position);
    if (i == 0) {
      for (size_t j = 0; j < 2; ++j) {
        initial.insert(A(j), anchors[j]);
        factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(A(j), anchors[j], prior);
      }
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(X(i), position, prior);
    } else {
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(X(i - 1), X(i), gtsam::Point3(1, .2, -.1), motion);
    }
    if (i >= 2) factors.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(X(i - 2), X(i), gtsam::Point3(2, .4, -.2), observation);
    const size_t j = i % 2;
    factors.emplace_shared<gtsam::BetweenFactor<gtsam::Point3>>(X(i), A(j), anchors[j] - position, observation);
    history.add(factors);
    all.insert(initial);
    smoother.update(factors, initial, {{X(i), double(i)}, {A(0), double(i)}, {A(1), double(i)}});
    const gtsam::Marginals batch(history, all);
    for (const auto key : {A(0), A(1), X(i)}) {
      EXPECT_LT((smoother.marginalCovariance(key) - batch.marginalCovariance(key)).norm(), 1e-8) << "step=" << i;
    }
    const gtsam::KeyVector keys{A(0), A(1), X(i)};
    EXPECT_LT((selected_covariance(smoother.getISAM2(), keys) - batch.jointMarginalCovariance(keys).fullMatrix()).norm(), 1e-8) << "step=" << i;
    if (i >= 3) EXPECT_FALSE(smoother.getISAM2().getLinearizationPoint().exists(X(i - 3)));
  }
}

TEST(UwbQueue, StaticAlignmentIsBoundedAndNeverExtrapolatesLatePose) {
  const std::map<double, gtsam::Key> first{{3., 42}};
  for (double stamp : {1., 2., 2.999}) {
    const auto bracket = static_alignment_bracket(first, stamp, 1., 3.);
    EXPECT_EQ(bracket.reason, "STATIC_ALIGNMENT");
    EXPECT_EQ(bracket.left, 42);
    EXPECT_FALSE(bracket.right);
  }
  EXPECT_EQ(static_alignment_bracket(first, .999, 1., 3.).reason, "WINDOW_EXPIRED");
  EXPECT_EQ(static_alignment_bracket(first, 3., 1., 3.).reason, "SYNC");
  EXPECT_EQ(static_alignment_bracket(first, 3.001, 1., 3.).reason, "WAIT_RIGHT");
  EXPECT_EQ(static_alignment_bracket({{3.1, 42}}, 2., 1., 3.).reason, "WINDOW_EXPIRED");
  EXPECT_EQ(static_alignment_bracket({{3., 42}, {3.1, 43}}, 2., 1., 3.).reason, "WINDOW_EXPIRED");
}

TEST(UwbFactors, OptimisticGeometryUsesCommonWorldTranslationNotTwoIndependentPoses) {
  using namespace gtsam::symbol_shorthand;
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(.1, -.2, .4), {3, 4, 5}));
  values.insert(X(1), gtsam::Pose3(gtsam::Rot3::RzRyRx(-.2, .1, 1.2), {4, 4.5, 5.2}));
  values.insert(A(0), gtsam::Point3(100, 3, 7));
  TagAnchorRangeFactor factor(X(0), X(1), A(0), {}, {.1, .15, .25}, .4, 100, .1);
  Eigen::RowVector3d J;
  for (int axis = 0; axis < 3; ++axis) {
    auto plus = values, minus = values;
    for (auto key : {X(0), X(1)}) {
      auto pose = values.at<gtsam::Pose3>(key);
      gtsam::Point3 step = gtsam::Point3::Zero();
      step[axis] = 1e-5;
      plus.update(key, gtsam::Pose3(pose.rotation(), pose.translation() + step));
      minus.update(key, gtsam::Pose3(pose.rotation(), pose.translation() - step));
    }
    J[axis] = (factor.unwhitenedError(plus)[0] - factor.unwhitenedError(minus)[0]) / 2e-5 / .1;
  }
  EXPECT_LT((factor.optimistic_translation_information(values) - J.transpose() * J).norm(), 1e-6);
}

#include <liuo_tunnel/common/sensors.hpp>
#include <filesystem>
#include <unistd.h>
TEST(UwbConfiguration, AaDefaultsOffAndRequiresExplicitBooleanOptIn) {
  const auto source = std::filesystem::path(__FILE__).parent_path().parent_path() / "config/phase3_self_calibrate.json";
  std::ifstream input(source);
  auto j = nlohmann::json::parse(input);
  const auto path = std::filesystem::temp_directory_path() /
    ("aa_config_test_" + std::to_string(getpid()) + ".json");
  auto write = [&]() { std::ofstream output(path); output << j.dump(); };
  j["uwb"].erase("use_aa"); write();
  EXPECT_FALSE(load_config(path.string())["uwb"]["use_aa"].get<bool>());
  j["uwb"]["use_aa"] = true; write();
  EXPECT_TRUE(load_config(path.string())["uwb"]["use_aa"].get<bool>());
  j["uwb"]["use_aa"] = false; write();
  EXPECT_FALSE(load_config(path.string())["uwb"]["use_aa"].get<bool>());
  j["uwb"]["use_aa"] = "false"; write();
  EXPECT_THROW(load_config(path.string()), std::runtime_error);
  std::filesystem::remove(path);
}
