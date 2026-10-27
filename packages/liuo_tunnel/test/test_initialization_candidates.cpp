#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/initialization_candidates.hpp>
#include <liuo_tunnel/uwb/stepped_bootstrap_lm.hpp>
#include <gtsam/slam/PriorFactor.h>
using namespace liuo_tunnel;

TEST(BootstrapTermination, BudgetIsNotReportedAsConvergence) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(0, gtsam::Point3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1));
  gtsam::Values initial;
  initial.insert(0, gtsam::Point3(10, 2, 1));
  gtsam_points::LevenbergMarquardtExtParams params;
  params.maxIterations = 0;
  params.absoluteErrorTol = params.relativeErrorTol = params.errorTol = 0;
  SteppedBootstrapLM optimizer(graph, initial, params, true);
  for (int i = 0; i < 100 && !optimizer.finished(); ++i) optimizer.advance();
  ASSERT_TRUE(optimizer.finished());
  EXPECT_EQ(optimizer.termination_flags(), 1u);
  EXPECT_GT(optimizer.last_absolute_error_change(), 0);
}

TEST(BootstrapTermination, ErrorAndCustomStopCanCoexistWithoutBudget) {
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3>>(0, gtsam::Point3::Zero(), gtsam::noiseModel::Isotropic::Sigma(3, 1));
  gtsam::Values initial;
  initial.insert(0, gtsam::Point3(1, 2, 3));
  gtsam_points::LevenbergMarquardtExtParams params;
  params.maxIterations = 30;
  params.absoluteErrorTol = params.relativeErrorTol = 0;
  params.errorTol = 1;
  params.termination_criteria = [](const gtsam::Values&) { return true; };
  SteppedBootstrapLM optimizer(graph, initial, params, true);
  for (int i = 0; i < 100 && !optimizer.finished(); ++i) optimizer.advance();
  ASSERT_TRUE(optimizer.finished());
  EXPECT_EQ(optimizer.termination_flags(), 16u | 32u);
  EXPECT_LT(optimizer.error(), 1);
}
namespace {
std::array<RangeSeedSphere, 3> spheres(const gtsam::Point3& p) {
  std::array<RangeSeedSphere, 3> s{{{{0, 0, 0}, 0}, {{400, 3, 0}, 0}, {{800, -2, 4}, 0}}};
  for (auto& a : s) a.radius = (a.anchor - p).norm();
  return s;
}
}  // namespace
TEST(InitializationCandidates, KeepsBothMirrorsAndLeverForEveryYaw) {
  const gtsam::Point3 p(180, .8, 2.4);
  const auto input = spheres(p);
  const auto result = intersect_range_seeds(input);
  ASSERT_EQ(result.reason, "MIRROR_SEED_PAIR");
  ASSERT_EQ(result.tags.size(), 2);
  EXPECT_LT(std::min((result.tags[0] - p).norm(), (result.tags[1] - p).norm()), 1e-8);
  EXPECT_GT((result.tags[0] - result.tags[1]).norm(), 1);
  for (const auto& tag : result.tags)
    for (const auto& s : input) EXPECT_NEAR((tag - s.anchor).norm(), s.radius, 1e-9);
  const gtsam::Vector3 lever(.1, .15, .25);
  const auto gravity = gtsam::Rot3::RzRyRx(.08, -.12, 0);
  const auto poses = pose_seeds_from_tags(result.tags, gravity, lever);
  ASSERT_EQ(poses.size(), 16);
  for (size_t k = 0; k < poses.size(); ++k) {
    EXPECT_LT((poses[k].transformFrom(lever) - result.tags[k / 8]).norm(), 1e-10);
    EXPECT_LT((poses[k].rotation().unrotate(gtsam::Vector3::UnitZ()) - gravity.unrotate(gtsam::Vector3::UnitZ())).norm(), 1e-10);
  }
}
TEST(InitializationCandidates, DoesNotDependOnWorldOriginOrAnchorOrder) {
  auto input = spheres({180, .8, 2.4});
  const auto baseline = intersect_range_seeds(input);
  const gtsam::Pose3 transform(gtsam::Rot3::RzRyRx(.2, -.3, 1.1), {120, -60, 80});
  for (auto& s : input) s.anchor = transform.transformFrom(s.anchor);
  std::swap(input[0], input[2]);
  const auto changed = intersect_range_seeds(input);
  ASSERT_EQ(changed.tags.size(), 2);
  for (const auto& tag : baseline.tags) {
    const auto expected = transform.transformFrom(tag);
    EXPECT_LT(std::min((expected - changed.tags[0]).norm(), (expected - changed.tags[1]).norm()), 1e-8);
  }
}
TEST(InitializationCandidates, RejectsInconsistentAndDegenerateInput) {
  auto input = spheres({180, .8, 2.4});
  for (auto& s : input) s.radius = 1;
  EXPECT_EQ(intersect_range_seeds(input).reason, "SEED_SPHERES_DO_NOT_INTERSECT");
  input[2].anchor = 2 * input[1].anchor;
  EXPECT_EQ(intersect_range_seeds(input).reason, "DEGENERATE_SEED_ANCHORS");
  input[0].radius = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(intersect_range_seeds(input).reason, "INVALID_SEED_SPHERE");
  EXPECT_THROW(pose_seeds_from_tags({}, gtsam::Rot3(), gtsam::Vector3::Zero(), 1000), std::invalid_argument);
}
TEST(InitializationCandidates, TangentIsNotAnArtificialMirrorPair) {
  std::array<RangeSeedSphere, 3> input{{{{0, 0, 0}, 5}, {{6, 0, 0}, 5}, {{0, 8, 0}, 5}}};
  const auto result = intersect_range_seeds(input);
  ASSERT_EQ(result.tags.size(), 1);
  EXPECT_EQ(result.reason, "TANGENT_SEED_SPHERES");
  EXPECT_LT((result.tags[0] - gtsam::Point3(3, 4, 0)).norm(), 1e-10);
}
TEST(InitializationCandidates, TransformsRobotTrajectoryNotMapOrBodyBias) {
  using gtsam::symbol_shorthand::A;
  using gtsam::symbol_shorthand::V;
  using gtsam::symbol_shorthand::X;
  gtsam::Values input;
  gtsam::Pose3 a(gtsam::Rot3::Rz(.3), {10, -2, 4}), b(gtsam::Rot3::Rz(.4), {11, -1, 5});
  input.insert(X(0), a);
  input.insert(X(1), b);
  input.insert(V(0), gtsam::Vector3(1, 2, 3));
  input.insert(A(1), gtsam::Point3(400, 3, 5));
  input.insert(gtsam::Symbol('b', 0), gtsam::Vector6::Ones().eval());
  const gtsam::Pose3 seed(gtsam::Rot3::Rz(2), {80, 2, 8});
  auto changed = transform_bootstrap_seed(input, X(0), seed);
  EXPECT_LT(gtsam::Pose3::Logmap(seed.between(changed.at<gtsam::Pose3>(X(0)))).norm(), 1e-10);
  EXPECT_LT(gtsam::Pose3::Logmap(a.between(b).between(changed.at<gtsam::Pose3>(X(0)).between(changed.at<gtsam::Pose3>(X(1))))).norm(), 1e-10);
  EXPECT_LT((a.rotation().unrotate(input.at<gtsam::Vector3>(V(0))) - seed.rotation().unrotate(changed.at<gtsam::Vector3>(V(0)))).norm(), 1e-10);
  EXPECT_EQ(changed.at<gtsam::Point3>(A(1)), input.at<gtsam::Point3>(A(1)));
  EXPECT_EQ(changed.at<gtsam::Vector6>(gtsam::Symbol('b', 0)), input.at<gtsam::Vector6>(gtsam::Symbol('b', 0)));
}
TEST(InitializationCandidates, SmallCostOrOptimizerReturnDoesNotResolveAmbiguity) {
  std::vector<BootstrapBranchEvidence> branches{{.01, gtsam::Pose3()}, {.02, gtsam::Pose3(gtsam::Rot3(), {0, 0, 7})}};
  EXPECT_FALSE(separated_bootstrap_branch(branches, 0));
  branches[1].cost = 10;
  EXPECT_TRUE(separated_bootstrap_branch(branches, 0));
  branches[1].cost = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(separated_bootstrap_branch(branches, 0));
  EXPECT_FALSE(separated_bootstrap_branch({branches[0]}, 0));
}

#include <liuo_tunnel/uwb/bootstrap_preconditioner.hpp>
TEST(InitializationCandidates, RawInterpolatedGaugeJacobianMatchesFiniteDifference) {
  using namespace gtsam::symbol_shorthand;
  gtsam::Values values;
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(.1, -.2, .6), {180, 2, 4}));
  values.insert(X(1), gtsam::Pose3(gtsam::Rot3::RzRyRx(.2, -.1, .7), {181, 3, 4.2}));
  values.insert(A(2), gtsam::Point3(400, -3, 5));
  TagAnchorRangeFactor factor(X(0), X(1), A(2), gtsam::Point3::Zero(), {.1, .15, .25}, .3, 220, .1);
  const auto analytic = range_gauge_jacobian(factor, values);
  for (int axis = 0; axis < 4; ++axis) {
    Eigen::Vector4d step = Eigen::Vector4d::Zero();
    step[axis] = 1e-5;
    const double numeric = (factor.unwhitenedError(move_bootstrap_gauge(values, step))[0] - factor.unwhitenedError(move_bootstrap_gauge(values, -step))[0]) / 2e-5;
    EXPECT_NEAR(analytic[axis], numeric, 1e-6);
  }
}
TEST(InitializationCandidates, RangePreconditionerReducesCostWithoutChangingRelativeMotion) {
  using namespace gtsam::symbol_shorthand;
  gtsam::Values truth;
  gtsam::NonlinearFactorGraph graph;
  const gtsam::Point3 lever(.1, .15, .25);
  std::vector<gtsam::Point3> anchors{{0, 3, 4}, {400, -3, 4}, {800, -3, .5}, {1200, 3, .5}};
  for (size_t i = 0; i < 8; ++i) {
    gtsam::Pose3 pose(gtsam::Rot3::Rz(.3), {200 + 3. * i, .5, 2.});
    truth.insert(X(i), pose);
    for (const auto& a : anchors) graph.emplace_shared<TagAnchorRangeFactor>(X(i), std::nullopt, std::nullopt, a, lever, 0, (pose.transformFrom(lever) - a).norm(), .1);
  }
  Eigen::Vector4d offset;
  offset << .5, .2, -.1, .05;
  const auto initial = move_bootstrap_gauge(truth, offset);
  const auto result = precondition_bootstrap(graph, initial);
  EXPECT_LT(result.cost, graph.error(initial) * .001);
  EXPECT_LE(result.iterations, 30);
  const auto before = initial.at<gtsam::Pose3>(X(0)).between(initial.at<gtsam::Pose3>(X(7)));
  const auto after = result.values.at<gtsam::Pose3>(X(0)).between(result.values.at<gtsam::Pose3>(X(7)));
  EXPECT_LT(gtsam::Pose3::Logmap(before.between(after)).norm(), 1e-9);
}

#include <liuo_tunnel/uwb/lateral_initialization.hpp>
namespace {
KnownMap installation_fixture() {
  KnownMap map;
  map.mode = "surveyed_map";
  map.covariance = .0004 * gtsam::Matrix::Identity(21, 21);
  nlohmann::json records = nlohmann::json::array(), covariance = nlohmann::json::array();
  const auto rotation = gtsam::Rot3::Rz(.4);
  for (size_t i = 0; i < 7; ++i) {
    const double d = i % 2 ? -2.8 : 2.8;
    map.ids.push_back(100 + i * 3);
    map.means.push_back(rotation * gtsam::Point3(400. * i, d, double(i % 3)) + gtsam::Point3(20, 4, 7));
    records.push_back({{"anchor_id", map.ids.back()}, {"signed_offset_m", d}});
    nlohmann::json row = nlohmann::json::array();
    for (size_t k = 0; k < 7; ++k) row.push_back(.0004 + (i == k ? .0025 : 0));
    covariance.push_back(row);
  }
  map.installation = {
    {"schema_version", "liuo_lateral_installation_v1"},
    {"reference_model", "shared_straight_horizontal_axis"},
    {"reference_id", "unit_test"},
    {"units", "m"},
    {"provenance", "synthetic unit test"},
    {"positive_side_convention", "declared test direction"},
    {"independent_from_map_survey", true},
    {"records", records},
    {"joint_covariance_m2", covariance}};
  return map;
}
}  // namespace
TEST(InitializationCandidates, InstallationAxisUsesRecordsAndPreservesUncertainty) {
  auto map = installation_fixture();
  const auto reference = LateralReference::fit(map);
  const Eigen::Vector2d n(-std::sin(.4), std::cos(.4));
  EXPECT_LT((reference.normal - n).norm(), 1e-8);
  EXPECT_NEAR(reference.offset, n.dot(Eigen::Vector2d(20, 4)), 1e-8);
  EXPECT_GT(reference.covariance(1, 1), .0004);
  EXPECT_GT(reference.covariance.determinant(), 0);
  map.installation["records"][1]["anchor_id"] = map.ids[0];
  EXPECT_THROW(LateralReference::fit(map), std::invalid_argument);
}
TEST(InitializationCandidates, ActualLocalWallGeometrySupportsTwoDirectionsNotHeight) {
  const auto reference = LateralReference::fit(installation_fixture());
  for (double yaw : {0., 1.57079632679, 3.14159265359}) {
    std::vector<Eigen::Vector3d> points;
    const auto body = gtsam::Rot3::Rz(yaw);
    for (double side : {-3., 3.})
      for (int x = -30; x <= 30; ++x)
        for (int z = 0; z < 8; ++z) points.push_back(body.unrotate(gtsam::Vector3(x, side - .8, .5 * z - 2.)));
    const auto walls = extract_local_walls(points);
    EXPECT_NEAR(walls.width, 6., 1e-8);
    EXPECT_NEAR(std::abs(walls.center_offset), .8, 1e-8);
    const std::vector<gtsam::Point3> tags{{180, 2, 1}, {180, 2, 8}};
    const gtsam::Vector3 lever(.1, .15, .25);
    const auto seeds = lateral_pose_seeds(tags, gtsam::Rot3(), lever, walls, reference);
    ASSERT_EQ(seeds.size(), 4);
    EXPECT_NEAR(seeds[1].z() - seeds[0].z(), 7., 1e-8);
    for (size_t i = 0; i < seeds.size(); ++i) {
      const double sign = i < 2 ? 1 : -1;
      EXPECT_NEAR(reference.normal.dot(seeds[i].translation().head<2>()) + sign * walls.center_offset, reference.offset, 1e-8);
    }
  }
}

#include <liuo_tunnel/uwb/installation_factors.hpp>
namespace {
void check_installation_derivatives(const gtsam::NoiseModelFactor& factor, const gtsam::Values& values) {
  std::vector<gtsam::Matrix> analytic;
  factor.unwhitenedError(values, &analytic);
  for (size_t i = 0; i < factor.keys().size(); ++i) {
    auto key = factor.keys()[i];
    char c = gtsam::Symbol(key).chr();
    int dimension = c == 'x' ? 6 : (c == 'l' ? 2 : 3);
    for (int axis = 0; axis < dimension; ++axis) {
      auto plus = values, minus = values;
      constexpr double eps = 1e-5;
      if (c == 'x') {
        gtsam::Vector6 d = gtsam::Vector6::Zero();
        d[axis] = eps;
        plus.update(key, values.at<gtsam::Pose3>(key).retract(d));
        minus.update(key, values.at<gtsam::Pose3>(key).retract(-d));
      } else if (c == 'l') {
        gtsam::Vector2 d = gtsam::Vector2::Zero();
        d[axis] = eps;
        plus.update(key, gtsam::Vector2(values.at<gtsam::Vector2>(key) + d));
        minus.update(key, gtsam::Vector2(values.at<gtsam::Vector2>(key) - d));
      } else {
        gtsam::Vector3 d = gtsam::Vector3::Zero();
        d[axis] = eps;
        plus.update(key, gtsam::Vector3(values.at<gtsam::Vector3>(key) + d));
        minus.update(key, gtsam::Vector3(values.at<gtsam::Vector3>(key) - d));
      }
      const auto numeric = ((factor.unwhitenedError(plus) - factor.unwhitenedError(minus)) / (2 * eps)).eval();
      EXPECT_LT((numeric - analytic[i].col(axis)).lpNorm<Eigen::Infinity>(), 1e-6);
    }
  }
}
}  // namespace
TEST(InitializationCandidates, CorrelatedInstallationAndWallFactorJacobians) {
  using namespace gtsam::symbol_shorthand;
  const auto L = gtsam::Symbol('l', 0);
  gtsam::Values v;
  v.insert(L, gtsam::Vector2(1.2, .7));
  v.insert(X(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(.1, -.2, .4), {12, 3, 8}));
  v.insert(A(1), gtsam::Point3(20, 3, 4));
  v.insert(A(2), gtsam::Point3(400, -3, 5));
  gtsam::Matrix2 C;
  C << .003, .0004, .0004, .003;
  InstallationOffsetFactor installation(L, {A(1), A(2)}, gtsam::Vector2(2.8, -2.8), C);
  WallReferenceFactor wall(X(0), L, {.2, .9, .1}, {.1, -.7, .2}, C);
  check_installation_derivatives(installation, v);
  check_installation_derivatives(wall, v);
  EXPECT_GT(dynamic_cast<const gtsam::noiseModel::Gaussian&>(*installation.noiseModel()).covariance()(0, 1), 0);
  const auto before = wall.unwhitenedError(v);
  auto moved = v;
  auto p = v.at<gtsam::Pose3>(X(0));
  moved.update(X(0), gtsam::Pose3(p.rotation(), p.translation() + gtsam::Vector3(0, 0, 15)));
  EXPECT_LT((wall.unwhitenedError(moved) - before).norm(), 1e-10);
  moved.update(X(0), gtsam::Pose3(p.rotation(), p.translation() + 10 * gtsam::Vector3(-std::sin(1.2), std::cos(1.2), 0)));
  EXPECT_LT((wall.unwhitenedError(moved) - before).norm(), 1e-10);
}
TEST(InitializationCandidates, WallObservationDoesNotChooseTunnelDirection) {
  using namespace gtsam::symbol_shorthand;
  const auto L = gtsam::Symbol('l', 0);
  gtsam::Values v;
  v.insert(L, gtsam::Vector2(std::acos(-1.) / 2, 0));
  v.insert(X(0), gtsam::Pose3(gtsam::Rot3(), {50, .8, 2}));
  WallReferenceFactor f(X(0), L, {0, 1, 0}, {0, -.8, 0}, gtsam::Matrix2::Identity());
  EXPECT_LT(f.unwhitenedError(v).norm(), 1e-10);
  v.update(X(0), gtsam::Pose3(gtsam::Rot3::Rz(std::acos(-1.)), {50, -.8, 9}));
  EXPECT_LT(f.unwhitenedError(v).norm(), 1e-10);
}

#include <liuo_tunnel/uwb/body_wall_observation.hpp>
TEST(InitializationCandidates, BodyWallNormalsComeFromPointsNotRepeatedGravityMeasurement) {
  const auto R = gtsam::Rot3::RzRyRx(.15, -.1, .8);
  std::vector<gtsam::Vector3> points;
  for (double side : {-3., 3.})
    for (int x = -20; x <= 20; ++x)
      for (int z = 0; z < 15; ++z) points.push_back(R.unrotate(gtsam::Vector3(x, side - .7, .25 * z - 1.8)));
  const auto a = body_wall_observation(points, R, .02);
  const auto b = body_wall_observation(points, gtsam::Rot3::Rx(.005) * R, .02);
  EXPECT_GT(std::abs(a.normal.dot(R.unrotate(gtsam::Vector3::UnitY()))), 1 - 1e-10);
  EXPECT_GT(std::abs(a.normal.dot(b.normal)), 1 - 1e-10);
  EXPECT_GT(a.covariance.determinant(), 0);
  EXPECT_GT(a.covariance(1, 1), 0);
}
TEST(InitializationCandidates, InstallationGeometryResidualsAreMapFrameEquivariant) {
  using namespace gtsam::symbol_shorthand;
  const auto L = gtsam::Symbol('l', 0);
  gtsam::Values v;
  v.insert(L, gtsam::Vector2(1.2, .7));
  v.insert(A(1), gtsam::Point3(20, 3, 4));
  v.insert(A(2), gtsam::Point3(400, -3, 5));
  v.insert(X(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(.1, -.2, .4), {12, 3, 8}));
  InstallationOffsetFactor a(L, {A(1), A(2)}, gtsam::Vector2(2.8, -2.8), gtsam::Matrix2::Identity());
  WallReferenceFactor w(X(0), L, {.2, .9, .1}, {.1, -.7, .2}, gtsam::Matrix2::Identity());
  auto shifted = v;
  const gtsam::Pose3 T(gtsam::Rot3::Rz(.6), {40, -20, 15});
  shifted.update(X(0), T.compose(v.at<gtsam::Pose3>(X(0))));
  for (auto key : {A(1), A(2)}) shifted.update(key, T.transformFrom(v.at<gtsam::Point3>(key)));
  shifted.update(L, gtsam::Vector2(1.8, .7 + gtsam::Vector3(std::cos(1.8), std::sin(1.8), 0).dot(T.translation())));
  EXPECT_LT((a.unwhitenedError(v) - a.unwhitenedError(shifted)).norm(), 1e-10);
  EXPECT_LT((w.unwhitenedError(v) - w.unwhitenedError(shifted)).norm(), 1e-10);
}

#include <gtsam/nonlinear/Marginals.h>
TEST(InitializationCandidates, JointInstallationPreservesMapAndInstallationCommonErrors) {
  using namespace gtsam::symbol_shorthand;
  const auto L = gtsam::Symbol('l', 0);
  auto map = installation_fixture();
  // Add a common horizontal datum error to all surveyed anchors.
  for (size_t i = 0; i < map.ids.size(); ++i)
    for (size_t j = 0; j < map.ids.size(); ++j) map.covariance.block<2, 2>(3 * i, 3 * j) += .01 * gtsam::Matrix2::Identity();
  const auto reference = LateralReference::fit(map);
  gtsam::Values values;
  gtsam::KeyVector keys;
  gtsam::Vector offsets(map.ids.size());
  gtsam::Matrix C(map.ids.size(), map.ids.size());
  for (size_t i = 0; i < map.ids.size(); ++i) {
    keys.push_back(A(map.ids[i]));
    values.insert(keys.back(), map.means[i]);
    offsets[i] = map.installation["records"][i]["signed_offset_m"];
    for (size_t j = 0; j < map.ids.size(); ++j) C(i, j) = map.installation["joint_covariance_m2"][i][j].get<double>();
  }
  values.insert(L, gtsam::Vector2(std::atan2(reference.normal.y(), reference.normal.x()), reference.offset));
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<SurveyedMapPrior>(keys, map.means, map.covariance);
  graph.emplace_shared<InstallationOffsetFactor>(L, keys, offsets, C);
  const auto covariance = gtsam::Marginals(graph, values, gtsam::Marginals::QR).marginalCovariance(L);
  EXPECT_LT((covariance - reference.covariance).norm(), 1e-7);
  EXPECT_GT(covariance(1, 1), .01 + .0004);
}

TEST(InitializationCandidates, RecordedPositionMirrorCannotBeCertifiedByTotalCost) {
  // N7 case 3, stamp 19: the wrong high branch wins the old cost/stability gate.
  std::vector<BootstrapBranchEvidence> branches{
    {207978.2359710491, gtsam::Pose3(gtsam::Rot3::Rz(3.141528487095105), {1799.984054572254, -.7459004145439476, 1.825159921245268})},
    {207969.2114728361, gtsam::Pose3(gtsam::Rot3::Rz(3.141554183688508), {1799.981244079938, -.7460257413391663, 6.708942127181323})}};
  EXPECT_TRUE(separated_bootstrap_branch(branches, 1));
  EXPECT_TRUE(unresolved_position_branch(branches, 1));
  branches[0].first_pose = branches[1].first_pose;
  EXPECT_FALSE(unresolved_position_branch(branches, 1));
  branches[0].first_pose = gtsam::Pose3(gtsam::Rot3(), {1799.984, -.746, 1.825});
  EXPECT_FALSE(unresolved_position_branch(branches, 1));  // opposite yaw uses the existing graph gate
}

#include <gtsam/base/cholesky.h>
TEST(InitializationCandidates, CenteredLinePreservesPhysicalLikelihoodAndJacobians) {
  using namespace gtsam::symbol_shorthand;
  const auto L = gtsam::Symbol('l', 0);
  const gtsam::Point3 origin(1200, -.4, 2);
  const double angle = 1.2;
  const gtsam::Vector3 normal(std::cos(angle), std::sin(angle), 0);
  gtsam::Values original;
  original.insert(L, gtsam::Vector2(angle, .7));
  original.insert(X(0), gtsam::Pose3(gtsam::Rot3::RzRyRx(.1, -.2, .4), {1800, 3, 8}));
  original.insert(A(1), gtsam::Point3(20, 3, 4));
  original.insert(A(2), gtsam::Point3(2400, -3, 5));
  auto centered = original;
  centered.update(L, gtsam::Vector2(angle, .7 - normal.dot(origin)));
  gtsam::Matrix2 C;
  C << .003, .0004, .0004, .003;
  InstallationOffsetFactor a(L, {A(1), A(2)}, gtsam::Vector2(2.8, -2.8), C);
  InstallationOffsetFactor b(L, {A(1), A(2)}, gtsam::Vector2(2.8, -2.8), C, origin);
  WallReferenceFactor c(X(0), L, {.2, .9, .1}, {.1, -.7, .2}, C);
  WallReferenceFactor d(X(0), L, {.2, .9, .1}, {.1, -.7, .2}, C, origin);
  EXPECT_LT((a.unwhitenedError(original) - b.unwhitenedError(centered)).norm(), 1e-10);
  EXPECT_LT((c.unwhitenedError(original) - d.unwhitenedError(centered)).norm(), 1e-10);
  check_installation_derivatives(b, centered);
  check_installation_derivatives(d, centered);
  auto scaled = centered;
  scaled.update(L, gtsam::Vector2(1000 * angle, .7 - normal.dot(origin)));
  InstallationOffsetFactor e(L, {A(1), A(2)}, gtsam::Vector2(2.8, -2.8), C, origin, 1000);
  WallReferenceFactor f(X(0), L, {.2, .9, .1}, {.1, -.7, .2}, C, origin, 1000);
  EXPECT_LT((b.unwhitenedError(centered) - e.unwhitenedError(scaled)).norm(), 1e-10);
  EXPECT_LT((d.unwhitenedError(centered) - f.unwhitenedError(scaled)).norm(), 1e-10);
  check_installation_derivatives(e, scaled);
  check_installation_derivatives(f, scaled);
}
TEST(InitializationCandidates, RecordedPositiveDefiniteLineBlockNeedsCoordinateConditioning) {
  // Rounded observed N10 frontal block. SPD, but GTSAM's pivot exponent check
  // rejects the radian/global-offset parameter scales. No information is added.
  gtsam::Matrix H(3, 3);
  H.setZero();
  H.topLeftCorner<2, 2>() << 1.98901835e10, 1.05838231e7, 1.05838231e7, 6320.74879;
  H(2, 2) = 3.1574;
  auto raw = H;
  EXPECT_FALSE(gtsam::choleskyPartial(raw, 2));
  gtsam::Matrix P = gtsam::Matrix::Identity(3, 3);
  P(1, 0) = -1200;
  gtsam::Matrix centered = P.transpose() * H * P;
  auto factored = centered;
  EXPECT_TRUE(gtsam::choleskyPartial(factored, 2));
  const gtsam::Matrix2 J = P.topLeftCorner<2, 2>();
  EXPECT_LT((J * centered.topLeftCorner<2, 2>().inverse() * J.transpose() - H.topLeftCorner<2, 2>().inverse()).norm(), 1e-10);
}

#include <liuo_tunnel/uwb/stepped_bootstrap_lm.hpp>
TEST(InitializationCandidates, SteppedOptimizerMatchesWholeSolveAndTermination) {
  using namespace gtsam::symbol_shorthand;
  gtsam::Values initial;
  initial.insert(X(0), gtsam::Pose3(gtsam::Rot3::Rz(.4), {250, .3, 2.}));
  gtsam::NonlinearFactorGraph graph;
  const gtsam::Point3 truth(200, .8, 1.6), lever(.1, .15, .25);
  for (const gtsam::Point3 anchor : std::vector<gtsam::Point3>{{0, 3, 4}, {400, -3, 4}, {800, 3, .4}, {1200, -3, .4}})
    graph.emplace_shared<TagAnchorRangeFactor>(X(0), std::nullopt, std::nullopt, anchor, lever, 0, (truth + gtsam::Rot3::Rz(.4) * lever - anchor).norm(), .1);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), gtsam::Pose3(gtsam::Rot3::Rz(.4), truth), gtsam::noiseModel::Isotropic::Sigma(6, 10));
  for (bool custom_stop : {false, true}) {
    gtsam_points::LevenbergMarquardtExtParams params;
    params.maxIterations = 30;
    if (custom_stop) params.termination_criteria = [](const gtsam::Values& v) { return v.at<gtsam::Pose3>(X(0)).x() < 220; };
    gtsam_points::LevenbergMarquardtOptimizerExt original(graph, initial, params);
    const auto expected = original.optimize();
    SteppedBootstrapLM stepped(graph, initial, params);
    size_t calls = 0;
    while (!stepped.advance()) ASSERT_LT(++calls, 100);
    EXPECT_EQ(stepped.iterations(), original.iterations());
    EXPECT_NEAR(stepped.error(), original.error(), 1e-12);
    EXPECT_LT(gtsam::Pose3::Logmap(expected.at<gtsam::Pose3>(X(0)).between(stepped.values().at<gtsam::Pose3>(X(0)))).norm(), 1e-12);
    EXPECT_TRUE(stepped.advance());
    EXPECT_EQ(stepped.iterations(), original.iterations());
    SteppedBootstrapLM trial(graph, initial, params, true);
    bool done = false;
    size_t trial_calls = 0;
    while (!done) {
      const auto before = trial.getInnerIterations();
      done = trial.advance();
      EXPECT_LE(trial.getInnerIterations() - before, 1);
      ASSERT_LT(++trial_calls, 2000);
    }
    EXPECT_EQ(trial.iterations(), original.iterations());
    EXPECT_EQ(trial.getInnerIterations(), original.getInnerIterations());
    EXPECT_NEAR(trial.error(), original.error(), 1e-12);
    EXPECT_LT(gtsam::Pose3::Logmap(expected.at<gtsam::Pose3>(X(0)).between(trial.values().at<gtsam::Pose3>(X(0)))).norm(), 1e-12);
  }
}

TEST(InitializationCandidates, ContinuousWallPointOwnershipIsDisjointAndBounded) {
  size_t dedicated = 0, vgicp = 0;
  for (size_t i = 0; i < 20339; ++i) {
    EXPECT_FALSE(dedicated_wall_point(false, i));
    if (dedicated_wall_point(true, i))
      ++dedicated;
    else
      ++vgicp;
  }
  EXPECT_EQ(dedicated, 5085);
  EXPECT_EQ(dedicated + vgicp, 20339);
}

TEST(InitializationCandidates, MilliradianLineCoordinatesAvoidObservedPivotScaleRejection) {
  gtsam::Matrix H = gtsam::Matrix::Identity(3, 3);
  H(0, 0) = std::pow(169445.71944338435, 2);
  H(1, 1) = std::pow(51.47770156549991, 2);
  auto raw = H;
  EXPECT_FALSE(gtsam::choleskyPartial(raw, 2));
  gtsam::Matrix P = gtsam::Matrix::Identity(3, 3);
  P(0, 0) = .001;
  gtsam::Matrix scaled = P.transpose() * H * P;
  auto factored = scaled;
  EXPECT_TRUE(gtsam::choleskyPartial(factored, 2));
  EXPECT_LT((P * scaled.inverse() * P.transpose() - H.inverse()).norm(), 1e-12);
}

// Actual independent reuse input, map export 20261022T140915Z_798,
// stationary range means from 20261022T141224Z_012 (39 samples per anchor).
TEST(InitializationCandidates, UncertainMapNonintersectionRetainsFiniteSearch) {
  std::array<RangeSeedSphere, 3> input{{
    {{-200.03045512147105, 2.4187071017307664, 1.901784427815304}, 219.85162991174903},
    {{200.00553151594167, -2.5598898641866845, 3.048282937511212}, 180.19665722327616},
    {{600.000231078469, -1.8360578816784716, -1.6466783019381377}, 580.1463911391492}}};
  EXPECT_EQ(intersect_range_seeds(input).reason, "SEED_SPHERES_DO_NOT_INTERSECT");
  const auto proposals = intersect_range_seeds(input, true);
  ASSERT_EQ(proposals.tags.size(), 2);
  EXPECT_EQ(proposals.reason, "NONINTERSECTING_MIRROR_PROPOSALS_NOT_SOLUTIONS");
  for (const auto& tag : proposals.tags) {
    EXPECT_TRUE(tag.allFinite());
    // These are deliberately NOT exact intersections or pose observations.
    EXPECT_GT(std::abs((tag-input[0].anchor).norm()-input[0].radius), .01);
  }
  EXPECT_GT((proposals.tags[0]-proposals.tags[1]).norm(), 1.);
}

namespace {
// The second residual has zero Jacobian: it changes neither the optimum nor
// information, like accumulated scatter in a marginal/AA factor.
class ConstantOffsetNonlinearCost : public gtsam::NoiseModelFactor1<double> {
public:
  explicit ConstantOffsetNonlinearCost(double offset)
      : NoiseModelFactor1(gtsam::noiseModel::Isotropic::Sigma(2, 1), 0), offset_(offset) {}
  gtsam::Vector evaluateError(const double& x, gtsam::OptionalMatrixType H = nullptr) const override {
    if (H) { *H = gtsam::Matrix::Zero(2,1); (*H)(0,0)=2*x; }
    return gtsam::Vector2(x*x-1,std::sqrt(2*offset_));
  }
private:
  double offset_;
};
}
TEST(InitializationCandidates, ExtensionSolveMustNotStopOnConstantBackgroundCost) {
  auto solve=[](double constant, double relative) {
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<ConstantOffsetNonlinearCost>(constant);
    gtsam::Values initial; initial.insert<double>(0,10.);
    gtsam_points::LevenbergMarquardtExtParams params;
    params.maxIterations=30; params.relativeErrorTol=relative;
    SteppedBootstrapLM optimizer(graph,initial,params,true);
    size_t calls=0;
    while (!optimizer.advance()) { if (++calls>1000) throw std::runtime_error("bounded solve exceeded"); }
    return optimizer.values().at<double>(0);
  };
  EXPECT_GT(std::abs(solve(900000.,1e-5)-1),.01);
  EXPECT_NEAR(solve(0,0),1,1e-5);
  EXPECT_NEAR(solve(900000.,0),1,1e-5);
}
