#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/initialization_candidates.hpp>
using namespace liuo_tunnel;
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
