#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/local_installation.hpp>

using namespace liuo_tunnel;
namespace {
std::vector<gtsam::Point3> walls(const gtsam::Pose3& body, double radius = 0) {
  std::vector<gtsam::Point3> points;
  for (double x = -20; x <= 20; x += .5)
    for (double z = 0; z <= 4.8; z += .2)
      for (double side : {-1., 1.}) points.push_back(body.transformTo(gtsam::Point3(x, side * 5.6 + (radius ? x * x / (2 * radius) : 0), z)));
  for (double x = -20; x <= 20; x += .5)
    for (double side : {-1., 1.})
      for (double offset = .1; offset <= .8; offset += .1) points.push_back(body.transformTo(gtsam::Point3(x, side * (5.6 - offset) + (radius ? x * x / (2 * radius) : 0), 0)));
  return points;
}
}  // namespace
TEST(LocalInstallation, ObservedBodyWallsAreGaugeIndependentAndFiniteInSpan) {
  const gtsam::Pose3 body(gtsam::Rot3::RzRyRx(.02, -.005, .4), {0, -1.875, 1.66});
  auto measurement = local_installation_observation(walls(body), body.rotation(), body.rotation().unrotate(gtsam::Point3(10, 0, 0)), .02);
  EXPECT_NEAR(measurement.normal.dot(body.rotation().unrotate(gtsam::Point3(0, 1, 0))), 1, 1e-8);
  EXPECT_NEAR(measurement.center_offset, 1.875, 1e-8);
  EXPECT_NEAR(measurement.width, 11.2, 1e-8);
  EXPECT_TRUE(measurement.supports(body.transformTo(gtsam::Point3(8, 5.52, 3.5))));
  EXPECT_FALSE(measurement.supports(body.transformTo(gtsam::Point3(11, 5.52, 3.5))));
  EXPECT_FALSE(measurement.supports(body.transformTo(gtsam::Point3(0, 5.52, -.5))));
  EXPECT_FALSE(measurement.supports(body.transformTo(gtsam::Point3(0, 5.52, 6))));
}
TEST(LocalInstallation, MeasuredGentleCurvatureAcceptedWithoutRadiusInput) {
  const gtsam::Pose3 body(gtsam::Rot3(), {0, -1.875, 1.66});
  auto measurement = local_installation_observation(walls(body, 3000), body.rotation(), {10, 0, 0}, .02);
  EXPECT_GT(measurement.surface_model_error, .001);
  EXPECT_TRUE(measurement.supports(body.transformTo(gtsam::Point3(8, 5.52, 3.5))));
  EXPECT_THROW(local_installation_observation(walls(body, 50), body.rotation(), {10, 0, 0}, .02), std::invalid_argument);
}
TEST(LocalInstallation, FactorUsesJointPoseAndAnchorAndRightRetractJacobian) {
  const gtsam::Pose3 body(gtsam::Rot3::RzRyRx(.02, -.005, .4), {0, -1.875, 1.66});
  auto measurement = local_installation_observation(walls(body), body.rotation(), body.rotation().unrotate(gtsam::Point3(10, 0, 0)), .02);
  LocalInstallationFactor factor(0, 1, measurement, 5.52);
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.3, -.2, .5), {21, -4, 8});
  const auto anchor = pose.transformFrom(body.transformTo(gtsam::Point3(8, 5.52, 3.5)));
  gtsam::Matrix Hx, Ha;
  EXPECT_NEAR(factor.evaluateError(pose, anchor, &Hx, &Ha)[0], 0, 1e-8);
  for (int i = 0; i < 6; ++i) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta[i] = 1e-5;
    const double derivative = (factor.evaluateError(pose.retract(delta), anchor)[0] - factor.evaluateError(pose.retract(-delta), anchor)[0]) / 2e-5;
    EXPECT_NEAR(Hx(0, i), derivative, 1e-7);
  }
  for (int i = 0; i < 3; ++i) {
    gtsam::Point3 delta = gtsam::Point3::Zero();
    delta[i] = 1e-5;
    EXPECT_NEAR(Ha(0, i), (factor.evaluateError(pose, anchor + delta)[0] - factor.evaluateError(pose, anchor - delta)[0]) / 2e-5, 1e-7);
  }
  EXPECT_THROW(factor.evaluateError(pose, pose.transformFrom(body.transformTo(gtsam::Point3(30, 5.52, 3.5)))), std::domain_error);
}

#include <liuo_tunnel/uwb/local_offset_input.hpp>
TEST(LocalInstallation, ExactSyntheticMetadataRejectsCoordinatesAndFakeFieldUncertainty) {
  nlohmann::json input = {
    {"source", "exact synthetic relation; not field calibrated; no covariance or algorithm prior"},
    {"definition", "horizontal signed normal offset from design axis to antenna phase center, metres; valid ordinary wall band"},
    {"anchors", {{{"id", 16}, {"side", "left"}, {"lateral_offset_m", 5.52}}, {{"id", 3}, {"side", "right"}, {"lateral_offset_m", -5.52}}}}};
  auto parsed = SyntheticLocalOffsets::parse(input);
  EXPECT_EQ(parsed.offsets.size(), 2);
  EXPECT_DOUBLE_EQ(parsed.offsets.at(16), 5.52);
  for (const auto* field : {"height", "station", "position", "order", "covariance"}) {
    auto invalid = input;
    invalid["anchors"][0][field] = 1.;
    EXPECT_THROW(SyntheticLocalOffsets::parse(invalid), std::invalid_argument);
  }
  auto invalid = input;
  invalid["anchors"][0]["side"] = "right";
  EXPECT_THROW(SyntheticLocalOffsets::parse(invalid), std::invalid_argument);
  invalid = input;
  invalid["source"] = "field calibrated";
  EXPECT_THROW(SyntheticLocalOffsets::parse(invalid), std::invalid_argument);
}

TEST(LocalInstallation, FloorAssociationUsesMeasuredSurfaceAndRejectsSubfloorBranch) {
  const gtsam::Pose3 body(gtsam::Rot3(), {0, -1.875, 1.66});
  auto measurement = local_installation_observation(walls(body), body.rotation(), {10, 0, 0}, .02);
  EXPECT_FALSE(measurement.supports(body.transformTo(gtsam::Point3(0, 5.52, -.3))));
  EXPECT_TRUE(measurement.supports(body.transformTo(gtsam::Point3(0, 5.52, .3))));
  const auto& floor = measurement.floors[1];
  EXPECT_NEAR(floor.signed_height(body.transformTo(gtsam::Point3(0, 5.52, 0))), 0, 1e-8);
  EXPECT_GT(floor.margin(body.transformTo(gtsam::Point3(10, 5.52, 0))), floor.margin(floor.center));
}

#include <liuo_tunnel/uwb/anchor_relative_candidate.hpp>
TEST(AnchorRelativeCandidate, IndependentWorldGaugeChangesPreserveSameVehicleRelativeGeometry) {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.2, -.1, .5), {15, 2, 1});
  const gtsam::Point3 anchor(19, -3, 4);
  const auto before = AnchorRelativeCandidate::from_joint(42, pose, anchor);
  const gtsam::Pose3 gauge(gtsam::Rot3::RzRyRx(-.4, .2, 1.2), {100, -90, 50});
  const auto after = AnchorRelativeCandidate::from_joint(42, gauge.compose(pose), gauge.transformFrom(anchor));
  ASSERT_TRUE(before.displacement(after));
  EXPECT_NEAR(*before.displacement(after), 0, 1e-12);
  const auto moved = AnchorRelativeCandidate::from_joint(42, gauge.compose(pose), gauge.transformFrom(gtsam::Point3(anchor + pose.rotation().rotate(gtsam::Point3(.6, 0, 0)))));
  EXPECT_NEAR(*before.displacement(moved), .6, 1e-12);
  EXPECT_GT(*before.displacement(moved), .5);
}
TEST(AnchorRelativeCandidate, DifferentVehicleTimeCannotSupplyConsistencyEvidence) {
  const auto first = AnchorRelativeCandidate::from_joint(42, gtsam::Pose3(), {1, 2, 3});
  const auto other = AnchorRelativeCandidate::from_joint(43, gtsam::Pose3(), {1, 2, 3});
  EXPECT_FALSE(first.displacement(other));
}

TEST(AnchorRelativeCandidate, MainGraphInitialGuessUsesSameObservedVehicleReference) {
  const gtsam::Pose3 candidate_pose(gtsam::Rot3::RzRyRx(.02, -.1, .5), {490, 2, 1});
  const auto candidate_anchor = candidate_pose.transformFrom(gtsam::Point3(2, -3, 1.9));
  const auto local = AnchorRelativeCandidate::from_joint(490, candidate_pose, candidate_anchor);
  const gtsam::Pose3 main_pose(gtsam::Rot3::RzRyRx(.03, -.08, .6), {515, 3, 1.2});
  EXPECT_GT((main_pose.transformTo(candidate_anchor) - local.point).norm(), 20);
  const auto transferred = local.in_reference(490, main_pose);
  EXPECT_NEAR((main_pose.transformTo(transferred) - local.point).norm(), 0, 1e-12);
  EXPECT_THROW(local.in_reference(491, main_pose), std::invalid_argument);
}

TEST(LocalInstallation, GroundAssociationUsesActualSupportAndCannotExtendToWallSpan) {
  const gtsam::Pose3 body(gtsam::Rot3(), {0, -1.875, 1.66});
  auto points = walls(body);
  points.erase(
    std::remove_if(
      points.begin(),
      points.end(),
      [&](const auto& point) {
        const auto world = body.transformFrom(point);
        return std::abs(world.z()) < 1e-8 && std::abs(world.x()) > 6;
      }),
    points.end());
  const auto observed = local_installation_observation(points, body.rotation(), {10, 0, 0}, .02);
  EXPECT_TRUE(observed.supports(body.transformTo(gtsam::Point3(5, 5.52, 3.5))));
  EXPECT_FALSE(observed.supports(body.transformTo(gtsam::Point3(8, 5.52, 3.5))));
  EXPECT_FALSE(observed.supports(body.transformTo(gtsam::Point3(-8, 5.52, 3.5))));
  EXPECT_NEAR(observed.floors[1].minimum_longitudinal, -6, 1e-8);
  EXPECT_NEAR(observed.floors[1].maximum_longitudinal, 6, 1e-8);
}
