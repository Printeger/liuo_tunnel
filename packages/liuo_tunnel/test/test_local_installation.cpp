#include <gtest/gtest.h>
#include <liuo_tunnel/uwb/local_installation.hpp>

using namespace liuo_tunnel;
namespace {
std::vector<gtsam::Point3> walls(const gtsam::Pose3& body, double radius = 0) {
  std::vector<gtsam::Point3> points;
  for (double x = -20; x <= 20; x += .5)
    for (double z = 0; z <= 4.8; z += .2)
      for (double side : {-1., 1.}) points.push_back(body.transformTo(gtsam::Point3(x, side * 5.6 + (radius ? x * x / (2 * radius) : 0), z)));
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
