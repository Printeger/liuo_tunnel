#include <gtest/gtest.h>
#include <liuo_tunnel/common/rolling_motion.hpp>

using namespace liuo_tunnel;

TEST(RollingMotion, HighwaySpeedAndYawRecoverOneStaticWorldPointWithLeverArm) {
  IMUIntegration imu;
  for (int i = 0; i <= 40; ++i) imu.insert_imu(1 + i * .005, {0, 0, 9.80665}, {0, 0, .2});
  gtsam::NavState initial(gtsam::Pose3(), gtsam::Vector3(80. / 3.6, 0, 0));
  RollingMotion motion(imu, initial, {}, 1, 1.1, 1.2);
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  extrinsic.translation() = Eigen::Vector3d(.5, -.2, .1);
  extrinsic.linear() = Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d world(20, 4, 2);
  auto pose = [](double t) {
    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation().x() = 80. / 3.6 * t;
    p.linear() = Eigen::AngleAxisd(.2 * t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return p;
  };
  const Eigen::Vector3d expected = pose(.1).inverse() * world;
  for (double offset : {0., .00123, .0493, .09999}) {
    const Eigen::Vector3d raw = (pose(.1 + offset) * extrinsic).inverse() * world;
    EXPECT_LT((motion.point(raw, offset, extrinsic) - expected).norm(), 1e-9);
  }
  EXPECT_THROW(motion.point(world, 100., extrinsic), std::invalid_argument);
}

TEST(RollingMotion, RejectsMissingCoverageAndUnorderedImu) {
  IMUIntegration imu;
  imu.insert_imu(1, {0, 0, 9.80665}, {0, 0, 0});
  imu.insert_imu(1.005, {0, 0, 9.80665}, {0, 0, 0});
  EXPECT_THROW((RollingMotion(imu, {}, {}, 1, 1, 1.1)), std::invalid_argument);
  imu.insert_imu(1.1, {0, 0, 9.80665}, {0, 0, 0});
  EXPECT_THROW((RollingMotion(imu, {}, {}, 1, 1, 1.1)), std::invalid_argument);
}

TEST(RollingMotion, AccelerationUsesMeasuredSpecificForceWithBoundedInterpolationError) {
  IMUIntegration imu;
  for (int i = 0; i <= 20; ++i) imu.insert_imu(1 + i * .005, {1, 0, 9.80665}, {0, 0, 0});
  RollingMotion motion(imu, gtsam::NavState(gtsam::Pose3(), {20, 0, 0}), {}, 1, 1, 1.1);
  const Eigen::Vector3d world(30, 2, 3);
  for (double t : {.0025, .0275, .0725, .099}) {
    const Eigen::Vector3d raw = world - Eigen::Vector3d(20 * t + .5 * t * t, 0, 0);
    EXPECT_LT((motion.point(raw, t, Eigen::Isometry3d::Identity()) - world).norm(), 4e-6);
  }
}

TEST(RollingMotion, RejectsDuplicateAndReversedSamplesWithFullCoverage) {
  for (double bad_stamp : {1.05, 1.049}) {
    IMUIntegration imu;
    for (int i = 0; i <= 10; ++i) imu.insert_imu(1 + i * .005, {0, 0, 9.80665}, {0, 0, 0});
    imu.insert_imu(bad_stamp, {0, 0, 9.80665}, {0, 0, 0});
    for (int i = 11; i <= 20; ++i) imu.insert_imu(1 + i * .005, {0, 0, 9.80665}, {0, 0, 0});
    EXPECT_THROW((RollingMotion(imu, {}, {}, 1, 1, 1.1)), std::invalid_argument);
  }
}
