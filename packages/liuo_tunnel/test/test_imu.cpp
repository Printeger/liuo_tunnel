#include <gtest/gtest.h>
#include <liuo_tunnel/common/imu_integration.hpp>
TEST(Imu, ConstantAccelerationAndNoiseDensity) {
  liuo_tunnel::IMUIntegrationParams p;
  EXPECT_NEAR(p.acc_noise * p.acc_noise, .02 * .02 / 200, 1e-14);
  liuo_tunnel::IMUIntegration imu(p);
  for (int i = 0; i <= 200; i++) imu.insert_imu(1 + i * .005, {1, 0, 9.80665}, {0, 0, 0});
  int n = 0;
  imu.integrate_imu(1, 2, gtsam::imuBias::ConstantBias(), &n);
  auto out = imu.integrated_measurements().predict(gtsam::NavState(), gtsam::imuBias::ConstantBias());
  EXPECT_EQ(n, 200);
  EXPECT_NEAR(out.position().x(), .5, 1e-9);
  EXPECT_NEAR(out.velocity().x(), 1, 1e-9);
  EXPECT_NEAR(out.position().z(), 0, 1e-9);
}
#include <liuo_tunnel/common/sensors.hpp>
TEST(Sensors, NonidentityExtrinsicsAndLeverArm) {
  Eigen::Isometry3d wb = Eigen::Isometry3d::Identity(), bl = wb;
  wb.translation() = Eigen::Vector3d(10, 20, 1.2);
  wb.linear() = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  bl.translation() = Eigen::Vector3d(1, 0, 0);
  bl.linear() = wb.linear();
  auto point = liuo_tunnel::point_in_lidar(wb, bl, {10, 22, 1.2});
  EXPECT_NEAR(point.x(), 0, 1e-10);
  EXPECT_NEAR(point.y(), -1, 1e-10);
  EXPECT_NEAR(point.z(), 0, 1e-10);
  auto tag = liuo_tunnel::tag_in_world(wb.translation(), wb.linear(), {1, 2, 3});
  EXPECT_NEAR(tag.x(), 8, 1e-10);
  EXPECT_NEAR(tag.y(), 21, 1e-10);
  EXPECT_NEAR(tag.z(), 4.2, 1e-10);
}

#include <liuo_tunnel/common/state_covariance.hpp>
TEST(StateCovariance, BodyVelocityRightRetractAndCrossTerms) {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(.4, -.3, 1.2), {3, 2, 1});
  const gtsam::Vector3 velocity(2, -3, 4);
  const auto H = liuo_tunnel::body_velocity_jacobian(pose, velocity);
  Eigen::Matrix<double, 3, 9> numerical;
  for (int i = 0; i < 9; i++) {
    gtsam::Vector6 d = gtsam::Vector6::Zero();
    gtsam::Vector3 dv = gtsam::Vector3::Zero();
    if (i < 6)
      d[i] = 1e-6;
    else
      dv[i - 6] = 1e-6;
    numerical.col(i) = (pose.retract(d).rotation().unrotate(velocity + dv) - pose.retract(-d).rotation().unrotate(velocity - dv)) / 2e-6;
  }
  EXPECT_LT((H - numerical).norm(), 1e-8);
  Eigen::Matrix<double, 9, 9> root = Eigen::Matrix<double, 9, 9>::Identity();
  root.block<3, 3>(6, 0) = .5 * gtsam::Matrix33::Identity();
  auto covariance = (root * root.transpose()).eval();
  auto transformed = (H * covariance * H.transpose()).eval();
  auto numerical_covariance = (numerical * covariance * numerical.transpose()).eval();
  EXPECT_LT((transformed - numerical_covariance).norm(), 1e-7);
  covariance.block<6, 3>(0, 6).setZero();
  covariance.block<3, 6>(6, 0).setZero();
  EXPECT_GT((transformed - H * covariance * H.transpose()).norm(), .1);
  gtsam::Matrix66 local = gtsam::Matrix66::Identity();
  local.diagonal() << 1, 2, 3, 4, 5, 6;
  auto parent = liuo_tunnel::parent_pose_covariance(pose, local);
  EXPECT_LT((parent.topLeftCorner<3, 3>() - pose.rotation().matrix() * local.bottomRightCorner<3, 3>() * pose.rotation().matrix().transpose()).norm(), 1e-12);
}
