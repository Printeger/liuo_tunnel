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
