#include <gtest/gtest.h>
#include <liuo_tunnel/common/imu_integration.hpp>
TEST(Imu, ConstantAccelerationAndNoiseDensity) {
 liuo_tunnel::IMUIntegrationParams p;
 EXPECT_NEAR(p.acc_noise*p.acc_noise,.02*.02/200,1e-14);
 liuo_tunnel::IMUIntegration imu(p);
 for(int i=0;i<=200;i++)imu.insert_imu(1+i*.005,{1,0,9.81},{0,0,0});
 int n=0;imu.integrate_imu(1,2,gtsam::imuBias::ConstantBias(),&n);
 auto out=imu.integrated_measurements().predict(gtsam::NavState(),gtsam::imuBias::ConstantBias());
 EXPECT_EQ(n,200);EXPECT_NEAR(out.position().x(),.5,1e-9);EXPECT_NEAR(out.velocity().x(),1,1e-9);EXPECT_NEAR(out.position().z(),0,1e-9);
}
