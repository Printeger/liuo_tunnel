#include <gtest/gtest.h>
#include <liuo_tunnel/common/estimator_status.hpp>
#include <limits>
using liuo_tunnel::fused_status;
TEST(EstimatorStatus, MissingAndRejectedMeasurementsAreDifferent) {
  auto absent = fused_status(10, 9, -1, "", .01, .02);
  EXPECT_EQ(absent.state, "DEGRADED");
  EXPECT_EQ(absent.reason, "UWB_ABSENT_LIO_CONTINUES");
  auto rejected = fused_status(10, 9, 9.9, "WINDOW_EXPIRED", .01, .02);
  EXPECT_EQ(rejected.state, "DEGRADED");
  EXPECT_EQ(rejected.reason, "UWB_REJECTED_LIO_CONTINUES:WINDOW_EXPIRED");
  EXPECT_EQ(fused_status(11, 9, 9.9, "WINDOW_EXPIRED", .01, .02).reason, absent.reason);
}
TEST(EstimatorStatus, DuplicateDoesNotSuppressFreshAcceptedInformation) {
  auto recovered = fused_status(10, 9.95, 9.99, "DUPLICATE", .01, .02);
  EXPECT_EQ(recovered.state, "TRACKING");
  EXPECT_EQ(recovered.reason, "JOINT_LIDAR_IMU_UWB");
}
TEST(EstimatorStatus, CovarianceFailureIsNotMisreportedAsHealthyFusion) {
  EXPECT_EQ(fused_status(10, 10, -1, "", .3, 1).reason, "POSITION_POSTERIOR_WEAK");
  auto lost = fused_status(10, 10, -1, "", 101, 102);
  EXPECT_EQ(lost.state, "LOST");
  EXPECT_EQ(lost.reason, "POSTERIOR_UNBOUNDED");
  EXPECT_EQ(fused_status(10, 10, -1, "", std::numeric_limits<double>::quiet_NaN(), 1).state, "LOST");
}
TEST(EstimatorStatus, NeverAcceptedMeasurementCannotLookFreshNearClockOrigin) {
  EXPECT_EQ(fused_status(.01, -1, -1, "", .01, .02).state, "DEGRADED");
}
