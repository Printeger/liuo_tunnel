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
TEST(EstimatorStatus, RecoveryRequiresFreshWindowAndDoesNotHideNewOutage) {
  auto recovering = fused_status(21, 20.99, -1, "", .01, .02, 31);
  EXPECT_EQ(recovering.state, "DEGRADED");
  EXPECT_EQ(recovering.reason, "UWB_RECOVERING_WINDOW");
  EXPECT_EQ(fused_status(30.9, 30.89, -1, "", .01, .02, 31).state, "DEGRADED");
  EXPECT_EQ(fused_status(31, 30.99, -1, "", .01, .02, 31).reason, "UWB_RECOVERY_UNVERIFIED");
  EXPECT_EQ(fused_status(25, 21, -1, "", .01, .02, 31).reason, "UWB_ABSENT_LIO_CONTINUES");
}

TEST(EstimatorStatus, PostfitDisagreementPreventsFalseRecoveryWithSmallPosterior) {
  const size_t n = 38;
  const double limit = liuo_tunnel::postfit_range_limit(n);
  EXPECT_GT(limit, double(n));
  EXPECT_LT(limit, 100.);
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, 71, n, 1225. * 1225. * n).reason, "UWB_POSTFIT_INCONSISTENT");
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, -1, n, limit).state, "TRACKING");
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, 71, n, limit + 1).state, "LOST");
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, -1, n, double(n)).state, "TRACKING");
}
TEST(EstimatorStatus, PostfitNoDataNonfiniteAndMissingInputAreExplicit) {
  EXPECT_TRUE(std::isinf(liuo_tunnel::postfit_range_limit(0)));
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, -1, 0, 0).state, "TRACKING");
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, 71, 2, NAN).reason, "UWB_POSTFIT_NONFINITE_OR_INVALID");
  EXPECT_EQ(fused_status(72, 71.99, -1, "", .01, .02, 71, 2, -1).state, "LOST");
  EXPECT_EQ(fused_status(72, 70, -1, "", .01, .02, 71, 2, 1e9).reason, "UWB_ABSENT_LIO_CONTINUES");
}

TEST(EstimatorStatus, FailedRecoveryCannotClearItselfWithSmallResiduals) {
  using liuo_tunnel::recovery_failure_stamp;
  double failure = recovery_failure_stamp(-1, 61.1, 71.1, 4, 3862991.639);
  EXPECT_DOUBLE_EQ(failure, 61.1);
  failure = recovery_failure_stamp(failure, 62, 71.1, 4, 1);
  EXPECT_DOUBLE_EQ(failure, 61.1);
  auto later = fused_status(92.2, 92.19, -1, "", .01, .02, 71.1, 5, 6.2197, failure);
  EXPECT_EQ(later.state, "DEGRADED");
  EXPECT_EQ(later.reason, "UWB_RECOVERY_UNVERIFIED");
  EXPECT_EQ(fused_status(93, 93, -1, "", .01, .02, 71.1, 4, 1e9, failure).state, "LOST");
  EXPECT_EQ(fused_status(93, 92, -1, "", .01, .02, 71.1, 4, 1, failure).reason, "UWB_ABSENT_LIO_CONTINUES");
  EXPECT_EQ(fused_status(93, 93, -1, "", NAN, NAN, 71.1, 4, 1, failure).reason, "POSTERIOR_NONFINITE");
  // A new session explicitly resets both the failure stamp and recovery deadline.
  EXPECT_EQ(fused_status(9, 9, -1, "", .01, .02, -1, 4, 1, -1).state, "TRACKING");
}
TEST(EstimatorStatus, RecoveryFailureOnlyLatchesAnActualFailedWindow) {
  using liuo_tunnel::recovery_failure_stamp;
  EXPECT_EQ(recovery_failure_stamp(-1, 9, -1, 4, 1e9), -1);
  EXPECT_EQ(recovery_failure_stamp(-1, 92, 71, 4, 1e9), -1);
  EXPECT_EQ(recovery_failure_stamp(-1, 62, 71, 0, 1e9), -1);
  EXPECT_EQ(recovery_failure_stamp(-1, 62, 71, 4, 1), -1);
  EXPECT_EQ(recovery_failure_stamp(-1, 62, 71, 4, NAN), 62);
  EXPECT_EQ(recovery_failure_stamp(62, 63, 71, 4, 1e9), 62);
}

TEST(EstimatorStatus, ExpiredTimerAndSmallResidualDoNotCertifyRecovery) {
  const auto status = fused_status(71.2, 71.19, -1, "", .16, .17, 71.1, 6, 5.0, -1);
  EXPECT_EQ(status.state, "DEGRADED");
  EXPECT_EQ(status.reason, "UWB_RECOVERY_UNVERIFIED");
  EXPECT_EQ(fused_status(80, 79, -1, "", .16, .17, 71.1).reason, "UWB_ABSENT_LIO_CONTINUES");
  EXPECT_EQ(fused_status(80, 80, -1, "", NAN, NAN, 71.1).reason, "POSTERIOR_NONFINITE");
  EXPECT_EQ(fused_status(9, 9, -1, "", .01, .02, -1).state, "TRACKING");
}
