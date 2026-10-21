#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Cholesky>
#include <stdexcept>
namespace liuo_tunnel {
// State ordering: right-local Pose3 [rotation,translation], world velocity.
// Preserve pose/velocity cross covariance when reporting body-frame velocity.
inline Eigen::Matrix<double, 3, 9> body_velocity_jacobian(const gtsam::Pose3& pose, const gtsam::Vector3& velocity) {
  Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
  H.leftCols<3>() = gtsam::skewSymmetric(pose.rotation().unrotate(velocity));
  H.rightCols<3>() = pose.rotation().matrix().transpose();
  return H;
}
inline gtsam::Matrix66 parent_pose_covariance(const gtsam::Pose3& pose, const gtsam::Matrix66& covariance) {
  gtsam::Matrix66 H = gtsam::Matrix66::Zero();
  H.block<3, 3>(0, 3) = pose.rotation().matrix();
  H.block<3, 3>(3, 0) = pose.rotation().matrix();
  return H * covariance * H.transpose();
}
template <typename BayesTree>
gtsam::Matrix joint_covariance(const BayesTree& tree, gtsam::Key first, gtsam::Key second) {
  auto marginal = tree.joint(first, second);
  auto information = marginal->hessian(gtsam::Ordering{first, second}).first;
  Eigen::LLT<gtsam::Matrix> llt(information);
  if (llt.info() != Eigen::Success) throw std::runtime_error("JOINT_COVARIANCE_NOT_POSITIVE_DEFINITE");
  gtsam::Matrix covariance = llt.solve(gtsam::Matrix::Identity(information.rows(), information.cols()));
  if (!covariance.allFinite()) throw std::runtime_error("JOINT_COVARIANCE_NONFINITE");
  return covariance;
}
}  // namespace liuo_tunnel
