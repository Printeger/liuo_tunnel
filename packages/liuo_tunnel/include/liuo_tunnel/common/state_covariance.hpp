#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
#include <algorithm>
#include <map>
#include <set>
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
// Discrete body twist: current body linear velocity and SO(3) log increment
// over [previous,current], expressed in the current body frame. This angular
// estimate is an interval quantity, not an instantaneous raw gyroscope sample.
// Columns: previous right-local Pose3, current right-local Pose3, world velocity.
inline gtsam::Vector6
interval_body_twist(const gtsam::Pose3& previous, const gtsam::Pose3& current, const gtsam::Vector3& velocity, double dt, Eigen::Matrix<double, 6, 15>* H = nullptr) {
  if (!std::isfinite(dt) || dt <= 0) throw std::invalid_argument("INVALID_TWIST_INTERVAL");
  gtsam::Matrix33 Dprevious, Dcurrent, Dlog;
  auto relative = previous.rotation().between(current.rotation(), Dprevious, Dcurrent);
  auto rotation_vector = gtsam::Rot3::Logmap(relative, Dlog);
  gtsam::Vector6 twist;
  twist.head<3>() = current.rotation().unrotate(velocity);
  twist.tail<3>() = rotation_vector / dt;
  if (H) {
    H->setZero();
    H->block<3, 9>(0, 6) = body_velocity_jacobian(current, velocity);
    H->block<3, 3>(3, 0) = Dlog * Dprevious / dt;
    H->block<3, 3>(3, 6) = Dlog * Dcurrent / dt;
  }
  return twist;
}
inline gtsam::Matrix66 parent_pose_covariance(const gtsam::Pose3& pose, const gtsam::Matrix66& covariance) {
  gtsam::Matrix66 H = gtsam::Matrix66::Zero();
  H.block<3, 3>(0, 3) = pose.rotation().matrix();
  H.block<3, 3>(3, 0) = pose.rotation().matrix();
  return H * covariance * H.transpose();
}
// Exact selected marginal of the current Gaussian Bayes tree. Only ancestors
// of selected frontals contribute. Their whitened conditionals form triangular
// R without re-elimination; C_selected = (R^-T E)^T (R^-T E). No covariance
// approximation, independence assumption or added information is introduced.
template <typename BayesTree>
gtsam::Matrix selected_covariance(const BayesTree& tree, const gtsam::KeyVector& keys) {
  if (keys.empty()) throw std::invalid_argument("EMPTY_COVARIANCE_SELECTION");
  using CliquePtr = decltype(tree[keys.front()]);
  std::vector<std::pair<size_t, CliquePtr>> cliques;
  std::set<const void*> seen;
  for (auto key : keys) {
    auto clique = tree[key];
    while (clique) {
      if (seen.insert(clique.get()).second) {
        size_t depth = 0;
        for (auto parent = clique->parent(); parent; parent = parent->parent()) ++depth;
        cliques.emplace_back(depth, clique);
      }
      clique = clique->parent();
    }
  }
  std::stable_sort(cliques.begin(), cliques.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
  std::map<gtsam::Key, std::pair<int, int>> blocks;
  int dimension = 0;
  for (const auto& item : cliques) {
    const auto& conditional = item.second->conditional();
    if (conditional->isConstrained()) throw std::runtime_error("CONSTRAINED_COVARIANCE_UNSUPPORTED");
    for (auto it = conditional->beginFrontals(); it != conditional->endFrontals(); ++it) {
      int n = conditional->getDim(it);
      blocks.emplace(*it, std::make_pair(dimension, n));
      dimension += n;
    }
  }
  std::vector<Eigen::Triplet<double>> entries;
  int row_base = 0;
  for (const auto& item : cliques) {
    const auto& conditional = item.second->conditional();
    const auto jacobian = conditional->augmentedJacobian();  // includes noise whitening
    int local_column = 0;
    for (auto key : conditional->keys()) {
      auto [column_base, n] = blocks.at(key);
      for (int column = 0; column < n; ++column)
        for (int row = 0; row < jacobian.rows(); ++row) {
          double value = jacobian(row, local_column + column);
          if (value != 0) {
            if (column_base + column < row_base + row) throw std::runtime_error("COVARIANCE_CONDITIONAL_NOT_TRIANGULAR");
            entries.emplace_back(column_base + column, row_base + row, value);
          }
        }
      local_column += n;
    }
    row_base += jacobian.rows();
  }
  Eigen::SparseMatrix<double> transpose_system(dimension, dimension);
  transpose_system.setFromTriplets(entries.begin(), entries.end());
  for (int i = 0; i < dimension; ++i)
    if (!std::isfinite(transpose_system.coeff(i, i)) || transpose_system.coeff(i, i) == 0) throw std::runtime_error("JOINT_COVARIANCE_SINGULAR");
  int selected_dimension = 0;
  std::set<gtsam::Key> unique;
  for (auto key : keys) {
    if (!unique.insert(key).second) throw std::invalid_argument("DUPLICATE_COVARIANCE_KEY");
    selected_dimension += blocks.at(key).second;
  }
  gtsam::Matrix selector = gtsam::Matrix::Zero(dimension, selected_dimension);
  int column = 0;
  for (auto key : keys) {
    auto [offset, n] = blocks.at(key);
    selector.block(offset, column, n, n).setIdentity();
    column += n;
  }
  gtsam::Matrix solved = transpose_system.template triangularView<Eigen::Lower>().solve(selector);
  gtsam::Matrix covariance = solved.transpose() * solved;
  if (!covariance.allFinite()) throw std::runtime_error("JOINT_COVARIANCE_NONFINITE");
  return covariance;
}
template <typename BayesTree>
gtsam::Matrix joint_covariance(const BayesTree& tree, gtsam::Key first, gtsam::Key second) {
  return selected_covariance(tree, gtsam::KeyVector{first, second});
}
}  // namespace liuo_tunnel
