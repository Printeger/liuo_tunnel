// Adapted from IAP include/iap/common/cloud_covariance_estimation.hpp @ fbccae168; namespace and independent project paths. See THIRD_PARTY_NOTICES.md.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <stdexcept>

namespace liuo_tunnel {

// Conservative surface-normal support, evaluated from measured local geometry.
// No world/rail axis or vehicle truth is involved. Weak translation directions
// have less than 1% of the strongest normal scatter eigenvalue.
inline Eigen::Matrix3d surface_translation_projector(const std::vector<Eigen::Matrix4d>& covariances, Eigen::Vector3d* spectrum = nullptr) {
  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  size_t supported = 0;
  for (const auto& covariance : covariances) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covariance.topLeftCorner<3, 3>());
    if (eig.info() != Eigen::Success) throw std::runtime_error("SURFACE_SPECTRUM_FAILED");
    if (eig.eigenvalues()[0] >= .01) continue;  // Isotropic unsupported neighborhoods.
    const Eigen::Vector3d normal = eig.eigenvectors().col(0);
    scatter += normal * normal.transpose();
    ++supported;
  }
  if (supported < 3) throw std::runtime_error("SURFACE_PROJECTION_REQUIRES_SUPPORTED_NORMALS");
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter / double(supported));
  if (spectrum) *spectrum = eig.eigenvalues();
  Eigen::Vector3d keep;
  for (int i = 0; i < 3; ++i) keep[i] = eig.eigenvalues()[i] >= .01 * eig.eigenvalues()[2] ? 1.0 : 0.0;
  return eig.eigenvectors() * keep.asDiagonal() * eig.eigenvectors().transpose();
}

/**
 * @brief Covariance regularization method
 */
enum class RegularizationMethod { NONE, PLANE, NORMALIZED_MIN_EIG, FROBENIUS };

/**
 * @brief Point covariance estimation
 */
class CloudCovarianceEstimation {
public:
  CloudCovarianceEstimation(const int num_threads = 1, bool validate_planarity = true, double tangent_variance = 1.0);
  ~CloudCovarianceEstimation();

  /**
   * @brief Estimate point normals and covariances
   * @param points    Input points
   * @param neighbors Neighbor indices (must be N * k, where N is the number of points)
   * @param normals   [output] Estimated normals
   * @param covs      [output] Estimated covariances
   */
  void estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, std::vector<Eigen::Vector4d>& normals, std::vector<Eigen::Matrix4d>& covs) const;

  /**
   * @brief Estimate point normals and covariances
   * @param points      Input points
   * @param neighbors   Neighbor indices (must be N * m, where N is the number of points)
   * @param k_neighbors Number of neighbors used for estimation (must be <= m)
   * @param normals     [output] Estimated normals
   * @param covs        [output] Estimated covariances
   */
  void estimate(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<int>& neighbors,
    const int k_neighbors,
    std::vector<Eigen::Vector4d>& normals,
    std::vector<Eigen::Matrix4d>& covs) const;

  /// Estimate point covariances
  std::vector<Eigen::Matrix4d> estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors, const int k_neighbors) const;

  /// Estimate point covariances
  std::vector<Eigen::Matrix4d> estimate(const std::vector<Eigen::Vector4d>& points, const std::vector<int>& neighbors) const;

  /**
   * @brief Regularize a covariance matrix
   * @param cov          Input covariance matrix
   * @param eigenvalues  [output] Eigenvalues of the covariance matrix
   * @param eigenvectors [output] Eigenvectors of the covariance matrix
   * @return Regularized covariance matrix
   */
  Eigen::Matrix4d regularize(const Eigen::Matrix4d& cov, Eigen::Vector3d* eigenvalues = nullptr, Eigen::Matrix3d* eigenvectors = nullptr) const;

private:
  const bool validate_planarity;
  const double tangent_variance;
  const RegularizationMethod regularization_method;
  const int num_threads;
};

}  // namespace liuo_tunnel
