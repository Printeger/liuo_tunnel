#pragma once
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <string>
#include <vector>

namespace liuo_tunnel {
struct TagRangeSeedObservation {
  Eigen::Vector3d tag;
  double range;
};
struct UnknownAnchorSeeds {
  std::vector<Eigen::Vector3d> points;
  std::string reason;
  double baseline = 0;
};
// Initial guesses only: tag positions are current graph values, not independent
// measurements. Every original range and the correlated installation likelihood
// must enter the joint graph once. This routine supplies no prior/covariance.
inline UnknownAnchorSeeds unknown_anchor_seeds(
    const std::vector<TagRangeSeedObservation>& samples, const Eigen::Vector2d& normal, double line_offset, double signed_offset) {
  UnknownAnchorSeeds result;
  result.reason = "ANCHOR_PENDING";
  if (samples.size() < 8) return result;
  if (samples.size() > 512) {
    result.reason = "SEED_CAPACITY_EXCEEDED";
    return result;
  }
  if (!normal.allFinite() || std::abs(normal.norm() - 1) > 1e-8 || !std::isfinite(line_offset) || !std::isfinite(signed_offset)) {
    result.reason = "INVALID_INSTALLATION_SEED";
    return result;
  }
  const Eigen::Vector3d n(normal.x(), normal.y(), 0), tangent(-normal.y(), normal.x(), 0);
  // Center coordinates on the observed trajectory for numerical conditioning.
  Eigen::Vector3d origin = samples.front().tag;
  origin += n * (line_offset + signed_offset - n.dot(origin));
  Eigen::Matrix<double, 3, 2> basis;
  basis.col(0) = tangent;
  basis.col(1) = Eigen::Vector3d::UnitZ();
  std::vector<Eigen::Vector2d> p;
  std::vector<double> squared;
  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  double mean_squared = 0;
  for (const auto& s : samples) {
    if (!s.tag.allFinite() || !std::isfinite(s.range) || s.range <= 0) {
      result.reason = "INVALID_RANGE_SEED";
      return result;
    }
    const auto delta = (s.tag - origin).eval();
    p.push_back(basis.transpose() * delta);
    squared.push_back(s.range * s.range - delta.squaredNorm());
    mean += p.back();
    mean_squared += squared.back();
    result.baseline = std::max(result.baseline, (s.tag - samples.front().tag).norm());
  }
  if (result.baseline < 20) return result;
  mean /= samples.size();
  mean_squared /= samples.size();
  Eigen::Matrix2d information = Eigen::Matrix2d::Zero();
  Eigen::Vector2d rhs = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < samples.size(); ++i) {
    const Eigen::Vector2d row = -2 * (p[i] - mean);
    information += row * row.transpose();
    rhs += row * (squared[i] - mean_squared);
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(information);
  if (eig.info() != Eigen::Success || eig.eigenvalues()[1] < 1e-8) return result;
  // Numerical rank only; it is not an observability/health certificate.
  if (eig.eigenvalues()[0] > eig.eigenvalues()[1] * 1e-8) {
    const Eigen::Vector2d solution = eig.eigenvectors() * eig.eigenvalues().cwiseInverse().asDiagonal() * eig.eigenvectors().transpose() * rhs;
    result.points.push_back(origin + basis * solution);
    result.reason = "MOTION_PLANE_SEED";
    return result;
  }
  const Eigen::Vector2d axis = eig.eigenvectors().col(1), null = eig.eigenvectors().col(0);
  const double along = axis.dot(rhs) / eig.eigenvalues()[1];
  // Averaged original sphere equation along the remaining unobserved direction.
  const double center = null.dot(mean);
  const double radius_squared = mean_squared - along * along + 2 * along * axis.dot(mean) + center * center;
  if (radius_squared < 0) {
    result.reason = "INCONSISTENT_SEED_SPHERES";
    return result;
  }
  const double radius = std::sqrt(radius_squared);
  for (double sign : {-1.0, 1.0}) result.points.push_back(origin + basis * (along * axis + (center + sign * radius) * null));
  result.reason = "UNRESOLVED_MIRROR_PAIR";
  return result;
}
}  // namespace liuo_tunnel
