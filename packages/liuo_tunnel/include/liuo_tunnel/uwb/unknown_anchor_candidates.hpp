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
// General three-dimensional sphere initialization. Estimated tag positions are
// only numerical seeds: uncertainty must still enter through the original joint
// graph. No axis, station, order, anchor height or covariance prior is supplied.
inline UnknownAnchorSeeds free_anchor_seeds(const std::vector<TagRangeSeedObservation>& samples) {
  UnknownAnchorSeeds result;
  result.reason = "ANCHOR_PENDING";
  if (samples.size() < 8) return result;
  if (samples.size() > 512) {
    result.reason = "SEED_CAPACITY_EXCEEDED";
    return result;
  }
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  for (const auto& sample : samples) {
    if (!sample.tag.allFinite() || !std::isfinite(sample.range) || sample.range <= 0) {
      result.reason = "INVALID_RANGE_SEED";
      return result;
    }
    origin += sample.tag;
    result.baseline = std::max(result.baseline, (sample.tag - samples.front().tag).norm());
  }
  if (result.baseline < 20) return result;
  origin /= samples.size();
  double mean_squared = 0;
  for (const auto& sample : samples) mean_squared += sample.range * sample.range - (sample.tag - origin).squaredNorm();
  mean_squared /= samples.size();
  Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
  Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
  for (const auto& sample : samples) {
    const Eigen::Vector3d delta = sample.tag - origin, row = -2 * delta;
    information += row * row.transpose();
    rhs += row * (sample.range * sample.range - delta.squaredNorm() - mean_squared);
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(information);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite() || eig.eigenvalues()[2] < 1e-8) return result;
  const double threshold = eig.eigenvalues()[2] * 1e-8;
  if (eig.eigenvalues()[1] <= threshold) {
    result.reason = "UNOBSERVED_ANCHOR_CIRCLE";
    return result;
  }
  Eigen::Vector3d solution = Eigen::Vector3d::Zero();
  for (int i = 0; i < 3; ++i)
    if (eig.eigenvalues()[i] > threshold) solution += eig.eigenvectors().col(i) * (eig.eigenvectors().col(i).dot(rhs) / eig.eigenvalues()[i]);
  if (eig.eigenvalues()[0] > threshold) {
    result.points.push_back(origin + solution);
    result.reason = "SPATIAL_RANGE_SEED";
  } else {
    const double radius_squared = mean_squared - solution.squaredNorm();
    if (!std::isfinite(radius_squared) || radius_squared < 0) {
      result.reason = "INCONSISTENT_SEED_SPHERES";
      return result;
    }
    for (double sign : {-1., 1.}) result.points.push_back(origin + solution + sign * std::sqrt(radius_squared) * eig.eigenvectors().col(0));
    result.reason = "UNRESOLVED_MIRROR_PAIR";
  }
  return result;
}
// Initial guesses only: tag positions are current graph values, not independent
// measurements. Every original range and the correlated installation likelihood
// must enter the joint graph once. This routine supplies no prior/covariance.
inline UnknownAnchorSeeds unknown_anchor_seeds(const std::vector<TagRangeSeedObservation>& samples, const Eigen::Vector2d& normal, double line_offset, double signed_offset) {
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
