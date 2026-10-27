#pragma once
#include <gtsam/geometry/Point3.h>
#include <Eigen/Eigenvalues>
#include <random>
#include <vector>
#include <stdexcept>
#include <string>
namespace liuo_tunnel {
struct LocalFloorPatch {
  gtsam::Vector3 normal, center, along, across;
  double variance, along_spread, across_spread;
  gtsam::Vector3 longitudinal_axis;
  double minimum_longitudinal, maximum_longitudinal;
  bool covers(const gtsam::Point3& point) const {
    const double position = longitudinal_axis.dot(point);
    return point.allFinite() && position >= minimum_longitudinal && position <= maximum_longitudinal;
  }
  double signed_height(const gtsam::Point3& point) const { return normal.dot(point - center); }
  double margin(const gtsam::Point3& point) const {
    const auto delta = (point - center).eval();
    return 3 * std::sqrt(variance * (1 + std::pow(along.dot(delta), 2) / along_spread + std::pow(across.dot(delta), 2) / across_spread));
  }
};
inline LocalFloorPatch local_floor_patch(
  const std::vector<gtsam::Point3>& points,
  const gtsam::Vector3& wall_normal,
  const gtsam::Vector3& tangent,
  const gtsam::Vector3& up,
  double center_offset,
  double width,
  size_t side,
  double range_sigma) {
  std::vector<gtsam::Point3> band;
  const double sign = side ? 1 : -1;
  for (const auto& p : points) {
    const double lateral = sign * (wall_normal.dot(p) - center_offset);
    if (lateral >= width / 2 - .9 && lateral <= width / 2 - .04 && std::abs(tangent.dot(p)) <= 20) band.push_back(p);
  }
  if (band.size() < 30) throw std::invalid_argument("LOCAL_FLOOR_NOT_OBSERVED");
  std::mt19937 random(17);
  std::vector<size_t> best;
  for (size_t trial = 0; trial < 256; ++trial) {
    const auto& a = band[random() % band.size()];
    gtsam::Vector3 normal = (band[random() % band.size()] - a).cross(band[random() % band.size()] - a);
    if (normal.norm() < .05) continue;
    normal.normalize();
    if (normal.dot(up) < 0) normal *= -1;
    if (normal.dot(up) < .95 || normal.dot(a) >= 0) continue;
    std::vector<size_t> supported;
    for (size_t i = 0; i < band.size(); ++i)
      if (std::abs(normal.dot(band[i] - a)) <= 3 * range_sigma) supported.push_back(i);
    if (supported.size() > best.size()) best = std::move(supported);
  }
  if (best.size() < 30) throw std::invalid_argument("LOCAL_FLOOR_NOT_OBSERVED");
  LocalFloorPatch result;
  result.center = gtsam::Vector3::Zero();
  for (auto i : best) result.center += band[i];
  result.center /= best.size();
  gtsam::Matrix3 scatter = gtsam::Matrix3::Zero();
  double lo = INFINITY, hi = -INFINITY, left = INFINITY, right = -INFINITY;
  for (auto i : best) {
    const auto delta = (band[i] - result.center).eval();
    scatter += delta * delta.transpose();
    lo = std::min(lo, tangent.dot(band[i]));
    hi = std::max(hi, tangent.dot(band[i]));
    left = std::min(left, wall_normal.dot(band[i]));
    right = std::max(right, wall_normal.dot(band[i]));
  }
  Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> eig(scatter / best.size());
  if (eig.info() != Eigen::Success) throw std::invalid_argument("LOCAL_FLOOR_EIGENSOLVER_FAILED");
  if (eig.eigenvalues()[1] < .005 || eig.eigenvalues()[0] > .04 * .04 || right - left < .3)
    throw std::invalid_argument(
      "LOCAL_FLOOR_SUPPORT_OR_PLANARITY;side=" + std::to_string(side) + ";inliers=" + std::to_string(best.size()) + ";lambda_min=" + std::to_string(eig.eigenvalues()[0]) +
      ";lambda_mid=" + std::to_string(eig.eigenvalues()[1]) + ";lo=" + std::to_string(lo) + ";hi=" + std::to_string(hi) + ";width=" + std::to_string(right - left));
  result.longitudinal_axis = tangent;
  result.minimum_longitudinal = lo;
  result.maximum_longitudinal = hi;
  result.normal = eig.eigenvectors().col(0);
  if (result.normal.dot(up) < 0) result.normal *= -1;
  if (result.normal.dot(up) < .95 || result.normal.dot(result.center) >= 0) throw std::invalid_argument("LOCAL_FLOOR_ORIENTATION");
  result.along = eig.eigenvectors().col(2);
  result.across = eig.eigenvectors().col(1);
  result.along_spread = eig.eigenvalues()[2];
  result.across_spread = eig.eigenvalues()[1];
  result.variance = std::max(range_sigma * range_sigma, eig.eigenvalues()[0]);
  return result;
}
}  // namespace liuo_tunnel
