#pragma once
#include <liuo_tunnel/uwb/lateral_initialization.hpp>
namespace liuo_tunnel {
// Disjoint ownership before voxelization: reserved points never enter VGICP.
inline bool dedicated_wall_point(bool enabled, size_t raw_index) {
  return enabled && raw_index % 4 == 0;
}
struct BodyWallObservation {
  gtsam::Vector3 normal;
  gtsam::Point3 center;
  gtsam::Matrix2 covariance;
  LocalWallPair leveled;
};
inline BodyWallObservation body_wall_observation(const std::vector<gtsam::Vector3>& points, const gtsam::Rot3& gravity, double range_sigma) {
  if (!std::isfinite(range_sigma) || range_sigma <= 0) throw std::invalid_argument("INVALID_WALL_RANGE_NOISE");
  std::vector<gtsam::Vector3> leveled;
  for (const auto& point : points) leveled.push_back(gravity * point);
  const auto pair = extract_local_walls(leveled);
  const gtsam::Vector3 approximate = gravity.unrotate(gtsam::Vector3(pair.normal.x(), pair.normal.y(), 0));
  const gtsam::Vector3 up = gravity.unrotate(gtsam::Vector3::UnitZ());
  std::array<gtsam::Vector3, 2> normals;
  std::array<double, 2> offsets;
  gtsam::Matrix2 covariance = gtsam::Matrix2::Zero();
  for (size_t side = 0; side < 2; ++side) {
    const double target = pair.center_offset + (side == 0 ? -1 : 1) * pair.width / 2;
    std::vector<gtsam::Vector3> support;
    gtsam::Vector3 center = gtsam::Vector3::Zero();
    for (const auto& point : points)
      if (std::abs(approximate.dot(point) - target) < .08) {
        support.push_back(point);
        center += point;
      }
    if (support.size() < 100) throw std::invalid_argument("INSUFFICIENT_DEDICATED_WALL_SUPPORT");
    center /= support.size();
    gtsam::Matrix3 scatter = gtsam::Matrix3::Zero();
    for (const auto& point : support) {
      const gtsam::Vector3 delta = point - center;
      scatter += delta * delta.transpose();
    }
    Eigen::SelfAdjointEigenSolver<gtsam::Matrix3> eig(scatter / support.size());
    normals[side] = eig.eigenvectors().col(0);
    if (normals[side].dot(approximate) < 0) normals[side] *= -1;
    offsets[side] = normals[side].dot(center);
    // Fit the 3-D plane from points, not from IMU gravity. Gravity only selects
    // the side walls and defines the tangent used by this local covariance.
    const gtsam::Vector3 tangent = up.cross(normals[side]).normalized();
    gtsam::Matrix2 information = gtsam::Matrix2::Zero();
    for (const auto& point : support) {
      const gtsam::Vector2 J(tangent.dot(point), -1);
      information += J * J.transpose();
    }
    // One effective sample per wall: do not claim N independent plane samples
    // from a fitted/voxelized surface. This is a declared local approximation.
    information /= support.size();
    Eigen::LLT<gtsam::Matrix2> solver(information);
    if (solver.info() != Eigen::Success) throw std::invalid_argument("WALL_ANGLE_OFFSET_UNOBSERVABLE");
    const double variance = std::max(range_sigma * range_sigma, eig.eigenvalues()[0]);
    covariance += .25 * variance * solver.solve(gtsam::Matrix2::Identity());
  }
  if (normals[0].dot(normals[1]) < .995) throw std::invalid_argument("NONPARALLEL_DEDICATED_WALLS");
  const gtsam::Vector3 normal = (normals[0] + normals[1]).normalized();
  return {normal, normal * ((offsets[0] + offsets[1]) / 2), covariance, pair};
}
}  // namespace liuo_tunnel
