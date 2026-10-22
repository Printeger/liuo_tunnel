#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace liuo_tunnel {
// Numerical seeds only. Original raw ranges and map covariance remain the
// likelihood in the joint graph; never add these positions as observations.
struct RangeSeedSphere {
  gtsam::Point3 anchor;
  double radius;
};
struct RangeSeedResult {
  std::vector<gtsam::Point3> tags;
  std::string reason;
};
inline RangeSeedResult intersect_range_seeds(const std::array<RangeSeedSphere, 3>& spheres) {
  for (const auto& s : spheres)
    if (!s.anchor.allFinite() || !std::isfinite(s.radius) || s.radius <= 0) return {{}, "INVALID_SEED_SPHERE"};
  const auto& a = spheres[0].anchor;
  const gtsam::Vector3 ab = spheres[1].anchor - a, ac = spheres[2].anchor - a;
  const double d = ab.norm();
  if (d < 1e-8) return {{}, "DEGENERATE_SEED_ANCHORS"};
  const gtsam::Vector3 ex = ab / d;
  const double i = ex.dot(ac);
  const gtsam::Vector3 perpendicular = ac - i * ex;
  const double j = perpendicular.norm();
  if (j < 1e-8 * std::max(1., ac.norm())) return {{}, "DEGENERATE_SEED_ANCHORS"};
  const gtsam::Vector3 ey = perpendicular / j, ez = ex.cross(ey);
  const double r0 = spheres[0].radius, r1 = spheres[1].radius, r2 = spheres[2].radius;
  const double x = ((r0 - r1) * (r0 + r1) + d * d) / (2 * d);
  const double y = ((r0 - r2) * (r0 + r2) + i * i + j * j - 2 * i * x) / (2 * j);
  const double z2 = r0 * r0 - x * x - y * y;
  // Roundoff tolerance only; no noise inflation or clipped physical likelihood.
  const double tolerance = 1e-12 * std::max(1., r0 * r0 + x * x + y * y);
  if (z2 < -tolerance) return {{}, "SEED_SPHERES_DO_NOT_INTERSECT"};
  const gtsam::Point3 center = a + x * ex + y * ey;
  if (z2 <= tolerance) return {{center}, "TANGENT_SEED_SPHERES"};
  const double z = std::sqrt(z2);
  return {{gtsam::Point3(center + z * ez), gtsam::Point3(center - z * ez)}, "MIRROR_SEED_PAIR"};
}
inline std::vector<gtsam::Pose3>
pose_seeds_from_tags(const std::vector<gtsam::Point3>& tags, const gtsam::Rot3& gravity_rotation, const gtsam::Vector3& tag_lever, size_t yaw_count = 8) {
  if (yaw_count < 2 || yaw_count > 16 || !tag_lever.allFinite()) throw std::invalid_argument("INVALID_SEED_BUDGET_OR_LEVER");
  std::vector<gtsam::Pose3> poses;
  for (const auto& tag : tags) {
    if (!tag.allFinite()) throw std::invalid_argument("INVALID_TAG_SEED");
    for (size_t k = 0; k < yaw_count; ++k) {
      const gtsam::Rot3 rotation = gtsam::Rot3::Rz(2 * std::acos(-1.) * k / yaw_count) * gravity_rotation;
      poses.emplace_back(rotation, tag - rotation * tag_lever);
    }
  }
  return poses;
}
// Change only the robot-frame gauge used as a numerical starting point.
// The fixed map and body-frame biases must not follow a robot hypothesis.
inline gtsam::Values transform_bootstrap_seed(const gtsam::Values& values, gtsam::Key first_pose, const gtsam::Pose3& seed) {
  const auto delta = seed.compose(values.at<gtsam::Pose3>(first_pose).inverse());
  auto transformed = values;
  for (auto key : values.keys()) {
    const char symbol = gtsam::Symbol(key).chr();
    if (symbol == 'x') transformed.update(key, delta.compose(values.at<gtsam::Pose3>(key)));
    if (symbol == 'v') transformed.update(key, gtsam::Vector3(delta.rotation() * values.at<gtsam::Vector3>(key)));
  }
  return transformed;
}
struct BootstrapBranchEvidence {
  double cost;
  gtsam::Pose3 first_pose;
};
// No independent certificate currently distinguishes persistent positional mirrors
// with the same orientation. Total optimized cost alone was falsified by N7.
// Fail closed until such a certificate exists; this is not a probability test.
inline bool unresolved_position_branch(const std::vector<BootstrapBranchEvidence>& branches, size_t winner) {
  if (winner >= branches.size()) return true;
  for (const auto& branch : branches) {
    const auto delta = branches[winner].first_pose.between(branch.first_pose);
    if (delta.translation().norm() > .5 && gtsam::Rot3::Logmap(delta.rotation()).norm() < .1) return true;
  }
  return false;
}
// A conservative gate over a declared FINITE search, not a global-optimum proof.
// The distances here distinguish result basins, not navigation accuracy limits.
inline bool separated_bootstrap_branch(const std::vector<BootstrapBranchEvidence>& branches, size_t winner, double minimum_cost_gap = 5.) {
  if (branches.size() < 2 || winner >= branches.size() || !std::isfinite(minimum_cost_gap) || minimum_cost_gap <= 0) return false;
  for (const auto& branch : branches) {
    if (!std::isfinite(branch.cost) || !branch.first_pose.matrix().allFinite()) return false;
    const auto delta = branches[winner].first_pose.between(branch.first_pose);
    if ((delta.translation().norm() > .5 || gtsam::Rot3::Logmap(delta.rotation()).norm() > .1) && branch.cost - branches[winner].cost < minimum_cost_gap) return false;
  }
  return true;
}
}  // namespace liuo_tunnel
