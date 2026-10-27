#pragma once
#include <liuo_tunnel/uwb/body_wall_observation.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <array>
#include <liuo_tunnel/uwb/local_floor.hpp>

namespace liuo_tunnel {
// A measured finite patch in ONE scan's body frame, never a global/free axis.
// The 10 m support limit is checked in every factor evaluation. Observed wall
// height bounds are association guards, not known installation heights.
struct LocalInstallationObservation {
  gtsam::Vector3 normal, tangent, up;
  double center_offset, width, minimum_height, maximum_height;
  double surface_model_error, sigma;
  std::array<LocalFloorPatch, 2> floors;
  static constexpr double span = 10.;
  bool supports(const gtsam::Point3& p) const {
    const auto& floor = floors[normal.dot(p) >= center_offset ? 1 : 0];
    return p.allFinite() && std::abs(tangent.dot(p)) <= span && up.dot(p) <= maximum_height + .25 && floor.covers(p) && floor.signed_height(p) >= -floor.margin(p);
  }
};
inline LocalInstallationObservation
local_installation_observation(const std::vector<gtsam::Point3>& raw_body_points, const gtsam::Rot3& gravity, const gtsam::Vector3& velocity_body, double range_sigma) {
  if (!velocity_body.allFinite() || velocity_body.norm() < .5) throw std::invalid_argument("LOCAL_WALL_DIRECTION_PENDING");
  std::vector<gtsam::Point3> points;
  for (const auto& p : raw_body_points)
    if (p.allFinite() && p.norm() <= 25.) points.push_back(p);
  const auto observed = body_wall_observation(points, gravity, range_sigma);
  LocalInstallationObservation result;
  result.up = gravity.unrotate(gtsam::Vector3::UnitZ());
  result.normal = observed.normal;
  const auto left = result.up.cross(velocity_body).eval();
  if (left.norm() < .5 || std::abs(result.normal.dot(left.normalized())) < .95) throw std::invalid_argument("LOCAL_WALL_MOTION_ASSOCIATION_AMBIGUOUS");
  if (result.normal.dot(left) < 0) result.normal *= -1;
  result.tangent = result.normal.cross(result.up).normalized();
  result.center_offset = result.normal.dot(observed.center);
  result.width = observed.leveled.width;
  result.minimum_height = std::numeric_limits<double>::infinity();
  result.maximum_height = -std::numeric_limits<double>::infinity();
  result.surface_model_error = 0;
  std::array<std::array<size_t, 6>, 2> bins{};
  for (const auto& p : points) {
    const double along = result.tangent.dot(p);
    if (along < -15 || along >= 15) continue;
    for (size_t side = 0; side < 2; ++side) {
      const double target = result.center_offset + (side ? 1 : -1) * result.width / 2;
      const double residual = std::abs(result.normal.dot(p) - target);
      if (residual > .12) continue;
      ++bins[side][std::min<size_t>(5, size_t((along + 15) / 5))];
      result.surface_model_error = std::max(result.surface_model_error, residual);
      result.minimum_height = std::min(result.minimum_height, result.up.dot(p));
      result.maximum_height = std::max(result.maximum_height, result.up.dot(p));
    }
  }
  for (const auto& side : bins)
    for (auto count : side)
      if (count < 10) throw std::invalid_argument("LOCAL_WALL_PATCH_NOT_CONTIGUOUS");
  if (result.maximum_height - result.minimum_height < 2 || observed.leveled.residual_rms > .04) throw std::invalid_argument("LOCAL_WALL_PATCH_GEOMETRY_UNSUPPORTED");
  // Worst-case angle/offset uncertainty over this finite patch, plus measured
  // surface-model discrepancy. No synthetic field calibration covariance.
  const auto& C = observed.covariance;
  const double variance = C(1, 1) + 100 * C(0, 0) + 20 * std::abs(C(0, 1));
  result.sigma = std::sqrt(variance + result.surface_model_error * result.surface_model_error);
  if (!std::isfinite(result.sigma) || result.sigma <= 0) throw std::invalid_argument("LOCAL_WALL_UNCERTAINTY_INVALID");
  for (size_t side = 0; side < 2; ++side)
    result.floors[side] = local_floor_patch(points, result.normal, result.tangent, result.up, result.center_offset, result.width, side, range_sigma);
  return result;
}
class LocalInstallationFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
public:
  LocalInstallationFactor(gtsam::Key pose, gtsam::Key anchor, const LocalInstallationObservation& observed, double signed_offset)
  : NoiseModelFactor2(gtsam::noiseModel::Isotropic::Sigma(1, observed.sigma), pose, anchor),
    observed_(observed),
    offset_(signed_offset) {
    if (!std::isfinite(offset_) || std::abs(offset_) > observed.width / 2 + .25) throw std::invalid_argument("LOCAL_OFFSET_OUTSIDE_OBSERVED_WALL_BAND");
  }
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& anchor, gtsam::OptionalMatrixType Hx = nullptr, gtsam::OptionalMatrixType Ha = nullptr)
    const override {
    gtsam::Matrix36 Px;
    gtsam::Matrix33 Pa;
    const auto local = pose.transformTo(anchor, Px, Pa);
    if (!local.allFinite() || std::abs(observed_.tangent.dot(local)) > LocalInstallationObservation::span) throw std::domain_error("LOCAL_INSTALLATION_OUTSIDE_OBSERVED_PATCH");
    if (Hx) *Hx = observed_.normal.transpose() * Px;
    if (Ha) *Ha = observed_.normal.transpose() * Pa;
    return gtsam::Vector1(observed_.normal.dot(local) - observed_.center_offset - offset_);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<LocalInstallationFactor>(*this); }

private:
  LocalInstallationObservation observed_;
  double offset_;
};
}  // namespace liuo_tunnel
