#pragma once
#include <liuo_tunnel/uwb/body_wall_observation.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <array>
#include <optional>
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
  double minimum_longitudinal = -span, maximum_longitudinal = span;
  bool supports_longitudinal(const gtsam::Point3& p) const {
    const double position = tangent.dot(p);
    return p.allFinite() && position >= minimum_longitudinal && position <= maximum_longitudinal;
  }
  bool supports(const gtsam::Point3& p) const {
    const auto& floor = floors[normal.dot(p) >= center_offset ? 1 : 0];
    return supports_longitudinal(p) && up.dot(p) <= maximum_height + .25 && floor.covers(p) && floor.signed_height(p) >= -floor.margin(p);
  }
};
inline LocalInstallationObservation local_installation_observation(
  const std::vector<gtsam::Point3>& raw_body_points,
  const gtsam::Rot3& gravity,
  const gtsam::Vector3& velocity_body,
  double range_sigma,
  const std::optional<gtsam::Vector3>& previous_left_body = std::nullopt) {
  if (!previous_left_body && (!velocity_body.allFinite() || velocity_body.norm() < .5)) throw std::invalid_argument("LOCAL_WALL_DIRECTION_PENDING");
  std::vector<gtsam::Point3> points;
  for (const auto& p : raw_body_points)
    if (p.allFinite() && p.norm() <= 25.) points.push_back(p);
  const auto observed = body_wall_observation(points, gravity, range_sigma);
  LocalInstallationObservation result;
  result.up = gravity.unrotate(gtsam::Vector3::UnitZ());
  result.normal = observed.normal;
  // Installation side is an orientation convention, not the current sign of
  // estimated velocity. Transport the last observed normal through retained
  // joint poses; use motion only to establish the initial convention.
  const gtsam::Vector3 left = previous_left_body ? *previous_left_body : result.up.cross(velocity_body).eval();
  if (!left.allFinite() || left.norm() < .5 || std::abs(result.normal.dot(left.normalized())) < .95) throw std::invalid_argument("LOCAL_WALL_MOTION_ASSOCIATION_AMBIGUOUS");
  if (result.normal.dot(left) < 0) result.normal *= -1;
  result.tangent = result.normal.cross(result.up).normalized();
  result.center_offset = result.normal.dot(observed.center);
  result.width = observed.leveled.width;
  result.minimum_height = std::numeric_limits<double>::infinity();
  result.maximum_height = -std::numeric_limits<double>::infinity();
  result.surface_model_error = 0;
  std::array<std::array<size_t, 6>, 2> bins{};
  std::array<std::array<double, 6>, 2> bin_min, bin_max;
  for (size_t side = 0; side < 2; ++side) {
    bin_min[side].fill(INFINITY);
    bin_max[side].fill(-INFINITY);
  }
  for (const auto& p : points) {
    const double along = result.tangent.dot(p);
    if (along < -15 || along >= 15) continue;
    for (size_t side = 0; side < 2; ++side) {
      const double target = result.center_offset + (side ? 1 : -1) * result.width / 2;
      const double residual = std::abs(result.normal.dot(p) - target);
      if (residual > .12) continue;
      const auto bin = std::min<size_t>(5, size_t((along + 15) / 5));
      ++bins[side][bin];
      bin_min[side][bin] = std::min(bin_min[side][bin], along);
      bin_max[side][bin] = std::max(bin_max[side][bin], along);
      result.surface_model_error = std::max(result.surface_model_error, residual);
      result.minimum_height = std::min(result.minimum_height, result.up.dot(p));
      result.maximum_height = std::max(result.maximum_height, result.up.dot(p));
    }
  }
  // Keep the same per-bin support criterion. At a portal, retain only the
  // contiguous two-sided component surrounding the sensor; do not fabricate
  // the missing wall behind it or join components across an observed gap.
  const auto supported = [&](size_t bin) { return bins[0][bin] >= 10 && bins[1][bin] >= 10; };
  if (!supported(2) || !supported(3)) throw std::invalid_argument("LOCAL_WALL_PATCH_NOT_CONTIGUOUS");
  size_t first = 2, last = 3;
  while (first > 0 && supported(first - 1)) --first;
  while (last < 5 && supported(last + 1)) ++last;
  for (size_t side = 0; side < 2; ++side) {
    result.minimum_longitudinal = std::max(result.minimum_longitudinal, bin_min[side][first]);
    result.maximum_longitudinal = std::min(result.maximum_longitudinal, bin_max[side][last]);
  }
  if (!(result.minimum_longitudinal < 0 && result.maximum_longitudinal > 0)) throw std::invalid_argument("LOCAL_WALL_PATCH_NOT_SURROUNDING_SENSOR");
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
  double error(const gtsam::Values& values) const override {
    const auto local = values.at<gtsam::Pose3>(key1()).transformTo(values.at<gtsam::Point3>(key2()));
    // A finite-domain observation has infinite trial cost outside its support.
    // Dogleg/LM may then reject the trial; no plane residual is extrapolated.
    // Invalid linearization states still fail explicitly in evaluateError.
    if (local.allFinite() && !supports_longitudinal(local)) return std::numeric_limits<double>::infinity();
    return gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>::error(values);
  }
  gtsam::Vector evaluateError(const gtsam::Pose3& pose, const gtsam::Point3& anchor, gtsam::OptionalMatrixType Hx = nullptr, gtsam::OptionalMatrixType Ha = nullptr)
    const override {
    gtsam::Matrix36 Px;
    gtsam::Matrix33 Pa;
    const auto local = pose.transformTo(anchor, Px, Pa);
    if (!supports_longitudinal(local))
      throw std::domain_error(
        "LOCAL_INSTALLATION_OUTSIDE_OBSERVED_PATCH;linearization=" + std::to_string(bool(Hx) || bool(Ha)) + ";along=" + std::to_string(observed_.tangent.dot(local)));
    if (Hx) *Hx = observed_.normal.transpose() * Px;
    if (Ha) *Ha = observed_.normal.transpose() * Pa;
    return gtsam::Vector1(observed_.normal.dot(local) - observed_.center_offset - offset_);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<LocalInstallationFactor>(*this); }

private:
  bool supports_longitudinal(const gtsam::Point3& local) const {
    // The known signed installation offset selects the measured floor band.
    // Its observed extent is also required by final candidate association;
    // never optimize against an extrapolated floor beyond that extent.
    return observed_.supports_longitudinal(local) && observed_.floors[offset_ >= 0 ? 1 : 0].covers(local);
  }
  LocalInstallationObservation observed_;
  double offset_;
};
}  // namespace liuo_tunnel
