#pragma once
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <optional>
#include <stdexcept>
namespace liuo_tunnel {
// A comparison witness only: never an independent pose or range observation.
struct AnchorRelativeCandidate {
  gtsam::Key reference;
  gtsam::Point3 point;
  static AnchorRelativeCandidate from_joint(gtsam::Key key, const gtsam::Pose3& pose, const gtsam::Point3& anchor) {
    const auto local = pose.transformTo(anchor);
    if (!local.allFinite()) throw std::invalid_argument("NONFINITE_RELATIVE_ANCHOR");
    return {key, local};
  }
  gtsam::Point3 in_reference(gtsam::Key key, const gtsam::Pose3& pose) const {
    if (key != reference) throw std::invalid_argument("ANCHOR_REFERENCE_KEY_MISMATCH");
    const auto result = pose.transformFrom(point);
    if (!result.allFinite()) throw std::invalid_argument("NONFINITE_RELATIVE_ANCHOR");
    return result;
  }
  std::optional<double> displacement(const AnchorRelativeCandidate& other) const {
    if (reference != other.reference) return std::nullopt;
    if (!point.allFinite() || !other.point.allFinite()) throw std::invalid_argument("NONFINITE_RELATIVE_ANCHOR");
    return (point - other.point).norm();
  }
};
}  // namespace liuo_tunnel
