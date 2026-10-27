#pragma once
#include <liuo_tunnel/uwb/calibrated_map.hpp>
#include <gtsam/geometry/Pose3.h>
#include <cmath>

namespace liuo_tunnel {
// Both estimator worlds define gravity as -Z. This is a frame relation, not a
// road axis: yaw in milliradians, then map-frame translation in metres.
inline gtsam::Pose3 gravity_frame_pose(const gtsam::Vector4& frame) {
  if (!frame.allFinite()) throw std::invalid_argument("MAP_FRAME_NONFINITE");
  return {gtsam::Rot3::Rz(frame[0] * .001), gtsam::Point3(frame.tail<3>())};
}
inline gtsam::Pose3 map_frame_pose(const gtsam::Vector4& frame, const gtsam::Pose3& local, Eigen::Matrix<double, 6, 4>* Hframe = nullptr, gtsam::Matrix66* Hlocal = nullptr) {
  const auto transform = gravity_frame_pose(frame);
  gtsam::Matrix66 composition;
  const auto pose = transform.compose(local, composition, Hlocal ? gtsam::OptionalJacobian<6, 6>(*Hlocal) : gtsam::OptionalJacobian<6, 6>());
  if (Hframe) {
    Eigen::Matrix<double, 6, 4> chart = Eigen::Matrix<double, 6, 4>::Zero();
    chart(2, 0) = .001;
    chart.bottomRightCorner<3, 3>() = transform.rotation().matrix().transpose();
    *Hframe = composition * chart;
  }
  return pose;
}

// Output chart: right-perturbed pose then map-world velocity; input is
// [G(4), local right-pose(6), local world velocity(3)].
inline Eigen::Matrix<double, 9, 13> map_frame_navigation_jacobian(const gtsam::Vector4& frame, const gtsam::Pose3& local, const gtsam::Vector3& velocity) {
  Eigen::Matrix<double, 6, 4> Hg;
  gtsam::Matrix66 Hx;
  map_frame_pose(frame, local, &Hg, &Hx);
  const auto rotation = gravity_frame_pose(frame).rotation();
  Eigen::Matrix<double, 9, 13> H = Eigen::Matrix<double, 9, 13>::Zero();
  H.block<6, 4>(0, 0) = Hg;
  H.block<6, 6>(0, 4) = Hx;
  H.block<3, 1>(6, 0) = .001 * gtsam::Vector3::UnitZ().cross(rotation.rotate(velocity));
  H.block<3, 3>(6, 10) = rotation.matrix();
  return H;
}

// Jacobian for exporting all A in the inherited map frame. Column order is
// G then local A in the exact order used by selected_covariance. Do not drop
// G/A cross terms: map translation uncertainty is shared by every anchor.
inline gtsam::Matrix map_frame_anchor_jacobian(const gtsam::Vector4& frame, const std::vector<gtsam::Point3>& anchors) {
  if (anchors.empty() || anchors.size() > 18) throw std::invalid_argument("MAP_FRAME_EXPORT_ANCHOR_COUNT");
  const auto rotation = gravity_frame_pose(frame).rotation().matrix();
  gtsam::Matrix H = gtsam::Matrix::Zero(3 * anchors.size(), 4 + 3 * anchors.size());
  for (size_t i = 0; i < anchors.size(); ++i) {
    if (!anchors[i].allFinite()) throw std::invalid_argument("MAP_FRAME_EXPORT_NONFINITE_ANCHOR");
    H.block<3, 1>(3 * i, 0) = .001 * rotation * gtsam::Vector3::UnitZ().cross(anchors[i]);
    H.block<3, 3>(3 * i, 1).setIdentity();
    H.block<3, 3>(3 * i, 4 + 3 * i) = rotation;
  }
  return H;
}

// Correspondences provide a numerical starting point only. This routine never
// creates a pose observation or claims that the alignment is observable.
inline gtsam::Vector4 gravity_frame_seed(const std::vector<gtsam::Point3>& local, const std::vector<gtsam::Point3>& map) {
  if (local.size() != map.size() || local.size() < 2 || local.size() > 18) throw std::invalid_argument("MAP_FRAME_SEED_COUNT");
  gtsam::Point3 l = gtsam::Point3::Zero(), m = gtsam::Point3::Zero();
  for (size_t i = 0; i < local.size(); ++i) {
    if (!local[i].allFinite() || !map[i].allFinite()) throw std::invalid_argument("MAP_FRAME_SEED_NONFINITE");
    l += local[i];
    m += map[i];
  }
  l /= local.size();
  m /= map.size();
  double dot = 0, cross = 0;
  for (size_t i = 0; i < local.size(); ++i) {
    const auto a = (local[i] - l).eval(), b = (map[i] - m).eval();
    dot += a.x() * b.x() + a.y() * b.y();
    cross += a.x() * b.y() - a.y() * b.x();
  }
  if (std::hypot(dot, cross) < 1e-9) throw std::invalid_argument("MAP_FRAME_SEED_HORIZONTAL_BASELINE_MISSING");
  const double yaw = std::atan2(cross, dot);
  gtsam::Vector4 result;
  result[0] = yaw * 1000.;
  result.tail<3>() = m - gtsam::Rot3::Rz(yaw).rotate(l);
  return result;
}

// ONE correlated map prior in its original map coordinates. Local A and the
// frame are joint unknowns; neither estimated local anchors nor LIO poses are
// fed back as independent measurements. Do not add CalibratedMapPrior as well.
class MapFramePrior : public gtsam::NoiseModelFactor {
public:
  MapFramePrior(gtsam::Key frame, const CalibratedMap& map)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(map.covariance), factor_keys(frame, map)),
    mean_(map.mean_vector()),
    count_(map.ids.size()) {
    if (map.has_reference) throw std::invalid_argument("MAP_FRAME_REQUIRES_ANCHOR_ONLY_MAP");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& values, gtsam::OptionalMatrixVecType H = nullptr) const override {
    const auto frame = values.at<gtsam::Vector4>(keys()[0]);
    const auto pose = gravity_frame_pose(frame);
    const auto R = pose.rotation().matrix();
    gtsam::Vector residual = -mean_;
    if (H) {
      H->clear();
      H->push_back(gtsam::Matrix::Zero(mean_.size(), 4));
      for (size_t i = 0; i < count_; ++i) H->push_back(gtsam::Matrix::Zero(mean_.size(), 3));
    }
    for (size_t i = 0; i < count_; ++i) {
      const auto point = values.at<gtsam::Point3>(keys()[i + 1]);
      residual.segment<3>(3 * i) += pose.transformFrom(point);
      if (H) {
        (*H)[0].block<3, 1>(3 * i, 0) = .001 * R * gtsam::Vector3::UnitZ().cross(point);
        (*H)[0].block<3, 3>(3 * i, 1).setIdentity();
        (*H)[i + 1].block<3, 3>(3 * i, 0) = R;
      }
    }
    return residual;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<MapFramePrior>(*this); }

private:
  static gtsam::KeyVector factor_keys(gtsam::Key frame, const CalibratedMap& map) {
    auto keys = map.keys();
    if (std::find(keys.begin(), keys.end(), frame) != keys.end()) throw std::invalid_argument("MAP_FRAME_KEY_COLLISION");
    keys.insert(keys.begin(), frame);
    return keys;
  }
  gtsam::Vector mean_;
  size_t count_;
};
}  // namespace liuo_tunnel
