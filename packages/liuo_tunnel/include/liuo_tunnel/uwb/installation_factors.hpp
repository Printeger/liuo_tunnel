#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <stdexcept>
#include <cmath>
namespace liuo_tunnel {
// One correlated installation observation. L=[normal angle, offset relative to origin].
// origin is a fixed numerical coordinate choice, not a measured centerline prior.
// A retains its existing map prior; no pre-fitted line prior is added.
class InstallationOffsetFactor : public gtsam::NoiseModelFactor {
public:
  InstallationOffsetFactor(
    gtsam::Key reference,
    const gtsam::KeyVector& anchors,
    const gtsam::Vector& signed_offsets,
    const gtsam::Matrix& covariance,
    const gtsam::Point3& origin = gtsam::Point3::Zero())
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(covariance), keys(reference, anchors)),
    offsets_(signed_offsets),
    origin_(origin) {
    if (anchors.size() != size_t(offsets_.size()) || covariance.rows() != offsets_.size() || covariance.cols() != offsets_.size() || !offsets_.allFinite())
      throw std::invalid_argument("INSTALLATION_FACTOR_DIMENSION");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& values, gtsam::OptionalMatrixVecType H = nullptr) const override {
    const auto& line = values.at<gtsam::Vector2>(keys()[0]);
    const gtsam::Vector3 n(std::cos(line[0]), std::sin(line[0]), 0), tangent(-n.y(), n.x(), 0);
    gtsam::Vector residual(offsets_.size());
    if (H) {
      H->assign(keys().size(), gtsam::Matrix::Zero(offsets_.size(), 3));
      (*H)[0] = gtsam::Matrix::Zero(offsets_.size(), 2);
    }
    for (int i = 0; i < offsets_.size(); ++i) {
      const gtsam::Point3 anchor = values.at<gtsam::Point3>(keys()[i + 1]) - origin_;
      residual[i] = offsets_[i] - n.dot(anchor) + line[1];
      if (H) {
        (*H)[0].row(i) << -tangent.dot(anchor), 1;
        (*H)[i + 1].row(i) = -n.transpose();
      }
    }
    return residual;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<InstallationOffsetFactor>(*this); }

private:
  static gtsam::KeyVector keys(gtsam::Key reference, const gtsam::KeyVector& anchors) {
    gtsam::KeyVector result{reference};
    result.insert(result.end(), anchors.begin(), anchors.end());
    return result;
  }
  using gtsam::NoiseModelFactor::keys;
  gtsam::Vector offsets_;
  gtsam::Point3 origin_;
};
// One dedicated cloud, never also used by VGICP. The unoriented wall normal
// leaves both tunnel directions valid. No vertical or axial position datum.
class WallReferenceFactor : public gtsam::NoiseModelFactor {
public:
  WallReferenceFactor(
    gtsam::Key pose,
    gtsam::Key reference,
    const gtsam::Vector3& body_normal,
    const gtsam::Point3& body_center,
    const gtsam::Matrix2& covariance,
    const gtsam::Point3& origin = gtsam::Point3::Zero())
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(covariance), gtsam::KeyVector{pose, reference}),
    normal_(body_normal.normalized()),
    center_(body_center),
    origin_(origin) {
    if (!body_normal.allFinite() || body_normal.norm() < 1e-8 || !body_center.allFinite()) throw std::invalid_argument("INVALID_BODY_WALL");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& values, gtsam::OptionalMatrixVecType H = nullptr) const override {
    const auto& pose = values.at<gtsam::Pose3>(keys()[0]);
    const auto& line = values.at<gtsam::Vector2>(keys()[1]);
    gtsam::Matrix33 Dnormal;
    const auto world_normal = pose.rotation().rotate(normal_, Dnormal);
    const double length2 = world_normal.head<2>().squaredNorm();
    if (length2 < 1e-10) throw std::domain_error("VERTICAL_WALL_NORMAL_INVALID");
    gtsam::Matrix36 Dcenter;
    const gtsam::Point3 world_center = pose.transformFrom(center_, Dcenter) - origin_;
    const gtsam::Vector3 n(std::cos(line[0]), std::sin(line[0]), 0), tangent(-n.y(), n.x(), 0);
    double angle = line[0] - std::atan2(world_normal.y(), world_normal.x());
    angle = .5 * std::atan2(std::sin(2 * angle), std::cos(2 * angle));
    if (H) {
      H->assign(2, gtsam::Matrix::Zero(2, 2));
      (*H)[0] = gtsam::Matrix::Zero(2, 6);
      const Eigen::RowVector3d Dangle(-world_normal.y() / length2, world_normal.x() / length2, 0);
      (*H)[0].block<1, 3>(0, 0) = -Dangle * Dnormal;
      (*H)[0].row(1) = -n.transpose() * Dcenter;
      (*H)[1] << 1, 0, -tangent.dot(world_center), 1;
    }
    return gtsam::Vector2(angle, line[1] - n.dot(world_center));
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<WallReferenceFactor>(*this); }

private:
  gtsam::Vector3 normal_;
  gtsam::Point3 center_, origin_;
};
}  // namespace liuo_tunnel
