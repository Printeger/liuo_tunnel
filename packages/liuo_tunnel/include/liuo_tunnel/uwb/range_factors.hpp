#pragma once
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <optional>
#include <stdexcept>

namespace liuo_tunnel {
// Linear world translation and SO(3) geodesic interpolation, recomputed at each
// linearization. Pose3 tangent order is right-local [rotation, translation].
inline gtsam::Point3 interpolated_tag(
  const gtsam::Pose3& left,
  const gtsam::Pose3& right,
  double beta,
  const gtsam::Point3& lever,
  gtsam::Matrix36* Hleft = nullptr,
  gtsam::Matrix36* Hright = nullptr) {
  if (!std::isfinite(beta) || beta < 0 || beta > 1) throw std::invalid_argument("invalid interpolation fraction");
  gtsam::Matrix33 Dleft, Dright, Dlog, Dexp, Cleft, Cexp;
  auto relative = left.rotation().between(right.rotation(), Dleft, Dright);
  auto tangent = gtsam::Rot3::Logmap(relative, Dlog);
  auto increment = gtsam::Rot3::Expmap(beta * tangent, Dexp);
  auto rotation = left.rotation().compose(increment, Cleft, Cexp);
  const auto chain = (Cexp * Dexp * beta * Dlog).eval();
  const auto lever_jac = (-rotation.matrix() * gtsam::skewSymmetric(lever)).eval();
  if (Hleft) {
    Hleft->leftCols<3>() = lever_jac * (Cleft + chain * Dleft);
    Hleft->rightCols<3>() = (1 - beta) * left.rotation().matrix();
  }
  if (Hright) {
    Hright->leftCols<3>() = lever_jac * chain * Dright;
    Hright->rightCols<3>() = beta * right.rotation().matrix();
  }
  return (1 - beta) * left.translation() + beta * right.translation() + rotation.rotate(lever);
}

// Stationary accelerometer direction supplies two attitude constraints only.
// No yaw or position prior is smuggled in as a map-frame initialization.
class GravityAlignmentFactor : public gtsam::NoiseModelFactor {
public:
  GravityAlignmentFactor(gtsam::Key pose, const gtsam::Vector3& measured_up, double sigma)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(3, sigma), gtsam::KeyVector{pose}),
    up_(measured_up.normalized()) {
    if (!measured_up.allFinite() || measured_up.norm() < 1e-8) throw std::invalid_argument("invalid gravity alignment");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& v, gtsam::OptionalMatrixVecType H = nullptr) const override {
    gtsam::Matrix33 D;
    auto predicted = v.at<gtsam::Pose3>(keys()[0]).rotation().unrotate(gtsam::Vector3::UnitZ(), D);
    if (H) {
      H->assign(1, gtsam::Matrix::Zero(3, 6));
      (*H)[0].leftCols<3>() = -D;
    }
    return up_ - predicted;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<GravityAlignmentFactor>(*this); }

private:
  gtsam::Vector3 up_;
};

// Keys: [X_left, optional X_right, optional A]. No fixed tag position cache.
class TagAnchorRangeFactor : public gtsam::NoiseModelFactor {
public:
  TagAnchorRangeFactor(
    gtsam::Key left,
    std::optional<gtsam::Key> right,
    std::optional<gtsam::Key> anchor,
    const gtsam::Point3& fixed_anchor,
    const gtsam::Point3& lever,
    double beta,
    double measurement,
    double sigma)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(1, sigma), make_keys(left, right, anchor)),
    right_(right),
    anchor_(anchor),
    fixed_(fixed_anchor),
    lever_(lever),
    beta_(beta),
    measured_(measurement) {
    if (
      !lever.allFinite() || !fixed_anchor.allFinite() || !std::isfinite(measurement) || measurement <= 0 || !std::isfinite(sigma) || sigma <= 0 || !std::isfinite(beta) ||
      beta < 0 || beta > 1 || (!right && beta != 0))
      throw std::invalid_argument("invalid range factor parameters");
    if (right && *right == left) throw std::invalid_argument("duplicate pose key; use synchronous factor");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& values, gtsam::OptionalMatrixVecType H = nullptr) const override {
    const auto& left = values.at<gtsam::Pose3>(keys()[0]);
    gtsam::Matrix36 Hl, Hr;
    gtsam::Point3 tag;
    if (right_)
      tag = interpolated_tag(left, values.at<gtsam::Pose3>(*right_), beta_, lever_, &Hl, &Hr);
    else
      tag = left.transformFrom(lever_, Hl);
    const auto anchor = anchor_ ? values.at<gtsam::Point3>(*anchor_) : fixed_;
    const gtsam::Vector3 delta = tag - anchor;
    double distance = delta.norm();
    if (!std::isfinite(distance) || distance < 1e-8) throw std::domain_error("RANGE_GEOMETRY_ZERO");
    const auto direction = (delta / distance).transpose().eval();
    if (H) {
      H->resize(keys().size());
      (*H)[0] = -direction * Hl;
      if (right_) (*H)[1] = -direction * Hr;
      if (anchor_) H->back() = direction;
    }
    return gtsam::Vector1(measured_ - distance);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<TagAnchorRangeFactor>(*this); }

private:
  static gtsam::KeyVector make_keys(gtsam::Key l, std::optional<gtsam::Key> r, std::optional<gtsam::Key> a) {
    gtsam::KeyVector keys{l};
    if (r) keys.push_back(*r);
    if (a) keys.push_back(*a);
    return keys;
  }
  std::optional<gtsam::Key> right_, anchor_;
  gtsam::Point3 fixed_, lever_;
  double beta_, measured_;
};

class AnchorAnchorRangeFactor : public gtsam::NoiseModelFactor {
public:
  AnchorAnchorRangeFactor(gtsam::Key a, gtsam::Key b, double measurement, double sigma)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Isotropic::Sigma(1, sigma), gtsam::KeyVector{a, b}),
    measured_(measurement) {
    if (a == b || !std::isfinite(measurement) || measurement <= 0 || !std::isfinite(sigma) || sigma <= 0) throw std::invalid_argument("invalid AA factor");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& v, gtsam::OptionalMatrixVecType H = nullptr) const override {
    const gtsam::Vector3 delta = v.at<gtsam::Point3>(keys()[0]) - v.at<gtsam::Point3>(keys()[1]);
    const double distance = delta.norm();
    if (!std::isfinite(distance) || distance < 1e-8) throw std::domain_error("RANGE_GEOMETRY_ZERO");
    if (H) {
      H->resize(2);
      (*H)[0] = -delta.transpose() / distance;
      (*H)[1] = delta.transpose() / distance;
    }
    return gtsam::Vector1(measured_ - distance);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<AnchorAnchorRangeFactor>(*this); }

private:
  double measured_;
};

// One prior over ALL provided map coordinates, retaining off-diagonal covariance.
class SurveyedMapPrior : public gtsam::NoiseModelFactor {
public:
  SurveyedMapPrior(const gtsam::KeyVector& keys, const std::vector<gtsam::Point3>& means, const gtsam::Matrix& covariance)
  : gtsam::NoiseModelFactor(gtsam::noiseModel::Gaussian::Covariance(covariance), keys),
    means_(means) {
    if (keys.empty() || keys.size() != means.size() || covariance.rows() != int(3 * keys.size()) || covariance.cols() != covariance.rows())
      throw std::invalid_argument("map prior dimensions");
  }
  gtsam::Vector unwhitenedError(const gtsam::Values& v, gtsam::OptionalMatrixVecType H = nullptr) const override {
    gtsam::Vector error(3 * keys().size());
    if (H) H->assign(keys().size(), gtsam::Matrix::Zero(error.size(), 3));
    for (size_t i = 0; i < keys().size(); i++) {
      error.segment<3>(3 * i) = means_[i] - v.at<gtsam::Point3>(keys()[i]);
      if (H) (*H)[i].block<3, 3>(3 * i, 0) = -gtsam::Matrix33::Identity();
    }
    return error;
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<SurveyedMapPrior>(*this); }

private:
  std::vector<gtsam::Point3> means_;
};
}  // namespace liuo_tunnel
