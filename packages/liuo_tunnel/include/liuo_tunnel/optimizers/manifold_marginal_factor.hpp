#pragma once
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

namespace liuo_tunnel::solver {
// The original marginal quadratic and its reference NEVER move. Only its
// Jacobian is expressed in the solver's current right-retraction coordinates.
// This retains the same branch-local marginal approximation, including all
// cross blocks, without freezing the active variables' linearization points.
class ManifoldMarginalFactor : public gtsam::LinearContainerFactor {
public:
  ManifoldMarginalFactor(const gtsam::GaussianFactor::shared_ptr& factor, const gtsam::Values& reference) : LinearContainerFactor(factor, reference) {
    if (!hasLinearizationPoint()) throw std::invalid_argument("MARGINAL_REFERENCE_REQUIRED");
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& current) const override {
    const gtsam::HessianFactor original(*factor());
    const auto& reference = *linearizationPoint();
    std::vector<size_t> dimensions, offsets;
    size_t total = 0;
    for (const auto key : keys()) {
      offsets.push_back(total);
      dimensions.push_back(reference.at(key).dim());
      total += dimensions.back();
    }
    gtsam::Vector delta(total);
    gtsam::Matrix J = gtsam::Matrix::Zero(total, total);
    for (size_t i = 0; i < keys().size(); ++i) {
      const auto key = keys()[i];
      const auto& origin = reference.at(key);
      delta.segment(offsets[i], dimensions[i]) = origin.localCoordinates_(current.at(key));
      if (dynamic_cast<const gtsam::GenericValue<gtsam::Pose3>*>(&origin)) {
        gtsam::Matrix66 H;
        reference.at<gtsam::Pose3>(key).localCoordinates(current.at<gtsam::Pose3>(key), {}, H);
        J.block(offsets[i], offsets[i], 6, 6) = H;
      } else if (
        dynamic_cast<const gtsam::GenericValue<gtsam::Vector>*>(&origin) || dynamic_cast<const gtsam::GenericValue<gtsam::Vector2>*>(&origin) ||
        dynamic_cast<const gtsam::GenericValue<gtsam::Vector3>*>(&origin) || dynamic_cast<const gtsam::GenericValue<gtsam::Vector4>*>(&origin) ||
        dynamic_cast<const gtsam::GenericValue<gtsam::Vector6>*>(&origin) || dynamic_cast<const gtsam::GenericValue<gtsam::imuBias::ConstantBias>*>(&origin)) {
        J.block(offsets[i], offsets[i], dimensions[i], dimensions[i]).setIdentity();
      } else {
        throw std::invalid_argument("UNSUPPORTED_MARGINAL_MANIFOLD");
      }
    }
    const gtsam::Matrix augmented = original.augmentedInformation();
    const gtsam::Matrix H = augmented.topLeftCorner(total, total);
    const gtsam::Vector g = augmented.topRightCorner(total, 1);
    const gtsam::Matrix transported = J.transpose() * H * J;
    const gtsam::Vector gradient = J.transpose() * (g - H * delta);
    const double constant = original.constantTerm() - 2 * g.dot(delta) + delta.dot(H * delta);
    std::vector<gtsam::Matrix> blocks;
    std::vector<gtsam::Vector> vectors;
    for (size_t i = 0; i < keys().size(); ++i) {
      vectors.push_back(gradient.segment(offsets[i], dimensions[i]));
      for (size_t j = i; j < keys().size(); ++j) blocks.push_back(transported.block(offsets[i], offsets[j], dimensions[i], dimensions[j]));
    }
    return gtsam::make_shared<gtsam::HessianFactor>(keys(), blocks, vectors, constant);
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::make_shared<ManifoldMarginalFactor>(factor()->clone(), *linearizationPoint()); }
};
}  // namespace liuo_tunnel::solver
