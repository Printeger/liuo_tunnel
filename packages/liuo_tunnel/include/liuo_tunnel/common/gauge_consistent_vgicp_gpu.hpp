#pragma once
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace liuo_tunnel {
// Public GPU kernels accumulate target, source and cross blocks independently in
// float. Their roundoff can inject information into the EXACT six-dimensional
// common-motion nullspace. Weak surveyed/range gauges expose this defect.
//
// The original cost depends only on D=T_target^-1*T_source. Its right-local
// differential is dD = -Ad(D^-1)*dTarget + dSource. In default mode preserve
// the GPU source block, gradient, cost and correspondences; reconstruct the
// other blocks by this identity in double. No prior, damping, eigenvalue clipping,
// registration-result observation is introduced. An explicit positive scalar
// weight, default 1, scales the entire original objective and derivatives
// consistently; it does not change correspondences or add constraints.
class GaugeConsistentVGICPFactorGPU : public gtsam_points::IntegratedVGICPFactorGPU {
public:
  GaugeConsistentVGICPFactorGPU(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const gtsam_points::GaussianVoxelMap::ConstPtr& target,
    const gtsam_points::PointCloud::ConstPtr& source,
    CUstream_st* stream = nullptr,
    std::shared_ptr<gtsam_points::TempBufferManager> buffer = nullptr,
    double weight = 1.0,
    const Eigen::Matrix3d& translation_projector = Eigen::Matrix3d::Identity())
  : IntegratedVGICPFactorGPU(target_key, source_key, target, source, stream, buffer),
    source_(source),
    stream_(stream),
    buffer_(buffer),
    weight_(weight),
    source_projector_(translation_projector),
    projecting_((translation_projector - Eigen::Matrix3d::Identity()).norm() > 1e-9) {
    if (
      !translation_projector.allFinite() || (translation_projector - translation_projector.transpose()).norm() > 1e-8 ||
      (translation_projector * translation_projector - translation_projector).norm() > 1e-8)
      throw std::invalid_argument("invalid translation projector");
    if (!std::isfinite(weight_) || weight_ <= 0 || weight_ > 1) throw std::invalid_argument("invalid native VGICP cost weight");
  }
  // Freeze only the unobservable relative translation at the current ICP
  // linearization, like frozen correspondences. This adds NO information in
  // that direction: both nonlinear trial cost and its Jacobian are projected.
  static gtsam::Pose3 project_relative(const gtsam::Pose3& relative, const gtsam::Vector3& reference, const Eigen::Matrix3d& target_projector) {
    return gtsam::Pose3(relative.rotation(), reference + target_projector * (relative.translation() - reference));
  }
  void set_linearization_point(const gtsam::Values& values, void* input) override {
    set_reference(values);
    IntegratedVGICPFactorGPU::set_linearization_point(values, input);
  }
  void set_evaluation_point(const gtsam::Values& values, void* input) override {
    if (projecting_)
      IntegratedVGICPFactorGPU::set_evaluation_point(projected_values(values), input);
    else
      IntegratedVGICPFactorGPU::set_evaluation_point(values, input);
  }
  double error(const gtsam::Values& values) const override {
    if (projecting_) return weight_ * IntegratedVGICPFactorGPU::error(projected_values(values));
    return weight_ * IntegratedVGICPFactorGPU::error(values);
  }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override {
    set_reference(values);
    auto original = IntegratedVGICPFactorGPU::linearize(values);
    gtsam::HessianFactor h(*original);
    gtsam::Matrix66 S = gtsam::Matrix66::Identity();
    S.bottomRightCorner<3, 3>() = source_projector_;
    const gtsam::Matrix66 Hs = weight_ * S.transpose() * h.hessianBlockDiagonal().at(keys()[1]) * S;
    const gtsam::Vector6 gs = weight_ * S.transpose() * h.linearTerm(h.begin() + 1);
    const gtsam::Matrix66 J = -values.at<gtsam::Pose3>(keys()[0]).between(values.at<gtsam::Pose3>(keys()[1])).inverse().AdjointMap();
    return gtsam::make_shared<gtsam::HessianFactor>(keys()[0], keys()[1], J.transpose() * Hs * J, J.transpose() * Hs, J.transpose() * gs, Hs, gs, weight_ * h.constantTerm());
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::make_shared<GaugeConsistentVGICPFactorGPU>(keys()[0], keys()[1], get_target(), source_, stream_, buffer_, weight_, source_projector_);
  }

private:
  void set_reference(const gtsam::Values& values) const {
    if (!projecting_) return;
    const auto relative = values.at<gtsam::Pose3>(keys()[0]).between(values.at<gtsam::Pose3>(keys()[1]));
    reference_translation_ = relative.translation();
    target_projector_ = relative.rotation().matrix() * source_projector_ * relative.rotation().matrix().transpose();
    reference_ready_ = true;
  }
  gtsam::Values projected_values(const gtsam::Values& values) const {
    if (!reference_ready_) throw std::runtime_error("TRANSLATION_PROJECTION_BEFORE_LINEARIZATION");
    const auto& target = values.at<gtsam::Pose3>(keys()[0]);
    const auto relative = target.between(values.at<gtsam::Pose3>(keys()[1]));
    gtsam::Values projected;
    projected.insert(keys()[0], target);
    projected.insert(keys()[1], target.compose(project_relative(relative, reference_translation_, target_projector_)));
    return projected;
  }
  mutable bool reference_ready_ = false;
  mutable gtsam::Vector3 reference_translation_;
  mutable Eigen::Matrix3d target_projector_;
  gtsam_points::PointCloud::ConstPtr source_;
  CUstream_st* stream_;
  std::shared_ptr<gtsam_points::TempBufferManager> buffer_;
  double weight_;
  Eigen::Matrix3d source_projector_;
  bool projecting_;
};
}  // namespace liuo_tunnel
