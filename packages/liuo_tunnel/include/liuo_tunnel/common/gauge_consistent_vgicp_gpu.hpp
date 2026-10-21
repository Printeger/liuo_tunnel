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
// differential is dD = -Ad(D^-1)*dTarget + dSource. Preserve the GPU source block,
// gradient, cost, correspondences and all nonlinear evaluations; reconstruct the
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
    double weight = 1.0)
  : IntegratedVGICPFactorGPU(target_key, source_key, target, source, stream, buffer),
    source_(source),
    stream_(stream),
    buffer_(buffer),
    weight_(weight) {
    if (!std::isfinite(weight_) || weight_ <= 0 || weight_ > 1) throw std::invalid_argument("invalid native VGICP cost weight");
  }
  double error(const gtsam::Values& values) const override { return weight_ * IntegratedVGICPFactorGPU::error(values); }
  gtsam::GaussianFactor::shared_ptr linearize(const gtsam::Values& values) const override {
    auto original = IntegratedVGICPFactorGPU::linearize(values);
    gtsam::HessianFactor h(*original);
    const gtsam::Matrix66 Hs = weight_ * h.hessianBlockDiagonal().at(keys()[1]);
    const gtsam::Vector6 gs = weight_ * h.linearTerm(h.begin() + 1);
    const gtsam::Matrix66 J = -values.at<gtsam::Pose3>(keys()[0]).between(values.at<gtsam::Pose3>(keys()[1])).inverse().AdjointMap();
    return gtsam::make_shared<gtsam::HessianFactor>(keys()[0], keys()[1], J.transpose() * Hs * J, J.transpose() * Hs, J.transpose() * gs, Hs, gs, weight_ * h.constantTerm());
  }
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::make_shared<GaugeConsistentVGICPFactorGPU>(keys()[0], keys()[1], get_target(), source_, stream_, buffer_, weight_);
  }

private:
  gtsam_points::PointCloud::ConstPtr source_;
  CUstream_st* stream_;
  std::shared_ptr<gtsam_points::TempBufferManager> buffer_;
  double weight_;
};
}  // namespace liuo_tunnel
