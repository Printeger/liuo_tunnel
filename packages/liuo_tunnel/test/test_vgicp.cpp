#include <gtest/gtest.h>
#include <liuo_tunnel/common/gauge_consistent_vgicp_gpu.hpp>
#include <liuo_tunnel/common/cloud_covariance_estimation.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_cpu.hpp>
#include <gtsam_points/ann/kdtree.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
TEST(Vgicp, VariablePoseFactorRecoversTranslatedGeometry) {
  // Offset the planes away from voxel boundaries: CPU VGICP uses same-cell lookup.
  // A plane exactly on a boundary can have no correspondences for one sign of prediction error.
  std::vector<Eigen::Vector4d> pts;
  for (int i = -12; i <= 12; i++)
    for (int j = -12; j <= 12; j++) {
      pts.emplace_back(i * .13, j * .13, .173, 1);
      pts.emplace_back(.237, i * .13, j * .13, 1);
      pts.emplace_back(i * .13, .321, j * .13, 1);
    }
  auto make = [](const std::vector<Eigen::Vector4d>& p) {
    gtsam_points::KdTree tree(p.data(), p.size());
    std::vector<int> neighbors(p.size() * 10);
    for (size_t i = 0; i < p.size(); i++) {
      size_t ids[10];
      double ds[10];
      tree.knn_search(p[i].data(), 10, ids, ds);
      for (int k = 0; k < 10; k++) neighbors[i * 10 + k] = ids[k];
    }
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(p);
    liuo_tunnel::CloudCovarianceEstimation cov;
    cloud->add_covs(cov.estimate(p, neighbors));
    return cloud;
  };
  auto target = make(pts);
  auto voxel = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(.5);
  voxel->insert(*target);
  for (auto& p : pts) p.x() -= .3;
  auto source = make(pts);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam_points::IntegratedVGICPFactor>(0, 1, voxel, source);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
  gtsam::Values values;
  values.insert(0, gtsam::Pose3());
  values.insert(1, gtsam::Pose3(gtsam::Rot3(), {.25, .01, .01}));

  auto out = gtsam::LevenbergMarquardtOptimizer(graph, values).optimize();
  EXPECT_NEAR(out.at<gtsam::Pose3>(1).translation().x(), .3, .03);
}
#include <gtsam/linear/HessianFactor.h>
TEST(Vgicp, UnsupportedPlaneDoesNotFabricateAxialInformation) {
  liuo_tunnel::CloudCovarianceEstimation original(1, false), guarded(1, true);
  Eigen::Matrix4d line = Eigen::Matrix4d::Zero();
  line(0, 0) = 1e-7;
  line(1, 1) = 2e-7;
  line(2, 2) = 1;
  auto old_cov = original.regularize(line), new_cov = guarded.regularize(line);
  EXPECT_GT((old_cov.topLeftCorner<3, 3>().inverse().norm()), new_cov.norm());
  std::vector<Eigen::Vector4d> points{{.15, .15, .15, 1}, {.15, .15, .25, 1}, {.15, .15, .35, 1}};
  auto block = [&](const Eigen::Matrix4d& cov) {
    auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
    cloud->add_covs(std::vector<Eigen::Matrix4d>(points.size(), cov));
    auto voxel = std::make_shared<gtsam_points::GaussianVoxelMapCPU>(.5);
    voxel->insert(*cloud);
    gtsam_points::IntegratedVGICPFactor factor(0, 1, voxel, cloud);
    gtsam::Values values;
    values.insert(0, gtsam::Pose3());
    values.insert(1, gtsam::Pose3());
    gtsam::HessianFactor h(*factor.linearize(values));
    return h.hessianBlockDiagonal().at(1).eval();
  };
  auto old_h = block(old_cov), new_h = block(new_cov);
  EXPECT_GT(old_h(3, 3), new_h(3, 3) * 100);
  Eigen::Matrix4d plane = Eigen::Matrix4d::Zero();
  plane(0, 0) = .1;
  plane(1, 1) = .2;
  plane(2, 2) = 1e-7;
  EXPECT_NEAR(guarded.regularize(plane)(2, 2), .001, 1e-10);
}

#include <gtsam_points/types/point_cloud_gpu.hpp>
#include <gtsam_points/types/gaussian_voxelmap_gpu.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_gpu.hpp>
#include <gtsam_points/cuda/nonlinear_factor_set_gpu.hpp>
#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
TEST(VgicpGPU, BinaryRawFactorRecoversTranslation) {
  gtsam_points::LinearizationHook::register_hook([] { return std::make_shared<gtsam_points::NonlinearFactorSetGPU>(); });
  std::vector<Eigen::Vector4d> points;
  for (int i = -12; i <= 12; i++)
    for (int j = -12; j <= 12; j++) {
      points.emplace_back(i * .13, j * .13, .173, 1);
      points.emplace_back(.237, i * .13, j * .13, 1);
      points.emplace_back(i * .13, .321, j * .13, 1);
    }
  auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(points);
  cpu->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity() * .01));
  auto target = gtsam_points::PointCloudGPU::clone(*cpu);
  auto voxels = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  voxels->insert(*target);
  for (auto& p : points) p.x() -= .2;
  auto shifted = std::make_shared<gtsam_points::PointCloudCPU>(points);
  shifted->add_covs(std::vector<Eigen::Matrix4d>(points.size(), Eigen::Matrix4d::Identity() * .01));
  auto source = gtsam_points::PointCloudGPU::clone(*shifted);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam_points::IntegratedVGICPFactorGPU>(0, 1, voxels, source);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(0, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 1e-5));
  gtsam::Values initial;
  initial.insert(0, gtsam::Pose3());
  initial.insert(1, gtsam::Pose3(gtsam::Rot3(), {.17, .01, .01}));
  auto result = gtsam_points::LevenbergMarquardtOptimizerExt(graph, initial).optimize();
  EXPECT_NEAR(result.at<gtsam::Pose3>(1).x(), .2, .03);
  EXPECT_EQ(graph.at(0)->keys().size(), 2);
}

TEST(VgicpGPU, BinaryFactorMustNotObserveCommonRigidMotion) {
  std::vector<Eigen::Vector4d> points;
  for (int i = -20; i <= 20; i++)
    for (int j = -12; j <= 12; j++) {
      points.emplace_back(i * .37, j * .13, .173, 1);
      points.emplace_back(.237, i * .37, j * .13, 1);
      points.emplace_back(i * .37, .321, j * .13, 1);
    }
  auto make = [](const std::vector<Eigen::Vector4d>& p) {
    auto cpu = std::make_shared<gtsam_points::PointCloudCPU>(p);
    cpu->add_covs(std::vector<Eigen::Matrix4d>(p.size(), Eigen::Matrix4d::Identity() * .01));
    return gtsam_points::PointCloudGPU::clone(*cpu);
  };
  auto target = make(points);
  auto voxels = std::make_shared<gtsam_points::GaussianVoxelMapGPU>(.5);
  voxels->insert(*target);
  gtsam::Pose3 delta(gtsam::Rot3::RzRyRx(.03, -.04, .02), {.2, .1, -.1});
  for (auto& point : points) point.head<3>() = delta.transformTo(gtsam::Point3(point.head<3>()));
  auto source = make(points);
  auto factor = gtsam::make_shared<liuo_tunnel::GaugeConsistentVGICPFactorGPU>(0, 1, voxels, source);
  gtsam::Pose3 world(gtsam::Rot3::RzRyRx(.2, -.3, .5), {100, 20, 3});
  gtsam::Values values;
  values.insert(0, world);
  values.insert(1, world.compose(delta));
  gtsam_points::NonlinearFactorSetGPU batch;
  batch.add(factor);
  batch.linearize(values);
  gtsam::HessianFactor h(*factor->linearize(values));
  Eigen::Matrix<double, 12, 6> gauge;
  gauge.topRows<6>().setIdentity();
  gauge.bottomRows<6>() = delta.inverse().AdjointMap();
  auto information = h.information();
  double defect = (information * gauge).norm() / information.norm();
  std::cout << "GPU common-motion relative information defect=" << defect << std::endl;
  EXPECT_LT(defect, 1e-12);
  auto native = gtsam::make_shared<gtsam_points::IntegratedVGICPFactorGPU>(0, 1, voxels, source);
  gtsam_points::NonlinearFactorSetGPU reference;
  reference.add(native);
  reference.linearize(values);
  gtsam::HessianFactor original(*native->linearize(values));
  EXPECT_LT((h.hessianBlockDiagonal().at(1) - original.hessianBlockDiagonal().at(1)).norm(), 1e-10);
  EXPECT_LT((h.linearTerm(h.begin() + 1) - original.linearTerm(original.begin() + 1)).norm(), 1e-10);
  EXPECT_DOUBLE_EQ(h.constantTerm(), original.constantTerm());
  EXPECT_LT((h.information() - original.information()).norm() / original.information().norm(), 1e-6);
  EXPECT_NEAR(factor->error(values), native->error(values), 1e-8);
}
